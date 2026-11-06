#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SIYI 云台 UDP 中转服务（跑在机载电脑 / Jetson 上）—— v4

包含两个对外服务：

  1. UDP 37260 —— 云台 SDK 指令中转（地面站 ↔ 相机）
  2. TCP 37261 —— 相机的 HTTP 文件接口代理（照片/视频的列表与下载）

背景（实测踩坑记录，2026-10-07）：
  1. A8 mini 的 UDP SDK 是【单客户端】模型：它把会话绑在一个源端口上。
     v1 版本为每个下游客户端新建一条上游 socket，源端口不断变化，
     导致相机应答错乱并最终挂死（约 9 分钟无响应）。
     → v2 改为【全局唯一的上游 socket】，源端口恒定。
  2. SDK 文档明确写着心跳包（CMD 0x00）【仅 TCP 连接时支持】。
     实测对 UDP 发送 0x00 会让相机的命令服务挂死（TCP 端口也变为拒绝连接），
     恢复办法是发送 0x80 软重启并等待约 20~40 秒。
     → v2 直接【丢弃所有 0x00 帧】，不转发给相机。
  3. 拍照/录像的真实结果由相机通过 CMD 0x0B 主动回传，
     0x0C 本身是"无 ACK"的（地面站据此显示真实成败）。
  4. 地面站状态轮询和机载航点云台执行器共用唯一上游 socket。
     SDK 回复与异步反馈回传给最近 10 秒内的有效客户端（最多 8 个），
     避免后发的状态查询抢走另一客户端的姿态应答或拍照反馈。

相机文件接口（HTTP，无需认证）：
  GET http://192.168.144.25:82/cgi-bin/media.cgi/api/v1/getdirectories?media_type=0
  GET .../getmediacount?media_type=0&path=101SIYI_IMG
  GET .../getmedialist?media_type=0&path=101SIYI_IMG&start=0&count=20
  GET http://192.168.144.25:82/photo/101SIYI_IMG/IMG_0001.jpg   ← 实际文件

协议帧：55 66 | control | len(2,LE) | seq(2,LE) | cmd(1) | payload | crc16(2,LE)
"""
import argparse
import binascii
import collections
import http.server
import select
import socket
import struct
import sys
import threading
import time
import urllib.error
import urllib.request

CAM_ADDR = ("192.168.144.25", 37260)
CAM_HTTP = "http://192.168.144.25:82"     # 相机 Web Server（文件接口）
LISTEN_ADDR = ("0.0.0.0", 37260)
PROXY_ADDR = ("0.0.0.0", 37261)           # HTTP 代理监听（给地面站用）
CMD_HEARTBEAT = 0x00          # 仅 TCP 支持，UDP 发送会导致相机挂死，必须丢弃
CMD_ROTATE = 0x07             # 云台转向（流式速度指令，必须显式发 0 才停）
ROTATE_DEADMAN_SEC = 1.0      # 地面站超过这个时间没有新的转向指令，就替它发停止
CLIENT_REPLY_TTL_SEC = 10.0   # 回复仅回传给最近发过有效 SDK 指令的客户端
MAX_REPLY_CLIENTS = 8        # 限制回复接收者数量，不随源端口数量无限增长
HTTP_TIMEOUT_SEC = 30.0
HTTP_CHUNK = 65536


def log(msg):
    print(time.strftime("[%Y-%m-%d %H:%M:%S] ") + msg, flush=True)


def command_id(data):
    """从帧里取命令号；解析失败返回 None。"""
    if len(data) < 8 or data[:2] != b"\x55\x66":
        return None
    return data[7]


def crc16(data):
    return binascii.crc_hqx(data, 0)


def build_frame(cmd, payload=b""):
    """构造一帧 SDK 指令（用途：替失联的地面站补发转向停止）。"""
    body = (b"\x55\x66\x01" + struct.pack("<H", len(payload))
            + struct.pack("<H", 0) + bytes((cmd,)) + payload)
    return body + struct.pack("<H", crc16(body))


def rotate_is_active(data):
    """0x07 的载荷是两个 int8 速度，非零即表示正在转向。"""
    return len(data) >= 10 and (data[8] != 0 or data[9] != 0)


def valid_frames(data):
    """Extract CRC-valid SDK frames, including bundled camera replies.

    Some camera datagrams contain several frames. Do not fan out arbitrary
    camera bytes, and do not let an embedded downstream heartbeat evade filtering.
    """
    frames = []
    offset = 0
    while offset + 10 <= len(data):
        offset = data.find(b"\x55\x66", offset)
        if offset < 0 or offset + 10 > len(data):
            break
        payload_size = struct.unpack_from("<H", data, offset + 3)[0]
        size = payload_size + 10
        if offset + size <= len(data):
            frame = data[offset:offset + size]
            expected = struct.unpack_from("<H", frame, len(frame) - 2)[0]
            if crc16(frame[:-2]) == expected:
                frames.append(frame)
                offset += size
                continue
        offset += 1
    return frames


class RecentClients:
    """Bounded recipients; only downstream commands renew their subscriptions."""

    def __init__(self, ttl=CLIENT_REPLY_TTL_SEC, limit=MAX_REPLY_CLIENTS):
        if ttl <= 0 or limit < 1:
            raise ValueError("client ttl and limit must be positive")
        self.ttl = ttl
        self.limit = limit
        self.clients = collections.OrderedDict()

    def active(self, now):
        while self.clients:
            oldest, last_seen = next(iter(self.clients.items()))
            if now - last_seen < self.ttl:
                break
            del self.clients[oldest]
        return tuple(self.clients)

    def touch(self, client, now):
        self.active(now)
        self.clients.pop(client, None)
        self.clients[client] = now
        while len(self.clients) > self.limit:
            self.clients.popitem(last=False)


def run_udp_relay(downstream, upstream, camera_addr=CAM_ADDR, stop_event=None,
                  clock=time.monotonic, client_ttl=CLIENT_REPLY_TTL_SEC,
                  max_clients=MAX_REPLY_CLIENTS, rotate_deadman=ROTATE_DEADMAN_SEC,
                  select_timeout=0.2):
    """Relay through one persistent upstream; injected sockets permit offline tests."""
    clients = RecentClients(client_ttl, max_clients)
    last_drop_log = -10.0
    forward_count = reply_count = 0
    rotate_active = False
    rotate_owner = None
    last_rotate_at = 0.0
    while stop_event is None or not stop_event.is_set():
        readable, _, _ = select.select([downstream, upstream], [], [], select_timeout)
        now = clock()
        clients.active(now)
        # Only a successfully forwarded velocity frame renews this deadline.
        # Other clients' queries/replies cannot keep a stale speed command alive.
        if rotate_active and now - last_rotate_at > rotate_deadman:
            try:
                upstream.sendto(build_frame(CMD_ROTATE, b"\x00\x00"), camera_addr)
                rotate_active = False
                log("!! 拨杆死手保护触发：%.1f 秒无转向指令，已发送停止（原控制端 %s）"
                    % (rotate_deadman, rotate_owner))
                rotate_owner = None
            except OSError as error:
                log("!! 死手保护发送失败: %s" % error)

        for sock in readable:
            if sock is downstream:
                try:
                    data, client = downstream.recvfrom(4096)
                except OSError:
                    continue
                for frame in valid_frames(data):
                    cmd = command_id(frame)
                    if cmd == CMD_HEARTBEAT:
                        if now - last_drop_log >= 10:
                            log("已丢弃心跳帧（CMD 0x00，来自 %s:%d）—— UDP 不支持心跳"
                                % (client[0], client[1]))
                            last_drop_log = now
                        continue
                    if cmd == CMD_ROTATE and len(frame) != 12:
                        continue  # A velocity frame must contain exactly two int8 values.
                    try:
                        # Commands (including zero-speed stop) are unicast once.
                        # Fanout is exclusively for validated camera replies.
                        upstream.sendto(frame, camera_addr)
                        clients.touch(client, now)
                        if cmd == CMD_ROTATE:
                            last_rotate_at = now
                            rotate_active = rotate_is_active(frame)
                            rotate_owner = client if rotate_active else None
                        forward_count += 1
                        log("-> #%d 转发 CMD 0x%02X（%d 字节）给相机，来自 %s:%d"
                            % (forward_count, cmd, len(frame), client[0], client[1]))
                    except OSError as error:
                        log("!! 转发失败: %s" % error)
            else:
                try:
                    data, source = upstream.recvfrom(4096)
                except OSError:
                    continue
                if source != camera_addr:
                    log("!! 忽略非相机来源 %s:%d" % (source[0], source[1]))
                    continue
                frames = valid_frames(data)
                if not frames:
                    continue
                recipients = clients.active(clock())
                if not recipients:
                    continue
                packet = b"".join(frames)
                reply_count += 1
                for client in recipients:
                    try:
                        downstream.sendto(packet, client)
                    except OSError as error:
                        log("!! 回传给 %s:%d 失败: %s" % (client[0], client[1], error))
                log("<- #%d 回传 %d 帧（%d 字节）给 %d 个近期客户端"
                    % (reply_count, len(frames), len(packet), len(recipients)))


class CameraHttpProxy(http.server.BaseHTTPRequestHandler):
    """把地面站的 HTTP 请求透传给相机 Web Server（端口 82）。

    地面站与相机不同网段，照片/视频的列表与下载都只能经由机载电脑。
    这里只做只读透传：GET/HEAD 原样转发，路径照搬。
    """

    server_version = "siyi-cam-proxy/1.0"
    protocol_version = "HTTP/1.1"

    def _forward(self, send_body):
        target = CAM_HTTP + self.path
        try:
            request = urllib.request.Request(target, method="GET")
            with urllib.request.urlopen(request, timeout=HTTP_TIMEOUT_SEC) as response:
                self.send_response(response.status)
                content_type = response.headers.get("Content-Type", "application/octet-stream")
                self.send_header("Content-Type", content_type)
                length = response.headers.get("Content-Length")
                if length:
                    self.send_header("Content-Length", length)
                # 无论有没有 Content-Length 都断开连接：
                # 相机有时用 chunked 编码（没有 Content-Length），若保持
                # keep-alive，客户端会一直等响应体直到超时
                #（Qt 会报 "Operation canceled"，实测踩过）。
                self.send_header("Connection", "close")
                self.close_connection = True
                self.end_headers()
                if not send_body:
                    return
                total = 0
                while True:
                    chunk = response.read(HTTP_CHUNK)
                    if not chunk:
                        break
                    self.wfile.write(chunk)
                    total += len(chunk)
                log("media-proxy: %s -> %d 字节" % (self.path[:70], total))
        except urllib.error.HTTPError as error:
            log("media-proxy: %s 相机返回 HTTP %s" % (self.path[:70], error.code))
            self.send_error(error.code, "camera returned %s" % error.code)
        except Exception as error:                      # noqa: BLE001
            log("media-proxy: %s 失败: %s" % (self.path[:70], error))
            try:
                self.send_error(502, "camera unreachable")
            except Exception:                           # noqa: BLE001
                pass

    def do_GET(self):                                   # noqa: N802
        self._forward(True)

    def do_HEAD(self):                                  # noqa: N802
        self._forward(False)

    def log_message(self, fmt, *args):
        # 默认实现会往 stderr 打日志，这里改走统一日志格式
        if "code" not in fmt:
            log("media-proxy: " + (fmt % args))


def start_http_proxy():
    server = http.server.ThreadingHTTPServer(PROXY_ADDR, CameraHttpProxy)
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, name="camera-http-proxy", daemon=True)
    thread.start()
    return server


def main():
    parser = argparse.ArgumentParser(description="SIYI A8 mini 云台/媒体中转")
    parser.add_argument("--no-http", action="store_true",
                        help="不启动 HTTP 媒体代理（只做云台 UDP 转发）")
    parser.add_argument("--http-only", action="store_true",
                        help="只启动 HTTP 媒体代理（照片/录像文件），不做 UDP 转发")
    args = parser.parse_args()

    # ⚠️ 为什么要把 HTTP 代理拆出去单独跑一个进程：
    # 照片/录像下载是几十 MB 的流量，如果和云台 UDP 转发挤在同一个 Python
    # 进程里，HTTP 线程会抢 GIL，把转发主循环拖住 —— 表现为"一点图片列表，
    # 云台操作杆就延迟极大"（2026-10-15 实际复现）。
    # 拆成两个进程后，下载再重也不影响云台指令的实时性。
    if args.http_only:
        start_http_proxy()
        log("HTTP-only 模式：只提供照片/录像文件代理，不转发云台指令")
        log("  监听 %s:%d → %s" % (PROXY_ADDR[0], PROXY_ADDR[1], CAM_HTTP))
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            pass
        return

    downstream = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    downstream.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    downstream.bind(LISTEN_ADDR)
    downstream.setblocking(False)

    # 唯一的、长期存活的上游 socket：源端口恒定，符合相机单客户端模型
    upstream = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    upstream.setblocking(False)

    log("siyi-gimbal-relay v4 启动")
    log("  监听 %s:%d（UDP，云台指令）" % LISTEN_ADDR)
    log("  上游 %s:%d（单一持久 socket，本地端口 %d）"
        % (CAM_ADDR[0], CAM_ADDR[1], upstream.getsockname()[1]))
    log("  安全策略: 丢弃 CMD 0x00（心跳，UDP 不支持）")
    log("  安全策略: CMD 0x07 转向死手保护 %.1f 秒（地面站失联自动停转）"
        % ROTATE_DEADMAN_SEC)
    log("  回复策略: CRC 校验后回传给最近 %.0f 秒的客户端，最多 %d 个"
        % (CLIENT_REPLY_TTL_SEC, MAX_REPLY_CLIENTS))
    if args.no_http:
        log("  媒体代理: 已禁用（--no-http，由独立的 siyi-media-proxy 服务提供）")
    else:
        try:
            start_http_proxy()
            log("  HTTP 代理 %s:%d → %s（照片/视频文件接口，只读）"
                % (PROXY_ADDR[0], PROXY_ADDR[1], CAM_HTTP))
        except OSError as error:
            log("!! HTTP 代理启动失败（端口被占用？）: %s" % error)

    try:
        run_udp_relay(downstream, upstream)
    finally:
        downstream.close()
        upstream.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
