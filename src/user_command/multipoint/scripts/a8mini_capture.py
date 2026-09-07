#!/usr/bin/env python3
"""Flight-owned capture, adapted from A8mini_Detction/rtsp_capture.py (c83acee6).

Keep native isolation and a single shared frame; enforce useful-frame progress
independently of individual reads. No ROS, CUDA or camera control imports.
"""

import multiprocessing as mp
import os
import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np
from a8mini_diagnostics import log


@dataclass
class CaptureConfig:
    source: str
    backend: str = "auto"
    codec: str = "h264"
    latency_ms: int = 150
    open_timeout_ms: int = 2000
    read_timeout_ms: int = 1000
    reconnect_delay: float = 0.2
    max_read_failures: int = 1
    backend_failures: int = 2
    max_frame_age_ms: int = 1000
    max_pixels: int = 3840 * 2160


def build_gstreamer_pipeline(rtsp_url, latency_ms, read_timeout_ms=1000,
                             codec="h264"):
    if codec not in ("h264", "h265"):
        raise ValueError("codec must be h264 or h265")
    location = rtsp_url.replace("\\", "\\\\").replace('"', '\\"')
    return (
        f'rtspsrc location="{location}" protocols=tcp latency={latency_ms} '
        f"tcp-timeout={read_timeout_ms * 1000} drop-on-latency=true ! "
        f"rtp{codec}depay ! {codec}parse ! nvv4l2decoder ! "
        "nvvidconv ! video/x-raw,format=BGRx ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink drop=true max-buffers=1 sync=false"
    )


def open_capture(config, backend):
    """两种后端均传入 open-only 超时；不静默降级到无超时构造。"""
    params = [cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, config.open_timeout_ms,
              cv2.CAP_PROP_READ_TIMEOUT_MSEC, config.read_timeout_ms]
    if backend == "gstreamer":
        source = build_gstreamer_pipeline(
            config.source, config.latency_ms, config.read_timeout_ms, config.codec)
        return cv2.VideoCapture(source, cv2.CAP_GSTREAMER, params)
    # 仅在采集子进程设置，避免影响主进程或其他客户端。
    os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
        f"rtsp_transport;tcp|rw_timeout;{config.read_timeout_ms * 1000}"
    )
    return cv2.VideoCapture(config.source, cv2.CAP_FFMPEG, params)


class SharedFrame:
    """固定容量的单帧槽；被终止进程使用过的锁和缓冲区不会复用。"""

    def __init__(self, context, max_pixels):
        self.data = context.RawArray("B", max_pixels * 3)
        # sequence, height, width, captured_at, cap_ms
        self.meta = context.RawArray("d", 5)
        self.lock = context.Lock()

    def publish(self, frame, captured_at, cap_ms):
        if frame.dtype != np.uint8 or frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError("采集帧必须是 uint8 BGR")
        if frame.size > len(self.data):
            raise ValueError("视频超过 --capture-max-pixels 容量")
        with self.lock:
            np.frombuffer(self.data, dtype=np.uint8, count=frame.size)[:] = frame.reshape(-1)
            self.meta[:] = (self.meta[0] + 1, *frame.shape[:2], captured_at, cap_ms)

    def take(self, previous_sequence, max_age_ms):
        if not self.lock.acquire(timeout=0.01):
            return None
        try:
            seq, height, width, captured_at, cap_ms = self.meta[:]
            if seq == 0 or seq == previous_sequence:
                return None
            if (time.monotonic() - captured_at) * 1000 > max_age_ms:
                return None
            shape = (int(height), int(width), 3)
            # 此复制是跨进程所有权转移所必需，绘图不修改生产者的帧。
            frame = np.frombuffer(self.data, dtype=np.uint8,
                                  count=shape[0] * shape[1] * 3).reshape(shape).copy()
            return int(seq), frame, captured_at, cap_ms
        finally:
            self.lock.release()


def capture_worker(config, backend, slot, connection):
    cap = None
    try:
        connection.send(("phase", "open", time.monotonic()))
        cap = open_capture(config, backend)
        if not cap.isOpened():
            raise RuntimeError("无法打开视频流（检查后端支持、解码插件和网络）")
        failures = 0
        while True:
            started = time.monotonic()
            connection.send(("phase", "read", started))
            ok, frame = cap.read()
            captured_at = time.monotonic()
            cap_ms = (captured_at - started) * 1000
            # OpenCV can return a decoded/buffered frame even after its interrupt
            # callback timed out. Do not let that success renew stream health.
            if cap_ms >= config.read_timeout_ms:
                raise RuntimeError(f"slow read: CAP={cap_ms:.1f} ms >= "
                                   f"read_timeout={config.read_timeout_ms} ms; ok={ok}")
            if not ok or frame is None:
                failures += 1
                if failures >= config.max_read_failures:
                    raise RuntimeError(
                        f"连续读帧失败 {failures} 次，CAP: {(captured_at - started) * 1000:.1f} ms")
                time.sleep(0.02)
                continue
            failures = 0
            if cap_ms >= config.max_frame_age_ms:
                # The independent progress watchdog bounds repeated slow reads.
                continue
            # Conservative local age starts BEFORE read, not after decode.
            # Camera/encoder/network buffering is still not measurable here.
            slot.publish(frame, started, cap_ms)
            connection.send(("frame", captured_at, cap_ms))
    except Exception as error:
        connection.send(("error", f"{type(error).__name__}: {error}"))
    finally:
        if cap is not None:
            cap.release()
        connection.close()


@dataclass
class FramePacket:
    frame: np.ndarray
    sequence: tuple
    captured_at: float
    cap_ms: float
    backend: str


class LatestFrameCapture:
    def __init__(self, config, worker=capture_worker):
        self.config = config
        self._worker = worker
        self._context = mp.get_context("spawn")  # 不 fork 已初始化的 CUDA。
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._session = None
        self._status = "starting"
        self._last_frame_at = None
        self._last_reason = "none"
        self._last_good_at = None  # Persists across reconnects for recovery-gap reporting.
        self._generation = 0
        self._frames_received = 0
        self._last_cap_ms = None
        self._error = None
        self._thread = threading.Thread(target=self._run, name="rtsp-watchdog", daemon=True)

    def start(self):
        self._thread.start()
        return self

    @property
    def status(self):
        with self._lock:
            return self._status_text()

    def _status_text(self):
        # Caller holds _lock.
        if self._status.startswith("streaming") and self._last_frame_at is not None:
            gap = time.monotonic() - self._last_frame_at
            if gap * 1000 >= self.config.max_frame_age_ms:
                return f"stalled ({self._session[1]}); no fresh frame for {gap:.1f}s"
        return self._status

    def snapshot(self):
        with self._lock:
            return {"status": self._status_text(), "generation": self._generation,
                    "reconnects": max(0, self._generation - 1),
                    "frames_received": self._frames_received, "last_cap_ms": self._last_cap_ms,
                    "last_reason": self._last_reason,
                    "since_last_fresh_ms": None if self._last_good_at is None else
                    round((time.monotonic() - self._last_good_at) * 1000, 2)}

    def read_latest(self, previous_sequence=None):
        with self._lock:
            if self._error is not None:
                raise RuntimeError("采集监控异常") from self._error
            if self._session is None:
                return None
            generation, backend, slot = self._session
            previous = previous_sequence[1] if previous_sequence and previous_sequence[0] == generation else 0
            value = slot.take(previous, self.config.max_frame_age_ms)
            if value is None:
                return None
            seq, frame, captured_at, cap_ms = value
            return FramePacket(frame, (generation, seq), captured_at, cap_ms, backend)

    @staticmethod
    def _terminate(process):
        if process.is_alive():
            process.terminate()
        process.join(timeout=0.5)
        if process.is_alive():
            process.kill()
            process.join(timeout=0.5)
        if process.is_alive():
            raise RuntimeError("采集进程无法终止，停止创建新进程")
        exitcode = process.exitcode
        process.close()
        return exitcode

    def _monitor(self, process, connection, backend):
        # spawn/import 有独立启动预算；收到 open 后开始计算打开超时。
        deadline = time.monotonic() + 10.0
        phase = "startup"
        first_frame = None
        progress_deadline = None
        stable = False
        while not self._stop.wait(0.01):
            for _ in range(100):
                if not connection.poll():
                    break
                try:
                    message = connection.recv()
                except EOFError:
                    return "采集进程退出", stable
                if message[0] == "error":
                    return message[1], stable
                if message[0] == "phase":
                    _, phase, started = message
                    timeout = (self.config.open_timeout_ms if phase == "open"
                               else self.config.read_timeout_ms)
                    deadline = started + timeout / 1000 + 0.25
                    if phase == "read" and progress_deadline is None:
                        progress_deadline = deadline
                elif message[0] == "frame":
                    first_frame = first_frame or message[1]
                    stable = message[1] - first_frame >= 30
                    progress_deadline = message[1] + self.config.read_timeout_ms / 1000 + 0.25
                    with self._lock:
                        recovered = self._last_frame_at is None
                        gap = None if self._last_good_at is None else message[1] - self._last_good_at
                        self._last_good_at = message[1]
                        self._last_frame_at = message[1]
                        self._frames_received += 1
                        self._last_cap_ms = round(message[2], 2)
                        self._status = f"streaming ({backend})"
                    if recovered:
                        log(f"[RTSP] first fresh frame backend={backend} generation={self._generation} "
                            f"gap_sec={gap}; "
                            f"previous_reason={self._last_reason}")
            if progress_deadline is not None and time.monotonic() > progress_deadline:
                return "watchdog: no fresh frame within read timeout", stable
            if time.monotonic() > deadline:
                return f"watchdog: {phase} 超时", stable
            if not process.is_alive():
                return f"采集进程退出，exitcode={process.exitcode}", stable
        return "stopped", stable

    def _run(self):
        backend = "gstreamer" if self.config.backend == "auto" else self.config.backend
        failures = 0
        generation = 0
        try:
            while not self._stop.is_set():
                generation += 1
                slot = SharedFrame(self._context, self.config.max_pixels)
                receiver, sender = self._context.Pipe(duplex=False)
                process = self._context.Process(
                    target=self._worker, args=(self.config, backend, slot, sender), daemon=True)
                started = False
                reason = "startup_failed"
                try:
                    process.start()
                    started = True
                    sender.close()
                    with self._lock:
                        self._session = (generation, backend, slot)
                        self._generation = generation
                        self._last_frame_at = None
                        self._status = f"connecting ({backend})"
                    log(f"[RTSP] connecting generation={generation} backend={backend} "
                        f"pid={process.pid}")
                    reason, stable = self._monitor(process, receiver, backend)
                finally:
                    with self._lock:
                        self._session = None
                        self._last_frame_at = None
                        self._status = "reconnecting"
                    sender.close()
                    receiver.close()
                    if started:
                        exitcode = self._terminate(process)
                        log(f"[RTSP] worker_exit generation={generation} exitcode={exitcode} reason={reason}")
                if self._stop.is_set():
                    break
                failures = 1 if stable else failures + 1
                with self._lock:
                    self._last_reason = reason
                log(f"[WARN] RTSP {backend}: generation={generation} {reason}; "
                    f"retry_in={self.config.reconnect_delay:.1f}s")
                if self.config.backend == "auto" and failures >= self.config.backend_failures:
                    backend = "ffmpeg" if backend == "gstreamer" else "gstreamer"
                    failures = 0
                    log(f"[WARN] 自动切换 RTSP 后端 -> {backend}")
                self._stop.wait(self.config.reconnect_delay)
        except Exception as error:
            with self._lock:
                self._error = error
        finally:
            with self._lock:
                self._session = None
                self._status = "stopped"

    def close(self):
        self._stop.set()
        if self._thread.ident is not None:
            self._thread.join(timeout=3)
            if self._thread.is_alive():
                raise RuntimeError("采集监控线程未能按时退出")
