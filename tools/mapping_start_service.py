#!/usr/bin/env python3
"""Loopback-only mapping startup API. Requests cannot select executables or paths."""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import subprocess
import threading

MAX_REQUEST = 1024


class MappingManager:
    def __init__(self, script, popen=subprocess.Popen):
        self.script = Path(script).resolve()
        self.popen = popen
        self.lock = threading.Lock()
        self.state = "idle"
        self.message = "等待地面站建图请求"

    def status(self):
        with self.lock:
            return dict(protocol=1, service="agri-mapping-start", state=self.state, message=self.message)

    def ensure(self):
        with self.lock:
            if self.state != "starting":
                self.state, self.message = "starting", "正在检查机载建图环境"
                threading.Thread(target=self._start, daemon=True).start()
        return self.status()

    def _start(self):
        state, message = "error", "建图启动失败，请检查机载日志"
        try:
            # Fixed local script only; no shell, command or path supplied by HTTP.
            child = self.popen(["/bin/bash", str(self.script)], stdin=subprocess.DEVNULL,
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               text=True, encoding="utf-8", errors="replace", bufsize=1)
            watchdog = threading.Timer(110, child.terminate)
            watchdog.daemon = True
            watchdog.start()
            try:
                for line in child.stdout:
                    line = line.strip()
                    if line.startswith(("STAGE: ", "ERROR: ")):
                        message = line.split(": ", 1)[1][:240]
                        with self.lock:
                            self.message = message
                    print(line, flush=True)
                result = child.wait()
            finally:
                watchdog.cancel()
                child.stdout.close()
            state = "ready" if result == 0 else "error"
            if result == 0:
                message = "建图已就绪，等待地面站接收点云"
        except OSError as exc:
            print("Cannot start mapping helper: {}".format(exc), flush=True)
            message = "无法执行建图启动脚本"
        with self.lock:
            self.state, self.message = state, message


def handler_for(manager):
    class Handler(BaseHTTPRequestHandler):
        def setup(self):
            super().setup()
            self.connection.settimeout(3)

        def reply(self, code, payload):
            raw = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(raw)))
            self.send_header("Connection", "close")
            self.end_headers()
            self.wfile.write(raw)

        def do_GET(self):
            if self.path != "/status":
                self.reply(404, dict(error="unknown endpoint"))
                return
            self.reply(200, manager.status())

        def do_POST(self):
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if self.path != "/ensure" or not 0 < size <= MAX_REQUEST:
                    raise ValueError("invalid request")
                if self.headers.get("Transfer-Encoding"):
                    raise ValueError("chunked requests are unsupported")
                request = json.loads(self.rfile.read(size))
                if (type(request) is not dict or type(request.get("protocol")) is not int
                        or request != dict(protocol=1, operation="ensure_mapping")):
                    raise ValueError("only ensure_mapping is supported")
            except (ValueError, OSError):
                self.reply(400, dict(error="only protocol 1 ensure_mapping is supported"))
                return
            self.reply(200, manager.ensure())

        def log_message(self, *_):
            pass
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--script", type=Path, required=True)
    parser.add_argument("--port", type=int, default=37263)
    args = parser.parse_args()
    if not args.script.is_file() or not 1 <= args.port <= 65535:
        parser.error("existing fixed script and valid port required")
    server = ThreadingHTTPServer(("127.0.0.1", args.port), handler_for(MappingManager(args.script)))
    server.daemon_threads = True
    print("Mapping startup API listening on loopback:{}".format(args.port), flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
