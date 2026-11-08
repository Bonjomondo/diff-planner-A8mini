#!/usr/bin/env python3
"""Decode an RTSP stream into bounded, keyframe-gated RGB frames."""

import json
import os
import re
import selectors
import subprocess
import threading
import time
from urllib.parse import quote, urlsplit, urlunsplit


_CORRUPTION = re.compile(
    r"could not find ref|error while decoding|invalid data|corrupt|conceal|"
    r"missing picture|invalid nal|non-existing pps|pps id out of range|"
    r"missed \d+ packets|packet loss|decoding error|error decoding",
    re.IGNORECASE,
)
_POC_ZERO_NOTE = re.compile(r"could not find ref with POC 0\b", re.IGNORECASE)


def redact(text):
    return re.sub(r"(rtsp://)[^/@\s]+@", r"\1<redacted>@", text)


def source_with_credentials(source, credentials_file):
    """Read relay credentials locally instead of placing them on the ROS parameter server."""
    parsed = urlsplit(source)
    if parsed.scheme.lower() != "rtsp" or not parsed.hostname:
        raise ValueError("a credentials file requires an RTSP source")
    if parsed.username is not None or parsed.password is not None:
        raise ValueError("RTSP source already contains credentials")
    with open(os.path.expanduser(credentials_file), encoding="utf-8") as handle:
        credentials = json.load(handle)
    username, password = credentials.get("username"), credentials.get("password")
    if not isinstance(username, str) or not username or not isinstance(password, str) or not password:
        raise ValueError("credentials file must contain a nonempty username and password")
    authority = quote(username, safe="") + ":" + quote(password, safe="") + "@" + parsed.netloc
    return urlunsplit((parsed.scheme, authority, parsed.path, parsed.query, parsed.fragment))


class PpmDecoder:
    """Read FFmpeg's P6 image pipe without assuming a fixed resolution."""

    def __init__(self, max_pixels):
        self.max_pixels = max_pixels
        self.buffer = bytearray()
        self.expected = None
        self.width = None
        self.height = None

    def feed(self, chunk):
        self.buffer.extend(chunk)
        frames = []
        while True:
            if self.expected is None:
                ends = []
                start = 0
                for _ in range(3):
                    end = self.buffer.find(b"\n", start)
                    if end < 0:
                        if len(self.buffer) > 128:
                            raise ValueError("invalid PPM header")
                        return frames
                    ends.append(end)
                    start = end + 1
                if ends[2] >= 128:
                    raise ValueError("PPM header is too large")
                if self.buffer[:ends[0]] != b"P6":
                    raise ValueError("invalid PPM magic")
                size = bytes(self.buffer[ends[0] + 1:ends[1]]).split()
                if len(size) != 2 or self.buffer[ends[1] + 1:ends[2]] != b"255":
                    raise ValueError("unsupported PPM format")
                try:
                    width, height = (int(value) for value in size)
                except ValueError as exc:
                    raise ValueError("invalid PPM dimensions") from exc
                if width <= 0 or height <= 0 or width * height > self.max_pixels:
                    raise ValueError("PPM dimensions exceed configured maximum")
                self.width, self.height = width, height
                self.expected = width * height * 3
                del self.buffer[:ends[2] + 1]
            if len(self.buffer) < self.expected:
                return frames
            frames.append((self.width, self.height, bytes(self.buffer[:self.expected])))
            del self.buffer[:self.expected]
            self.expected = None


def ffmpeg_command(source, file_loop=False, threads=2, rtsp_timeout_sec=5):
    if not source:
        raise ValueError("source is empty")
    command = [
        "ffmpeg", "-nostdin", "-hide_banner", "-loglevel", "warning",
        "-fflags", "+discardcorrupt", "-err_detect", "explode",
        "-threads", str(threads),
    ]
    if source.lower().startswith("rtsp://"):
        command += ["-rtsp_transport", "tcp", "-stimeout", str(int(rtsp_timeout_sec * 1000000))]
    elif file_loop:
        command += ["-stream_loop", "-1", "-re"]
    command += [
        "-i", source, "-map", "0:v:0", "-an", "-sn", "-dn",
        # Before the first decoded keyframe, never pass predictive frames.
        "-vf", r"select=if(eq(key\,1)\,st(0\,1)\,ld(0))",
        "-vsync", "0", "-pix_fmt", "rgb24", "-vcodec", "ppm",
        "-f", "image2pipe", "pipe:1",
    ]
    return command


class Capture:
    """Supervise FFmpeg and retain only the most recent accepted frame."""

    def __init__(self, source, log, file_loop=False, max_pixels=4096 * 2160,
                 startup_timeout=30.0, stall_timeout=3.0, quiet_time=1.0,
                 threads=2, rtsp_timeout_sec=5.0):
        self.source = source
        self.log = log
        self.file_loop = file_loop
        self.max_pixels = max_pixels
        self.startup_timeout = startup_timeout
        self.stall_timeout = stall_timeout
        self.quiet_time = quiet_time
        self.threads = threads
        self.rtsp_timeout_sec = rtsp_timeout_sec
        self.stop_event = threading.Event()
        self.condition = threading.Condition()
        self.thread = threading.Thread(target=self._run, name="siyi-capture", daemon=True)
        self.latest = None
        self.sequence = 0
        self.state = "starting"
        self.restarts = 0
        self.corruptions = 0
        self.reference_notes = 0
        self.accepted = 0

    def start(self):
        self.thread.start()
        return self

    def close(self):
        self.stop_event.set()
        with self.condition:
            self.condition.notify_all()
        if self.thread.ident is not None:
            self.thread.join(timeout=4)
            if self.thread.is_alive():
                raise RuntimeError("capture thread did not stop")

    def snapshot(self):
        with self.condition:
            return dict(state=self.state, restarts=self.restarts,
                        corruptions=self.corruptions, reference_notes=self.reference_notes,
                        accepted=self.accepted)

    def read_latest(self, previous, timeout=0.2, max_age=1.0):
        deadline = time.monotonic() + timeout
        with self.condition:
            while not self.stop_event.is_set():
                if self.latest is not None and self.latest[0] != previous:
                    seq, width, height, data, received = self.latest
                    if time.monotonic() - received <= max_age:
                        return seq, width, height, data
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self.condition.wait(remaining)
        return None

    def _state(self, state):
        with self.condition:
            self.state = state
            if state != "streaming":
                self.latest = None
            self.condition.notify_all()

    def _emit(self, width, height, data, received):
        with self.condition:
            self.sequence += 1
            self.accepted += 1
            self.latest = (self.sequence, width, height, data, received)
            self.state = "streaming"
            self.condition.notify_all()

    def _run(self):
        delay = 1.0
        while not self.stop_event.is_set():
            self._state("waiting_for_keyframe")
            try:
                reason, ever_streamed = self._session()
            except Exception as exc:
                reason, ever_streamed = redact("%s: %s" % (type(exc).__name__, exc)), False
            if self.stop_event.is_set():
                break
            self._state("reconnecting")
            with self.condition:
                self.restarts += 1
            self.log("warning", "video session ended: %s; retry in %.1fs" % (reason, delay))
            self.stop_event.wait(delay)
            delay = 1.0 if ever_streamed else min(delay * 2.0, 10.0)
        self._state("stopped")

    def _session(self):
        command = ffmpeg_command(self.source, self.file_loop, self.threads, self.rtsp_timeout_sec)
        proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                stdin=subprocess.DEVNULL, bufsize=0)
        selector = selectors.DefaultSelector()
        os.set_blocking(proc.stdout.fileno(), False)
        os.set_blocking(proc.stderr.fileno(), False)
        selector.register(proc.stdout, selectors.EVENT_READ, "stdout")
        selector.register(proc.stderr, selectors.EVENT_READ, "stderr")
        decoder = PpmDecoder(self.max_pixels)
        stderr_pending = b""
        started = time.monotonic()
        last_frame = None
        last_fault = started
        last_fault_log = 0.0
        ever_streamed = False
        try:
            while not self.stop_event.is_set():
                now = time.monotonic()
                if proc.poll() is not None:
                    return "decoder exited (%s)" % proc.returncode, ever_streamed
                if not ever_streamed and now - started > self.startup_timeout:
                    return "no accepted keyframe before startup timeout", ever_streamed
                if ever_streamed and last_frame is not None and now - last_frame > self.stall_timeout:
                    return "no new decoded frame before timeout", ever_streamed
                events = selector.select(0.2)
                # Process decoder errors first when both pipes are ready.
                events.sort(key=lambda event: event[0].data != "stderr")
                for key, _ in events:
                    try:
                        data = os.read(key.fileobj.fileno(), 256 * 1024)
                    except BlockingIOError:
                        continue
                    if not data:
                        try:
                            selector.unregister(key.fileobj)
                        except KeyError:
                            pass
                        continue
                    if key.data == "stderr":
                        stderr_pending += data
                        if len(stderr_pending) > 32768:
                            stderr_pending = stderr_pending[-32768:]
                        while b"\n" in stderr_pending:
                            line, stderr_pending = stderr_pending.split(b"\n", 1)
                            line = redact(line.decode("utf-8", "replace").strip())
                            if not line:
                                continue
                            # The working A8 mini player also logs this specific
                            # note while decoding normal frames. It is not, on
                            # its own, evidence that the current image is bad.
                            # Initial keyframe gating remains in FFmpeg.
                            if _POC_ZERO_NOTE.search(line):
                                with self.condition:
                                    self.reference_notes += 1
                            elif _CORRUPTION.search(line):
                                last_fault = time.monotonic()
                                with self.condition:
                                    self.corruptions += 1
                                    self.latest = None
                                    self.condition.notify_all()
                                if last_fault - last_fault_log > 5:
                                    self.log("warning", "decoder rejected suspect video: " + line[:250])
                                    last_fault_log = last_fault
                                if ever_streamed:
                                    return "decoder reported corrupt video", ever_streamed
                            elif time.monotonic() - last_fault_log > 5:
                                self.log("warning", "decoder: " + line[:250])
                                last_fault_log = time.monotonic()
                    else:
                        for width, height, pixels in decoder.feed(data):
                            last_frame = time.monotonic()
                            if last_frame - last_fault >= self.quiet_time:
                                self._emit(width, height, pixels, last_frame)
                                ever_streamed = True
            return "stopped", ever_streamed
        finally:
            selector.close()
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait(timeout=1)
            proc.stdout.close()
            proc.stderr.close()
