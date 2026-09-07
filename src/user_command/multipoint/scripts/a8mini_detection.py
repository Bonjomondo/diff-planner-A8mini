#!/usr/bin/env python3
"""RTSP-only flight integration for Bonjomondo/A8mini_Detction (c83acee6).

Uses local capture watchdog and upstream class-name/drawing helpers. No camera control
commands, ROS imports, or flight-state subscriptions are used in this process.
"""

import argparse
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import sys
import tempfile
import threading
import time

from a8mini_video import AsyncVideoWriter
from a8mini_diagnostics import log, write_runtime_snapshot


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-path", default="~/Documents/A8mini_Detction")
    parser.add_argument("--model", default="yolo11s.engine")
    parser.add_argument("--source", default="rtsp://192.168.144.25:8554/main.264")
    parser.add_argument("--backend", choices=("auto", "gstreamer", "ffmpeg"), default="auto")
    parser.add_argument("--codec", choices=("h264", "h265"), default="h264")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.55)
    parser.add_argument("--max-fps", type=float, default=30.0)
    parser.add_argument("--cpu-threads", type=int, default=2)
    parser.add_argument("--latency-ms", type=int, default=200)
    parser.add_argument("--open-timeout-ms", type=int, default=5000)
    parser.add_argument("--read-timeout-ms", type=int, default=2500)
    parser.add_argument("--reconnect-delay", type=float, default=1.0)
    parser.add_argument("--max-frame-age-ms", type=int, default=1000)
    parser.add_argument("--video-dir", default="~/Videos/a8mini_detection")
    parser.add_argument("--video-fps", type=float, default=25.0)
    parser.add_argument("--save-video", action="store_true")
    parser.add_argument("--no-display", action="store_true")
    args = parser.parse_args(argv)
    for key in ("max_fps", "video_fps", "cpu_threads", "imgsz", "open_timeout_ms",
                "read_timeout_ms", "max_frame_age_ms"):
        if not math.isfinite(getattr(args, key)) or getattr(args, key) <= 0:
            parser.error(key + " must be positive and finite")
    for key in ("latency_ms", "reconnect_delay"):
        if not math.isfinite(getattr(args, key)) or getattr(args, key) < 0:
            parser.error(key + " must be nonnegative and finite")
    if not 0 <= args.conf <= 1:
        parser.error("conf must be in [0, 1]")
    return args


def acquire_source_lock(source):
    digest = hashlib.sha256(source.encode()).hexdigest()[:24]
    path = Path(tempfile.gettempdir()) / ("a8mini_detection_" + digest + ".lock")
    lock = path.open("a")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock.close()
        raise RuntimeError("another integrated detector is already reading this source")
    return lock  # Keep the inode; unlinking could allow concurrent owners.


def run(args, stop):
    repo = Path(args.repo_path).expanduser().resolve()
    if not (repo / "rtsp_capture.py").is_file():
        raise FileNotFoundError("A8mini_Detction repo_path is invalid: " + str(repo))
    model_path = Path(args.model).expanduser()
    if not model_path.is_absolute():
        model_path = repo / model_path
    if not model_path.is_file():
        raise FileNotFoundError("model does not exist: " + str(model_path))
    # Apply before loading OpenCV/PyTorch; spawn workers inherit these limits.
    for key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
        os.environ[key] = str(args.cpu_threads)
    os.nice(10)
    sys.path.insert(0, str(repo))
    import cv2
    import numpy as np
    import torch
    from ultralytics import YOLO
    from a8mini_capture import CaptureConfig, LatestFrameCapture
    from A8mini_RTSP_YOLO_Detection import ClassNameResolver, draw_detection

    cv2.setNumThreads(args.cpu_threads)
    torch.set_num_threads(args.cpu_threads)
    torch.set_num_interop_threads(1)
    display = not args.no_display and bool(os.environ.get("DISPLAY"))
    write_runtime_snapshot(args, model_path, cv2, torch)
    window = "A8 mini realtime detection"
    capture = None
    video = None
    totals = {"processed": 0, "stale_results": 0, "skipped_capture_frames": 0}
    # Two-second extrema avoid hiding a short inference stall behind the last frame.
    window_max = {"cap_ms": 0, "yolo_ms": 0, "local_age_ms": 0}

    def diagnostics(event):
        log("[DIAG] " + json.dumps({"event": event, "totals": totals,
            "window_max": window_max, "capture": capture.snapshot() if capture is not None else None,
            "video": {"written": video.written, "dropped": video.dropped, "error": video.error}
                     if video is not None else None}, sort_keys=True))
        for key in window_max:
            window_max[key] = 0
    try:
        model = YOLO(str(model_path), task="detect")
        names = ClassNameResolver(model_path)
        log("[DETECTION] Warming up model before opening the live stream", flush=True)
        model.predict(source=np.zeros((args.imgsz, args.imgsz, 3), dtype=np.uint8),
                      imgsz=args.imgsz, conf=args.conf, device=0, rect=False, verbose=False)
        if stop.is_set():
            return
        capture = LatestFrameCapture(CaptureConfig(
            source=args.source, backend=args.backend, codec=args.codec,
            latency_ms=args.latency_ms, open_timeout_ms=args.open_timeout_ms,
            read_timeout_ms=args.read_timeout_ms, reconnect_delay=args.reconnect_delay,
            max_frame_age_ms=args.max_frame_age_ms)).start()
        if args.save_video:
            video = AsyncVideoWriter(args.video_dir, args.video_fps, cv2)
        log("[DETECTION] source=%s model=%s max_fps=%.1f threads=%d display=%s save_video=%s" %
              (args.source, model_path, args.max_fps, args.cpu_threads, display, args.save_video), flush=True)
        log("[DETECTION] capture=local/a8mini_capture.py opencv=%s open_ms=%d read_ms=%d "
            "max_local_age_ms=%d reconnect_sec=%.1f; AGE includes read; excludes upstream buffering" %
            (cv2.__version__, args.open_timeout_ms, args.read_timeout_ms,
             args.max_frame_age_ms, args.reconnect_delay), flush=True)
        sequence = None
        next_inference = 0.0
        last_stats = time.monotonic()
        last_result = 0.0
        last_waiting = 0.0
        count = 0
        while not stop.is_set():
            if display and cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
            now = time.monotonic()
            if now < next_inference:
                stop.wait(min(0.01, next_inference - now))
                continue
            packet = capture.read_latest(sequence)
            if packet is None:
                if display and now - last_result > args.max_frame_age_ms / 1000 and now - last_waiting > 0.2:
                    waiting = np.zeros((180, 800, 3), dtype=np.uint8)
                    cv2.putText(waiting, "Waiting: " + capture.status, (15, 90),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
                    cv2.imshow(window, waiting)
                    last_waiting = now
                if now - last_stats >= 2:
                    log("[STATS] RTSP: " + capture.status + "; waiting for fresh frame", flush=True)
                    diagnostics("waiting")
                    last_stats, count = now, 0
                stop.wait(0.01)
                continue
            if sequence is not None and sequence[0] == packet.sequence[0]:
                totals["skipped_capture_frames"] += max(0, packet.sequence[1] - sequence[1] - 1)
            else:
                totals["skipped_capture_frames"] += max(0, packet.sequence[1] - 1)
            sequence = packet.sequence
            started = time.monotonic()
            next_inference = started + 1.0 / args.max_fps
            frame = packet.frame
            result = model.predict(source=frame, imgsz=args.imgsz, conf=args.conf,
                                   device=0, rect=False, verbose=False)[0]
            boxes = result.boxes.data.detach().cpu().numpy() if result.boxes is not None else []
            yolo_ms = (time.monotonic() - started) * 1000
            local_age = (time.monotonic() - packet.captured_at) * 1000
            for key, value in (("cap_ms", packet.cap_ms), ("yolo_ms", yolo_ms), ("local_age_ms", local_age)):
                window_max[key] = round(max(window_max[key], value), 2)
            if local_age > args.max_frame_age_ms:
                totals["stale_results"] += 1
                # Rate-limit persistent stale-result warnings as well as good-frame stats.
                if time.monotonic() - last_stats >= 2:
                    diagnostics("stale_inference")
                    last_stats, count = time.monotonic(), 0
                continue
            class_names = names.resolve(result.names, [row[-1] for row in boxes])
            for row in boxes:
                draw_detection(frame, row[:4], int(row[-1]), float(row[-2]), class_names)
            now = time.monotonic()
            # Includes read + inference, excludes camera/encoder/network internal buffering.
            age_ms = (now - packet.captured_at) * 1000
            metrics = "YOLO %.1f ms AGE %.1f ms objects %d" % (yolo_ms, age_ms, len(boxes))
            cv2.putText(frame, metrics, (20, 35), cv2.FONT_HERSHEY_SIMPLEX,
                        0.65, (0, 255, 255), 2)
            if display:
                cv2.imshow(window, frame)
            if video is not None:
                video.submit(frame, packet.captured_at)
            last_result = now
            totals["processed"] += 1
            count += 1
            if now - last_stats >= 2:
                log("[STATS] %s FPS %.1f CAP %.1f ms %s VIDEO_DROP %d" %
                      (packet.backend, count / (now - last_stats), packet.cap_ms, metrics,
                       video.dropped if video else 0), flush=True)
                diagnostics("streaming")
                last_stats, count = now, 0
    finally:
        try:
            if capture is not None:
                capture.close()
        finally:
            if video is not None:
                video.close()
            diagnostics("detector_exit")
            if display:
                cv2.destroyAllWindows()


def main(argv=None):
    args = parse_args(argv)
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    try:
        with acquire_source_lock(args.source):
            run(args, stop)
    except KeyboardInterrupt:
        return 0
    except Exception as error:
        log("[ERROR] Detection failed: %s: %s" % (type(error).__name__, error), flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
