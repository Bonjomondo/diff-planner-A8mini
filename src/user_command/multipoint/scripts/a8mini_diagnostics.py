#!/usr/bin/env python3
"""Small, optional detector diagnostics; never import ROS or initialize GPU libraries."""

from datetime import datetime
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import sys
import time


def log(message, **kwargs):
    kwargs.setdefault("flush", True)
    print("%s mono=%.6f pid=%d %s" % (
        datetime.now().astimezone().isoformat(timespec="milliseconds"),
        time.monotonic(), os.getpid(), message), **kwargs)


def write_runtime_snapshot(args, model_path, cv2, torch):
    directory = os.environ.get("UAV_DEBUG_LOG_DIR")
    if not directory:
        return
    try:
        root = Path(directory) / ("detector_runtime_%d" % os.getpid())
        root.mkdir(parents=True, exist_ok=True)
        sources = {}
        scripts = Path(__file__).resolve().parent
        repo = Path(args.repo_path).expanduser().resolve()
        paths = [scripts / name for name in ("a8mini_detection.py", "a8mini_capture.py",
                                             "a8mini_video.py", "a8mini_diagnostics.py")]
        paths += [repo / name for name in ("rtsp_capture.py", "A8mini_RTSP_YOLO_Detection.py")]
        for path in paths:
            try:
                if path.stat().st_size > 1024 * 1024:
                    sources[str(path)] = {"skipped": "source larger than 1 MiB"}
                    continue
                data = path.read_bytes()
                (root / (path.name + ".snapshot")).write_bytes(data)
                sources[str(path)] = {"sha256": hashlib.sha256(data).hexdigest(), "bytes": len(data)}
            except OSError as error:
                sources[str(path)] = {"unavailable": str(error)}
        packages = {}
        for package in ("ultralytics", "torch", "numpy", "tensorrt", "opencv-python", "opencv-contrib-python"):
            try:
                packages[package] = importlib.metadata.version(package)
            except importlib.metadata.PackageNotFoundError:
                packages[package] = "no distribution metadata"
        stat = model_path.stat()
        manifest = {"schema_version": 1, "wall_time": datetime.now().astimezone().isoformat(),
                    "unix_sec": time.time(), "monotonic_sec": time.monotonic(),
                    "pid": os.getpid(), "python_executable": sys.executable, "python_version": sys.version,
                    "parameters": vars(args), "packages": packages, "opencv_version": cv2.__version__,
                    "torch_version": str(torch.__version__), "torch_cuda_version": str(torch.version.cuda),
                    "model": {"path": str(model_path), "size": stat.st_size, "mtime_ns": stat.st_mtime_ns},
                    "sources": sources,
                    "thread_environment": {key: os.environ.get(key) for key in (
                        "OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS")}}
        (root / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
        (root / "opencv_build.txt").write_text(cv2.getBuildInformation())
        log("[DIAGNOSTICS] detector runtime saved: " + str(root))
    except Exception as error:
        log("[WARN] Runtime snapshot unavailable; detection continues: " + str(error))
