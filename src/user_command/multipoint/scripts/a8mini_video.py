#!/usr/bin/env python3
"""Bounded asynchronous annotated-video recording; never wait for disk in submit."""

import csv
from datetime import datetime
from pathlib import Path
import queue
import threading
import time

from a8mini_diagnostics import log


class AsyncVideoWriter:
    def __init__(self, directory, fps, cv2_module=None):
        if cv2_module is None:
            import cv2 as cv2_module
        self.cv2 = cv2_module
        self.directory = Path(directory).expanduser()
        self.fps = fps
        self.queue = queue.Queue(maxsize=2)
        self.stop = threading.Event()
        self.error = None
        self.dropped = 0
        self.written = 0
        self.thread = threading.Thread(target=self._run, name="detection-video", daemon=True)
        self.thread.start()

    def submit(self, frame, captured_at):
        if self.error is not None or self.stop.is_set():
            return False
        # The producer transfers ownership and must not mutate this frame later.
        item = (frame, captured_at, time.time())
        try:
            self.queue.put_nowait(item)
        except queue.Full:
            try:
                self.queue.get_nowait()
                self.dropped += 1
            except queue.Empty:
                pass
            try:
                self.queue.put_nowait(item)
            except queue.Full:
                self.dropped += 1
                return False
        return True

    def _run(self):
        writer = None
        timestamps = None
        size = None
        opened_at = 0.0
        frame_index = 0
        try:
            while not self.stop.is_set() or not self.queue.empty():
                try:
                    frame, captured_at, wall_time = self.queue.get(timeout=0.1)
                except queue.Empty:
                    continue
                current_size = (frame.shape[1], frame.shape[0])
                # Periodically finalize files; a hard power cut only risks the current segment.
                if writer is None or size != current_size or captured_at - opened_at >= 60:
                    if writer is not None:
                        writer.release()
                    if timestamps is not None:
                        timestamps.close()
                    self.directory.mkdir(parents=True, exist_ok=True)
                    path = self.directory / (datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".avi")
                    writer = self.cv2.VideoWriter(str(path), self.cv2.VideoWriter_fourcc(*"MJPG"),
                                                  self.fps, current_size)
                    if not writer.isOpened():
                        raise OSError("cannot open video writer: " + str(path))
                    timestamps = path.with_suffix(".csv").open("x", newline="")
                    rows = csv.writer(timestamps)
                    rows.writerow(("frame", "capture_monotonic_sec", "result_unix_sec"))
                    size, opened_at, frame_index = current_size, captured_at, 0
                    log("[VIDEO] Saving annotated video: " + str(path))
                writer.write(frame)
                rows.writerow((frame_index, "%.6f" % captured_at, "%.6f" % wall_time))
                timestamps.flush()
                frame_index += 1
                self.written += 1
        except Exception as error:
            self.error = str(error)
            log("[ERROR] Video recording disabled; detection continues: " + self.error)
        finally:
            try:
                if writer is not None:
                    writer.release()
            finally:
                if timestamps is not None:
                    timestamps.close()

    def close(self):
        self.stop.set()
        self.thread.join(timeout=5)
        if self.thread.is_alive():
            log("[ERROR] Video writer did not stop; current segment may be incomplete")
        log("[VIDEO] written=%d dropped=%d error=%s" %
            (self.written, self.dropped, self.error))
