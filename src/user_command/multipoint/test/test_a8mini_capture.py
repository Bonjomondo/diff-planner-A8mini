"""无需相机/GPU：真实 spawn 进程中的断流、阻塞与故障切换测试。"""

import contextlib
import io
import json
import multiprocessing as mp
import sys
import tempfile
import time
import types
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import a8mini_capture as rtsp


class FakeCapture:
    def __init__(self, source, backend):
        self.source, self.backend, self.count = source, backend, 0

    def isOpened(self):
        return True

    def read(self):
        if self.source == "slow-success" and self.backend == "gstreamer":
            time.sleep(0.12)  # Past read timeout, before the old watchdog's grace.
            return True, np.zeros((2, 2, 3), dtype=np.uint8)
        if self.source == "stale-success" and self.backend == "gstreamer":
            time.sleep(0.06)  # Repeated read phases must not renew frame health.
            return True, np.zeros((2, 2, 3), dtype=np.uint8)
        if self.source == "hang-read" and self.backend == "gstreamer":
            time.sleep(60)
        if self.source == "hang-release" and self.backend == "gstreamer":
            return False, None
        if self.source == "flaky" and self.backend == "gstreamer" and self.count:
            return False, None
        time.sleep(0.01)
        self.count += 1
        return True, np.full((2, 2, 3), self.count % 256, dtype=np.uint8)

    def release(self):
        if self.source == "hang-release" and self.backend == "gstreamer":
            time.sleep(60)


def fake_open(config, backend):
    if config.source == "hang-open" and backend == "gstreamer":
        time.sleep(60)
    if config.source == "error-open" and backend == "gstreamer":
        raise RuntimeError("injected open error")
    return FakeCapture(config.source, backend)


def fake_worker(config, backend, slot, connection):
    rtsp.open_capture = fake_open
    rtsp.capture_worker(config, backend, slot, connection)


def native_test_worker(config, backend, slot, connection):
    rtsp.open_capture = lambda cfg, _: rtsp.NativeGStreamerCapture(
        cfg, pipeline="videotestsrc is-live=true pattern=white ! "
                      "video/x-raw,format=BGR,width=3,height=2,framerate=25/1 ! "
                      "appsink name=frames drop=true max-buffers=1 sync=false wait-on-eos=false")
    rtsp.capture_worker(config, backend, slot, connection)


class NativeGStreamerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            import gi
            gi.require_version("Gst", "1.0")
            gi.require_version("GstVideo", "1.0")
            from gi.repository import Gst
            Gst.init(None)
            if not Gst.ElementFactory.find("videotestsrc") or not Gst.ElementFactory.find("appsink"):
                raise RuntimeError("test source/appsink missing")
        except (ImportError, ValueError, RuntimeError) as error:
            raise unittest.SkipTest("optional system GStreamer unavailable: " + str(error))

    def test_real_gi_video_padding_and_eos(self):
        cap = rtsp.NativeGStreamerCapture(
            rtsp.CaptureConfig("test", open_timeout_ms=1000, read_timeout_ms=100),
            pipeline="videotestsrc num-buffers=1 pattern=white ! "
                     "video/x-raw,format=BGR,width=3,height=2 ! "
                     "appsink name=frames sync=false wait-on-eos=false")
        try:
            ok, frame = cap.read()
            self.assertTrue(ok)
            self.assertEqual(frame.shape, (2, 3, 3))
            np.testing.assert_array_equal(frame, np.full((2, 3, 3), 255, np.uint8))
            self.assertEqual(cap.read(), (False, None))
        finally:
            cap.release()
        self.assertFalse(cap.isOpened())
        self.assertIsNone(cap.pipeline)
        cap.release()

    def test_real_gi_pipeline_error_is_reported(self):
        with tempfile.TemporaryDirectory() as directory, self.assertRaisesRegex(RuntimeError, "GStreamer"):
            rtsp.NativeGStreamerCapture(
                rtsp.CaptureConfig("test", open_timeout_ms=100, read_timeout_ms=100),
                pipeline='filesrc location="%s/missing.raw" ! '
                         'appsink name=frames sync=false' % directory)

    def test_real_gi_spawn_shared_slot_and_cleanup(self):
        before = {p.pid for p in mp.active_children()}
        capture = rtsp.LatestFrameCapture(rtsp.CaptureConfig(
            "test", backend="gstreamer", open_timeout_ms=2000,
            read_timeout_ms=1000, max_pixels=6), worker=native_test_worker).start()
        try:
            deadline = time.monotonic() + 5
            packet = None
            while packet is None and time.monotonic() < deadline:
                packet = capture.read_latest()
                time.sleep(0.01)
            self.assertIsNotNone(packet, capture.status)
            np.testing.assert_array_equal(packet.frame, np.full((2, 3, 3), 255, np.uint8))
            self.assertEqual(packet.backend, "gstreamer")
        finally:
            capture.close()
        self.assertEqual({p.pid for p in mp.active_children()}, before)


class CaptureTests(unittest.TestCase):
    def config(self, source, **kwargs):
        return rtsp.CaptureConfig(source, open_timeout_ms=80, read_timeout_ms=80,
                                  reconnect_delay=0.01, max_pixels=4, **kwargs)

    def wait_packet(self, capture, backend=None, previous=None, timeout=8):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            packet = capture.read_latest(previous)
            if packet is not None and (backend is None or packet.backend == backend):
                return packet
            time.sleep(0.01)
        self.fail(f"没有收到 {backend} 的新帧: {capture.status}")

    def test_auto_recovers_from_native_hangs_and_errors(self):
        for source in ("hang-open", "hang-read", "hang-release", "error-open", "flaky"):
            with self.subTest(source=source):
                before = {p.pid for p in mp.active_children()}
                capture = rtsp.LatestFrameCapture(self.config(source), worker=fake_worker).start()
                try:
                    packet = self.wait_packet(capture, "ffmpeg")
                    self.assertEqual(packet.sequence[0], 3)
                    self.assertEqual(packet.frame.shape, (2, 2, 3))
                finally:
                    capture.close()
                self.assertEqual({p.pid for p in mp.active_children()}, before)

    def test_successful_but_timed_out_reads_reconnect_without_publishing(self):
        capture = rtsp.LatestFrameCapture(
            self.config("slow-success"), worker=fake_worker).start()
        try:
            packet = self.wait_packet(capture)
            self.assertEqual(packet.backend, "ffmpeg")
            self.assertEqual(packet.sequence[0], 3)
        finally:
            capture.close()

    def test_repeated_stale_success_cannot_extend_progress_deadline(self):
        capture = rtsp.LatestFrameCapture(
            self.config("stale-success", max_frame_age_ms=30), worker=fake_worker).start()
        try:
            packet = self.wait_packet(capture)
            self.assertEqual(packet.backend, "ffmpeg")
            self.assertEqual(packet.sequence[0], 3)
            self.assertIn("no fresh frame", capture._last_reason)
        finally:
            capture.close()

    def test_timeout_success_is_rejected_and_read_time_is_in_local_age(self):
        frame = np.zeros((2, 2, 3), dtype=np.uint8)
        for elapsed in (0.04, 2.5157):
            with self.subTest(elapsed=elapsed):
                cap, slot, conn = Mock(), Mock(), Mock()
                cap.read.side_effect = [(True, frame), RuntimeError("end test")]
                # open phase, first read start/end, next read start (normal only).
                with patch.object(rtsp, 'open_capture', return_value=cap), \
                        patch.object(rtsp.time, 'monotonic', side_effect=[9.0, 10.0, 10.0 + elapsed, 13.0]):
                    rtsp.capture_worker(rtsp.CaptureConfig('fake', read_timeout_ms=2500),
                                        'ffmpeg', slot, conn)
                cap.release.assert_called_once()
                if elapsed > 2.5:
                    slot.publish.assert_not_called()
                    self.assertIn('slow read', conn.send.call_args.args[0][1])
                else:
                    self.assertEqual(slot.publish.call_args.args[1], 10.0)
                    self.assertAlmostEqual(slot.publish.call_args.args[2], 40.0)

    def test_streaming_status_expires_when_frames_stop(self):
        capture = rtsp.LatestFrameCapture(self.config('fake', max_frame_age_ms=100))
        with capture._lock:
            capture._session = (1, 'ffmpeg', None)
            capture._status = 'streaming (ffmpeg)'
            capture._last_frame_at = time.monotonic()
        self.assertEqual(capture.status, 'streaming (ffmpeg)')
        with capture._lock:
            capture._last_frame_at -= 1
        self.assertIn('stalled', capture.status)

    def test_explicit_backend_reconnects_without_switching(self):
        capture = rtsp.LatestFrameCapture(
            self.config("flaky", backend="gstreamer"), worker=fake_worker).start()
        try:
            first = self.wait_packet(capture)
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline:
                packet = self.wait_packet(capture, previous=first.sequence)
                self.assertEqual(packet.backend, "gstreamer")
                if packet.sequence[0] > first.sequence[0]:
                    break
            else:
                self.fail("未重新连接")
        finally:
            capture.close()

    def test_slow_consumer_skips_frames_and_owns_snapshot(self):
        capture = rtsp.LatestFrameCapture(
            self.config("normal", backend="ffmpeg"), worker=fake_worker).start()
        try:
            first = self.wait_packet(capture)
            original = first.frame.copy()
            time.sleep(0.15)  # 模拟耗时推理；采集继续推进。
            second = self.wait_packet(capture, previous=first.sequence)
            self.assertGreater(second.sequence[1] - first.sequence[1], 3)
            np.testing.assert_array_equal(first.frame, original)
        finally:
            capture.close()

    def test_close_during_blocked_read_is_bounded(self):
        capture = rtsp.LatestFrameCapture(
            self.config("hang-read", backend="gstreamer"), worker=fake_worker).start()
        time.sleep(0.5)
        started = time.monotonic()
        capture.close()
        self.assertLess(time.monotonic() - started, 2)
        self.assertFalse(capture._thread.is_alive())

    def test_shared_slot_rejects_duplicates_and_stale_frames(self):
        slot = rtsp.SharedFrame(mp.get_context("spawn"), 4)
        frame = np.zeros((2, 2, 3), dtype=np.uint8)
        slot.publish(frame, time.monotonic(), 12)
        snapshot = slot.take(0, 1000)
        self.assertEqual(snapshot[3], 12)
        self.assertIsNone(slot.take(snapshot[0], 1000))
        snapshot[1][:] = 255
        np.testing.assert_array_equal(slot.take(0, 1000)[1], frame)
        slot.publish(frame, time.monotonic() - 2, 12)
        self.assertIsNone(slot.take(0, 1000))

    def test_killed_worker_lock_cannot_block_consumer(self):
        slot = rtsp.SharedFrame(mp.get_context("spawn"), 4)
        slot.lock.acquire()
        try:
            started = time.monotonic()
            self.assertIsNone(slot.take(0, 1000))
            self.assertLess(time.monotonic() - started, 0.1)
        finally:
            slot.lock.release()

    def test_capacity_checked(self):
        slot = rtsp.SharedFrame(mp.get_context("spawn"), 4)
        with self.assertRaisesRegex(ValueError, "容量"):
            slot.publish(np.zeros((3, 3, 3), dtype=np.uint8), time.monotonic(), 1)

    def test_both_backends_receive_unclamped_open_timeouts(self):
        config = rtsp.CaptureConfig("rtsp://camera/main.264", read_timeout_ms=800)
        for backend in ("gstreamer", "ffmpeg"):
            with patch.object(rtsp.cv2, "VideoCapture") as constructor, \
                    patch.object(rtsp.cv2, "getBuildInformation", return_value="GStreamer: YES"), \
                    patch.dict(rtsp.os.environ):
                rtsp.open_capture(config, backend)
                params = constructor.call_args.args[2]
                self.assertEqual(params, [rtsp.cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 2000,
                                          rtsp.cv2.CAP_PROP_READ_TIMEOUT_MSEC, 800])

    def test_native_gstreamer_used_when_opencv_has_no_gstreamer(self):
        config = rtsp.CaptureConfig("rtsp://camera/main.264", codec="h265")
        with patch.object(rtsp.cv2, "getBuildInformation", return_value="GStreamer: NO"), \
                patch.object(rtsp.cv2, "VideoCapture") as cv_capture, \
                patch.object(rtsp, "NativeGStreamerCapture") as native:
            self.assertIs(rtsp.open_capture(config, "gstreamer"), native.return_value)
            native.assert_called_once_with(config)
            cv_capture.assert_not_called()

    def test_gst_buffer_padding_is_not_pixels_and_frame_owns_its_memory(self):
        data = bytearray([99, 98, 1, 2, 3, 4, 5, 6, 90, 91, 7, 8, 9, 10, 11, 12, 92, 93])
        frame = rtsp.copy_gst_frame(data, height=2, width=2, stride=8, offset=2)
        np.testing.assert_array_equal(frame, [[[1, 2, 3], [4, 5, 6]],
                                              [[7, 8, 9], [10, 11, 12]]])
        data[:] = bytes(len(data))
        self.assertEqual(int(frame[0, 0, 0]), 1)
        for height, width, stride, offset in ((2, 2, 5, 0), (3, 2, 8, 0),
                                              (2, 2, 8, -1), (0, 2, 8, 0)):
            with self.subTest(layout=(height, width, stride, offset)), self.assertRaises(ValueError):
                rtsp.copy_gst_frame(data, height, width, stride, offset)

    def test_codecs(self):
        for codec in ("h264", "h265"):
            pipeline = rtsp.build_gstreamer_pipeline('rtsp://camera/"stream', 100, 800, codec)
            self.assertIn(f"rtp{codec}depay ! {codec}parse", pipeline)
            self.assertIn("tcp-timeout=800000", pipeline)
            self.assertIn('\\"stream', pipeline)
            self.assertIn("max-buffers=1", pipeline)


if __name__ == '__main__':
    unittest.main()
