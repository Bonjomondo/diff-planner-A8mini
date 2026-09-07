#!/usr/bin/env python3

import importlib.util
import io
from pathlib import Path
import sys
import tempfile
import threading
import time
import types
import unittest
from unittest.mock import Mock, patch

import numpy as np

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
import a8mini_detection as detection
import a8mini_diagnostics as diagnostics
from a8mini_video import AsyncVideoWriter


class VideoTest(unittest.TestCase):
    def test_slow_writer_drops_frames_without_blocking_producer(self):
        entered, unblock = threading.Event(), threading.Event()
        received = []

        def write(frame):
            entered.set()
            unblock.wait(2)
            received.append(int(frame[0, 0, 0]))

        cv2 = Mock()
        cv2.VideoWriter.return_value.write.side_effect = write
        with tempfile.TemporaryDirectory() as directory:
            video = AsyncVideoWriter(directory, 15, cv2)
            try:
                video.submit(np.zeros((16, 16, 3), np.uint8), time.monotonic())
                self.assertTrue(entered.wait(1))
                started = time.monotonic()
                for index in range(1, 21):
                    video.submit(np.full((16, 16, 3), index, np.uint8), time.monotonic())
                self.assertLess(time.monotonic() - started, 0.2)
                self.assertEqual(video.queue.qsize(), 2)
                self.assertGreater(video.dropped, 0)
            finally:
                unblock.set()
                video.close()
            self.assertEqual(received, [0, 19, 20])
            self.assertTrue(cv2.VideoWriter.return_value.release.called)
            self.assertEqual(len(next(Path(directory).glob('*.csv')).read_text().splitlines()), 4)

    def test_failed_video_open_disables_save_and_releases_writer(self):
        cv2 = Mock()
        cv2.VideoWriter.return_value.isOpened.return_value = False
        with tempfile.TemporaryDirectory() as directory:
            video = AsyncVideoWriter(directory, 15, cv2)
            video.submit(np.zeros((16, 16, 3), np.uint8), time.monotonic())
            video.close()
            self.assertIn('cannot open', video.error)
            self.assertFalse(video.submit(np.zeros((16, 16, 3), np.uint8), time.monotonic()))
            cv2.VideoWriter.return_value.release.assert_called_once()

    def test_resolution_change_starts_new_segment(self):
        cv2 = Mock()
        with tempfile.TemporaryDirectory() as directory:
            video = AsyncVideoWriter(directory, 15, cv2)
            video.submit(np.zeros((16, 16, 3), np.uint8), time.monotonic())
            video.submit(np.zeros((32, 32, 3), np.uint8), time.monotonic())
            video.close()
            self.assertEqual(cv2.VideoWriter.call_count, 2)
            self.assertEqual(len(list(Path(directory).glob('*.csv'))), 2)


class DetectionTest(unittest.TestCase):
    def test_runtime_snapshot_uses_detector_environment_and_external_source_hashes(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'yolo11s.engine').write_bytes(b'fake engine')
            (root / 'rtsp_capture.py').write_text('# external capture\n')
            (root / 'A8mini_RTSP_YOLO_Detection.py').write_text('# external helpers\n')
            cv2 = types.SimpleNamespace(__version__='actual-cv', getBuildInformation=lambda: 'GStreamer: NO')
            torch = types.SimpleNamespace(__version__='actual-torch', version=types.SimpleNamespace(cuda='test-cuda'))
            args = detection.parse_args(['--repo-path', directory])
            with patch.dict(diagnostics.os.environ, {'UAV_DEBUG_LOG_DIR': directory}):
                diagnostics.write_runtime_snapshot(args, root / 'yolo11s.engine', cv2, torch)
            manifest_path = next(root.glob('detector_runtime_*/manifest.json'))
            import json
            manifest = json.loads(manifest_path.read_text())
            self.assertEqual(manifest['python_executable'], sys.executable)
            self.assertEqual(manifest['opencv_version'], 'actual-cv')
            self.assertIn('sha256', manifest['sources'][str(root / 'rtsp_capture.py')])
            self.assertEqual((manifest_path.parent / 'opencv_build.txt').read_text(), 'GStreamer: NO')

    def test_duplicate_source_rejected_then_released(self):
        source = 'test-source-' + str(time.monotonic())
        with detection.acquire_source_lock(source):
            with self.assertRaisesRegex(RuntimeError, 'already reading'):
                detection.acquire_source_lock(source)
        with detection.acquire_source_lock(source):
            pass

    def test_invalid_limits_rejected(self):
        for arguments in (['--max-fps', 'nan'], ['--cpu-threads', '0'], ['--video-fps', '-1'],
                          ['--read-timeout-ms', '0'], ['--open-timeout-ms', '-1'],
                          ['--reconnect-delay', 'nan'], ['--latency-ms', '-1'],
                          ['--max-frame-age-ms', '0']):
            with self.subTest(arguments=arguments), self.assertRaises(SystemExit):
                detection.parse_args(arguments)

    def test_save_toggle_preserves_inference_and_capture_cleanup(self):
        for save in (False, True):
            with self.subTest(save=save), tempfile.TemporaryDirectory() as directory:
                repo = Path(directory)
                (repo / 'rtsp_capture.py').touch()
                (repo / 'yolo11s.engine').touch()
                stop = threading.Event()
                model = Mock()
                result = types.SimpleNamespace(boxes=None, names={0: 'person'})
                calls = []
                def predict(**kw):
                    calls.append(kw)
                    if len(calls) == 2:
                        stop.set()
                    return [result]
                model.predict.side_effect = predict
                capture = Mock()
                capture.snapshot.return_value = {"status": "fake", "generation": 1}
                capture.read_latest.return_value = types.SimpleNamespace(
                    frame=np.zeros((16, 16, 3), np.uint8), captured_at=time.monotonic(),
                    cap_ms=1, backend='fake', sequence=(1, 1))
                upstream = Mock()
                modules = {
                    'cv2': Mock(__version__='test'), 'torch': Mock(),
                    'ultralytics': types.SimpleNamespace(YOLO=Mock(return_value=model)),
                    'a8mini_capture': types.SimpleNamespace(
                        CaptureConfig=Mock(), LatestFrameCapture=Mock(
                            return_value=Mock(start=Mock(return_value=capture)))),
                    'A8mini_RTSP_YOLO_Detection': upstream,
                }
                args = detection.parse_args(['--repo-path', directory, '--no-display',
                                             '--read-timeout-ms', '3200',
                                             '--open-timeout-ms', '6000',
                                             '--latency-ms', '250', '--reconnect-delay', '2',
                                             '--max-frame-age-ms', '800'] +
                                            (['--save-video'] if save else []))
                with patch.dict(sys.modules, modules), patch.object(detection.os, 'nice'), \
                        patch.object(detection, 'AsyncVideoWriter') as writer:
                    writer.return_value.written = 1
                    writer.return_value.dropped = 0
                    writer.return_value.error = None
                    detection.run(args, stop)
                    self.assertEqual(model.predict.call_count, 2)  # warmup + one live frame
                    modules['a8mini_capture'].CaptureConfig.assert_called_once_with(
                        source=args.source, backend=args.backend, codec=args.codec,
                        read_timeout_ms=3200, open_timeout_ms=6000, latency_ms=250,
                        reconnect_delay=2.0, max_frame_age_ms=800)
                    capture.close.assert_called_once()
                    self.assertEqual(writer.called, save)
                    if save:
                        writer.return_value.submit.assert_called_once()
                        writer.return_value.close.assert_called_once()

    def test_launch_command_boolean_switch_and_paths_with_spaces(self):
        spec = importlib.util.spec_from_file_location('supervisor', SCRIPTS / 'a8mini_detection_node.py')
        supervisor = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {'rospy': Mock()}):
            spec.loader.exec_module(supervisor)
        command = supervisor.build_command({'save_detection_video': True, 'display': False,
                                             'repo_path': '/tmp/repo with spaces'}, '/tmp/runner.py')
        self.assertIn('--save-video', command)
        self.assertIn('--no-display', command)
        self.assertIn('/tmp/repo with spaces', command)
        self.assertNotIn('--save-video', supervisor.build_command({'save_detection_video': False}, 'run.py'))
        with self.assertRaises(ValueError):
            supervisor.build_command({'save_detection_video': 'false'}, 'run.py')
        command = supervisor.build_command({'read_timeout_ms': 2500, 'max_fps': 8}, 'run.py')
        self.assertEqual(command[command.index('--read-timeout-ms') + 1], '2500')
        self.assertEqual(command[command.index('--max-fps') + 1], '8')

    def test_disabled_detection_never_spawns_worker_even_when_save_enabled(self):
        spec = importlib.util.spec_from_file_location('supervisor', SCRIPTS / 'a8mini_detection_node.py')
        supervisor = importlib.util.module_from_spec(spec)
        ros = Mock()
        with patch.dict(sys.modules, {'rospy': ros}):
            spec.loader.exec_module(supervisor)
        for params in ({}, {'enable_realtime_detection': False, 'save_detection_video': True},
                       {'enable_realtime_detection': 'false'}):
            with self.subTest(params=params), patch.object(supervisor.subprocess, 'Popen') as spawn:
                ros.get_param.return_value = params
                if isinstance(params.get('enable_realtime_detection'), str):
                    with self.assertRaises(ValueError):
                        supervisor.main()
                else:
                    self.assertEqual(supervisor.main(), 0)
                spawn.assert_not_called()

    def test_enabled_detection_starts_supervised_worker(self):
        spec = importlib.util.spec_from_file_location('supervisor', SCRIPTS / 'a8mini_detection_node.py')
        supervisor = importlib.util.module_from_spec(spec)
        ros = Mock()
        ros.get_param.return_value = {'enable_realtime_detection': True, 'save_detection_video': True}
        ros.is_shutdown.return_value = False
        with patch.dict(sys.modules, {'rospy': ros}):
            spec.loader.exec_module(supervisor)
        process = Mock(returncode=0, stdout=io.StringIO("native decoder message\n"))
        process.poll.return_value = 0
        with patch.object(supervisor.subprocess, 'Popen', return_value=process) as spawn, \
                patch.object(supervisor.os, 'killpg') as kill_group:
            self.assertEqual(supervisor.main(), 0)
            self.assertIn('--save-video', spawn.call_args.args[0])
            self.assertTrue(spawn.call_args.kwargs['start_new_session'])
            kill_group.assert_any_call(process.pid, supervisor.signal.SIGKILL)
            ros.on_shutdown.assert_called_once()


if __name__ == '__main__':
    unittest.main()
