#!/usr/bin/env python3

import importlib.util
import io
import json
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
from a8mini_labels import load_class_names, draw_detection
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
    def test_window_close_and_keyboard_exit(self):
        cv = Mock()
        cv.waitKey.return_value = -1
        cv.getWindowProperty.return_value = 0
        self.assertFalse(detection.window_should_close(cv, 'detector', False))
        cv.getWindowProperty.assert_not_called()
        self.assertTrue(detection.window_should_close(cv, 'detector', True))
        cv.getWindowProperty.return_value = 1
        self.assertFalse(detection.window_should_close(cv, 'detector', True))
        for key in (ord('q'), 27, 3):
            cv.waitKey.return_value = key
            self.assertTrue(detection.window_should_close(cv, 'detector', True))

    def test_runtime_snapshot_records_bundled_sources_and_model_hashes(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory).resolve()
            (root / 'yolo11s.engine').write_bytes(b'fake engine')
            (root / 'yolo11s.names.json').write_text('{"names": {"0": "person"}}')
            cv2 = types.SimpleNamespace(__version__='actual-cv', getBuildInformation=lambda: 'GStreamer: NO')
            torch = types.SimpleNamespace(__version__='actual-torch', version=types.SimpleNamespace(cuda='test-cuda'))
            args = detection.parse_args(['--model', str(root / 'yolo11s.engine')])
            with patch.dict(diagnostics.os.environ, {'UAV_DEBUG_LOG_DIR': directory}):
                diagnostics.write_runtime_snapshot(args, root / 'yolo11s.engine', cv2, torch)
            manifest_path = next(root.glob('detector_runtime_*/manifest.json'))
            manifest = json.loads(manifest_path.read_text())
            self.assertEqual(manifest['python_executable'], sys.executable)
            self.assertEqual(manifest['opencv_version'], 'actual-cv')
            self.assertIn('sha256', manifest['sources'][str(root / 'yolo11s.names.json')])
            self.assertIn('sha256', manifest['sources'][str(SCRIPTS / 'a8mini_labels.py')])
            self.assertEqual(manifest['model']['sha256'], diagnostics.sha256_file(root / 'yolo11s.engine'))
            self.assertEqual((manifest_path.parent / 'opencv_build.txt').read_text(), 'GStreamer: NO')

    def test_bundled_names_and_drawing(self):
        model_path = SCRIPTS.parent / 'models/yolo11s.engine'
        names = load_class_names(model_path)
        self.assertEqual(len(names), 80)
        self.assertEqual(names[0], 'person')
        frame = np.zeros((16, 16, 3), np.uint8)
        cv2 = Mock(FONT_HERSHEY_SIMPLEX=0, LINE_AA=16)
        cv2.getTextSize.return_value = ((20, 10), 2)
        draw_detection(frame, [-5, -5, 20, 20], 0, 0.9, names, cv2)
        self.assertEqual(cv2.rectangle.call_args_list[0].args[1:3], ((0, 0), (15, 15)))
        self.assertEqual(cv2.putText.call_args.args[1], 'person 0.90')

    def test_invalid_or_missing_names_fail_before_inference(self):
        with tempfile.TemporaryDirectory() as directory:
            model = Path(directory) / 'model.engine'
            with self.assertRaises(FileNotFoundError):
                load_class_names(model)
            names_file = model.with_suffix('.names.json')
            for names in ({'0': 'person', '2': 'car'}, {'0': 'class0'}, {'bad': 'person'}):
                with self.subTest(names=names):
                    names_file.write_text(json.dumps({'names': names}))
                    with self.assertRaises(ValueError):
                        load_class_names(model)

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
                model_path = Path(directory) / 'yolo11s.engine'
                model_path.touch()
                model_path.with_suffix('.names.json').write_text('{"names": {"0": "person"}}')
                stop = threading.Event()
                model = Mock()
                boxes = Mock()
                boxes.data.detach.return_value.cpu.return_value.numpy.return_value = np.array(
                    [[0, 0, 15, 15, 0.9, 0]])
                result = types.SimpleNamespace(boxes=boxes, names={0: 'class0'})
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
                modules = {
                    'cv2': Mock(__version__='test'), 'torch': Mock(),
                    'ultralytics': types.SimpleNamespace(YOLO=Mock(return_value=model)),
                    'a8mini_capture': types.SimpleNamespace(
                        CaptureConfig=Mock(), LatestFrameCapture=Mock(
                            return_value=Mock(start=Mock(return_value=capture)))),
                }
                modules['cv2'].getTextSize.return_value = ((20, 10), 2)
                args = detection.parse_args(['--model', str(model_path), '--no-display',
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
                    self.assertEqual(modules['cv2'].putText.call_args_list[0].args[1],
                                     'person 0.90')
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
                                             'model': '/tmp/model with spaces.engine'}, '/tmp/runner.py')
        self.assertIn('--save-video', command)
        self.assertIn('--no-display', command)
        self.assertIn('/tmp/model with spaces.engine', command)
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
