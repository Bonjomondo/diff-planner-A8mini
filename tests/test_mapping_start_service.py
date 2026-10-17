import importlib.util
import io
import json
from pathlib import Path
import threading
import time
import unittest
from urllib.error import HTTPError
from urllib.request import Request, urlopen
from http.server import ThreadingHTTPServer

spec = importlib.util.spec_from_file_location("mapping_start", Path(__file__).resolve().parents[1] / "tools/mapping_start_service.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class StartupTests(unittest.TestCase):
    def setUp(self):
        self.release = threading.Event()
        self.calls = []
        owner = self
        class Child:
            stdout = io.StringIO("STAGE: 正在连接飞控\n")
            def terminate(self): owner.release.set()
            def wait(self):
                owner.release.wait(3)
                return 0
        def popen(command, **kwargs):
            self.calls.append(command)
            return Child()
        self.manager = module.MappingManager("/fixed/start_mapping_only.sh", popen=popen)
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), module.handler_for(self.manager))
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.url = "http://127.0.0.1:{}/".format(self.server.server_port)

    def tearDown(self):
        self.release.set()
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()

    def post(self, obj):
        with urlopen(Request(self.url + "ensure", data=json.dumps(obj).encode(), headers={"Content-Type": "application/json"}), timeout=2) as response:
            return json.load(response)

    def test_duplicate_requests_share_one_start_and_status_does_not_start(self):
        with urlopen(self.url + "status", timeout=2) as response:
            self.assertEqual(json.load(response)["state"], "idle")
        self.assertFalse(self.calls)
        for _ in range(5):
            self.assertEqual(self.post(dict(protocol=1, operation="ensure_mapping"))["state"], "starting")
        self.assertEqual(self.calls, [["/bin/bash", str(Path("/fixed/start_mapping_only.sh").resolve())]])
        self.release.set()
        for _ in range(100):
            if self.manager.status()["state"] == "ready": break
            time.sleep(.01)
        self.assertEqual(self.manager.status()["state"], "ready")

    def test_rejects_flight_and_arbitrary_script_requests(self):
        for payload in (dict(protocol=1, operation="takeoff"),
                        dict(protocol=1, operation="ensure_mapping", script="/tmp/anything"),
                        dict(protocol=True, operation="ensure_mapping"), [], "invalid"):
            with self.subTest(payload=payload), self.assertRaises(HTTPError) as error:
                self.post(payload)
            self.assertEqual(error.exception.code, 400)
        self.assertFalse(self.calls)

    def test_helper_failure_reported_and_retry_allowed(self):
        class FailedChild:
            stdout = io.StringIO("ERROR: 飞控未连接\n")
            def terminate(self): pass
            def wait(self): return 1
        self.manager.popen = lambda *args, **kwargs: FailedChild()
        self.manager.ensure()
        for _ in range(100):
            if self.manager.status()["state"] == "error": break
            time.sleep(.01)
        self.assertEqual(self.manager.status()["state"], "error")
        self.assertEqual(self.manager.status()["message"], "飞控未连接")


if __name__ == "__main__":
    unittest.main()
