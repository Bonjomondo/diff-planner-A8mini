#!/usr/bin/python3 -I
"""Installed root-owned; fixed read-only command, lifetime tied to stdin pipe."""
import os
import subprocess
import sys

if len(sys.argv) != 1:
    raise SystemExit("No arguments accepted")
child = subprocess.Popen(["/usr/bin/dmesg", "--follow", "--time-format", "iso"],
                         stdin=subprocess.DEVNULL, env={"PATH": "/usr/bin:/bin", "LC_ALL": "C"})
try:
    while os.read(0, 4096):
        pass
finally:
    if child.poll() is None:
        child.terminate()
    try:
        child.wait(timeout=2)
    except subprocess.TimeoutExpired:
        child.kill()
        child.wait()
raise SystemExit(0)
