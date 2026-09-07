#!/usr/bin/env python3
"""Low-rate, best-effort flight diagnostics. Standard library; no ROS/control APIs."""

import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import platform
import shutil
import signal
import subprocess
import threading
import time


def stamp():
    return {"wall_time": datetime.now().astimezone().isoformat(timespec="milliseconds"),
            "unix_sec": time.time(), "monotonic_sec": time.monotonic()}


class BoundedLog:
    """One writer per file. Rotation bounds disk usage; I/O errors disable only this log."""

    def __init__(self, path, max_bytes=20 * 1024 * 1024, backups=2):
        self.path, self.max_bytes, self.backups = Path(path), max_bytes, backups
        self.file = None
        self.size = 0
        self.error = None

    def write(self, row):
        if self.error:
            return False
        try:
            line = json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n"
            size = len(line.encode("utf-8"))
            if size > self.max_bytes:
                raise ValueError("single diagnostic record exceeds file limit")
            if self.file is None:
                self.file = self.path.open("a", encoding="utf-8", buffering=1)
                self.size = self.path.stat().st_size
            if self.size + size > self.max_bytes:
                self.file.close()
                self.file = None
                for index in range(self.backups, 0, -1):
                    src = self.path if index == 1 else Path(str(self.path) + "." + str(index - 1))
                    if src.exists():
                        src.replace(str(self.path) + "." + str(index))
                self.file = self.path.open("w", encoding="utf-8", buffering=1)
                self.size = 0
            self.file.write(line)
            self.size += size
            return True
        except (OSError, ValueError) as error:
            self.error = str(error)
            print("[DIAGNOSTICS] disabled %s: %s" % (self.path.name, error), flush=True)
            self.close()
            return False

    def close(self):
        if self.file is not None:
            try:
                self.file.close()
            except OSError:
                pass
            self.file = None


def read_text(path):
    try:
        return Path(path).read_text().strip()
    except OSError as error:
        return "unavailable: " + str(error)


def counter_rate(current, previous, elapsed):
    # Missing data and a device/process counter reset are not zero activity.
    if previous is None or elapsed <= 0 or current < previous:
        return None
    return round((current - previous) / elapsed, 3)


def parse_process_stat(text):
    # comm may contain spaces and parentheses; fields after its final ')' are fixed.
    fields = text[text.rindex(")") + 2:].split()
    return {"state": fields[0], "ppid": int(fields[1]),
            "cpu_ticks": int(fields[11]) + int(fields[12]),
            "threads": int(fields[17]), "start_ticks": int(fields[19]),
            "rss_pages": int(fields[21])}


class HostSampler:
    def __init__(self, log_dir, proc="/proc", sysfs="/sys"):
        self.proc, self.sysfs, self.log_dir = Path(proc), Path(sysfs), Path(log_dir)
        self.hz, self.page_size = os.sysconf("SC_CLK_TCK"), os.sysconf("SC_PAGE_SIZE")
        self.previous_time = None
        self.previous_cpu, self.previous_net, self.previous_process = {}, {}, {}

    def cpu(self):
        rows = {}
        for line in (self.proc / "stat").read_text().splitlines():
            parts = line.split()
            if not parts[0].startswith("cpu"):
                continue
            values = list(map(int, parts[1:9]))  # guest is already counted in user/nice
            previous = self.previous_cpu.get(parts[0])
            busy = iowait = None
            if previous is not None:
                delta = [a - b for a, b in zip(values, previous)]
                total = sum(delta)
                if total > 0 and all(x >= 0 for x in delta):
                    busy = round(100 * (total - delta[3] - delta[4]) / total, 2)
                    iowait = round(100 * delta[4] / total, 2)
            rows[parts[0]] = {"busy_percent": busy, "iowait_percent": iowait,
                              "ticks": values}
            self.previous_cpu[parts[0]] = values
        return rows

    def network(self, elapsed):
        rows = {}
        current = {}
        for line in (self.proc / "net/dev").read_text().splitlines()[2:]:
            name, counters = line.split(":", 1)
            name, values = name.strip(), list(map(int, counters.split()))
            current[name] = values
            previous = self.previous_net.get(name)
            rows[name] = {
                "rx_bytes": values[0], "rx_packets": values[1], "rx_errors": values[2],
                "rx_dropped": values[3], "tx_bytes": values[8], "tx_packets": values[9],
                "tx_errors": values[10], "tx_dropped": values[11],
                "rx_bytes_per_sec": counter_rate(values[0], previous[0] if previous else None, elapsed),
                "tx_bytes_per_sec": counter_rate(values[8], previous[8] if previous else None, elapsed),
            }
            for field in ("operstate", "carrier", "speed"):
                rows[name][field] = read_text(self.sysfs / "class/net" / name / field)
        self.previous_net = current
        return rows

    def processes(self, elapsed):
        candidates, selected = {}, set()
        for path in self.proc.iterdir():
            if not path.name.isdigit():
                continue
            try:
                stat = parse_process_stat((path / "stat").read_text())
                cmdline = (path / "cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace")
                candidates[int(path.name)] = (path, stat, cmdline)
                if any(name in cmdline for name in (
                        "a8mini_", "A8mini_RTSP", "px4ctrl", "mavros_node", "mapping_mid360",
                        "laserMapping", "faster_lio", "ekf", "diff_planner", "multipointplan", "rosbag record",
                        "flight_diagnostics.py", "rviz")):
                    selected.add(int(path.name))
            except (OSError, ValueError, IndexError):
                continue  # Process vanished while reading /proc.
        # multiprocessing spawn command lines do not name a8mini_capture.py.
        while True:
            expanded = selected | {pid for pid, (_, stat, _) in candidates.items()
                                   if stat["ppid"] in selected}
            if expanded == selected:
                break
            selected = expanded
        rows, current = [], {}
        for pid in sorted(selected):
            path, stat, cmdline = candidates[pid]
            key = (pid, stat["start_ticks"])
            previous = self.previous_process.get(key)
            current[key] = stat["cpu_ticks"]
            rate = counter_rate(stat["cpu_ticks"], previous, elapsed)
            row = dict(stat, pid=pid, cpu_percent_one_core=None if rate is None else round(rate / self.hz * 100, 2),
                       rss_bytes=stat["rss_pages"] * self.page_size,
                       # Do not dump full argv (RTSP credentials can be in arguments).
                       name=read_text(path / "comm"), wchan=read_text(path / "wchan"),
                       io=read_text(path / "io"))
            rows.append(row)
        self.previous_process = current
        return rows

    def sample(self):
        row = dict(stamp(), schema_version=1, event="sample")
        now = row["monotonic_sec"]
        elapsed = now - self.previous_time if self.previous_time is not None else 0
        row["sample_interval_sec"] = elapsed
        sections = {
            "cpu": self.cpu, "network": lambda: self.network(elapsed),
            "processes": lambda: self.processes(elapsed),
            "meminfo": lambda: read_text(self.proc / "meminfo"),
            "loadavg": lambda: read_text(self.proc / "loadavg"),
            "diskstats": lambda: read_text(self.proc / "diskstats"),
            "net_snmp": lambda: read_text(self.proc / "net/snmp"),
            "net_netstat": lambda: read_text(self.proc / "net/netstat"),
            "pressure": lambda: {kind: read_text(self.proc / "pressure" / kind)
                                  for kind in ("cpu", "io", "memory")},
            "thermal": lambda: {p.name: {"type": read_text(p / "type"), "temp_millideg_c": read_text(p / "temp")}
                                for p in (self.sysfs / "class/thermal").glob("thermal_zone*")},
            "disk_free_bytes": lambda: shutil.disk_usage(self.log_dir).free,
        }
        for name, reader in sections.items():
            try:
                row[name] = reader()
            except (OSError, ValueError, IndexError) as error:
                row[name] = {"unavailable": str(error)}
        self.previous_time = now
        row["sample_cost_ms"] = round((time.monotonic() - now) * 1000, 3)
        return row


class StreamProbe:
    """Own only the subprocess started here. Never use tegrastats --stop/pkill."""

    def __init__(self, argv, path, max_bytes):
        self.argv = argv
        self.log = BoundedLog(path, max_bytes)
        self.process = None
        self.thread = None
        self.pipe_lifetime = argv == ["sudo", "-n", "/usr/local/libexec/a8mini-kernel-log-reader"]

    def start(self):
        try:
            self.process = subprocess.Popen(self.argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                            stdin=subprocess.PIPE if self.pipe_lifetime else subprocess.DEVNULL,
                                            text=True, errors="replace", start_new_session=True)
        except OSError as error:
            self.log.write(dict(stamp(), event="unavailable", command=self.argv, error=str(error)))
            self.log.close()
            return self
        self.thread = threading.Thread(target=self._read, daemon=True)
        self.thread.start()
        return self

    def _read(self):
        self.log.write(dict(stamp(), event="started", command=self.argv, pid=self.process.pid))
        try:
            for line in self.process.stdout:
                if not self.log.write(dict(stamp(), event="output", text=line.rstrip()[:16384])):
                    break
        finally:
            self.process.stdout.close()
            # Do not block if the writer failed while the child is still running.
            self.log.write(dict(stamp(), event="reader_end", returncode=self.process.poll()))
            self.log.close()

    def close(self):
        if self.process is None:
            return
        if self.pipe_lifetime:
            # EOF lets the root-owned helper reap its own privileged child.
            self.process.stdin.close()
            self.process.wait(timeout=5)
        elif self.process.poll() is None:
            self.process.terminate()
        try:
            self.process.wait(timeout=1)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=1)
        if self.thread is not None:
            self.thread.join(timeout=1)


def command_snapshot(argv):
    try:
        result = subprocess.run(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, errors="replace", timeout=2)
        return dict(command=argv, returncode=result.returncode, output=result.stdout[:65536])
    except (OSError, subprocess.TimeoutExpired) as error:
        return dict(command=argv, unavailable=str(error))


def kernel_command():
    """Never prompt for credentials in the flight background process."""
    executable = shutil.which("dmesg") or "/usr/bin/dmesg"
    check = command_snapshot([executable, "--time-format", "iso"])
    if check.get("returncode") == 0:
        return [executable, "--follow", "--time-format", "iso"], {"mode": "direct"}
    # Fixed path/arguments match the narrowly scoped sudoers installation.
    command = ["sudo", "-n", "/usr/bin/dmesg", "--time-format", "iso"]
    elevated = command_snapshot(command)
    if elevated.get("returncode") == 0:
        return ["sudo", "-n", "/usr/local/libexec/a8mini-kernel-log-reader"], {"mode": "sudo_read_only"}
    return [executable, "--follow", "--time-format", "iso"], {
        "mode": "unavailable", "direct_error": check, "sudo_error": elevated,
        "action": "Run sudo bash sh_files/install_kernel_log_access.sh <username> once"}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log-dir", required=True)
    parser.add_argument("--camera-ip", default="192.168.144.25")
    parser.add_argument("--interval", type=float, default=1)
    parser.add_argument("--max-mb", type=int, default=20)
    parser.add_argument("--duration", type=float, default=0, help="0: until signal")
    parser.add_argument("--parent-pid", type=int, help="stop if debug launcher disappears")
    parser.add_argument("--no-probes", action="store_true", help="only read proc/sysfs")
    parser.add_argument("--no-ping", action="store_true")
    args = parser.parse_args(argv)
    if not math.isfinite(args.interval) or args.interval < 0.5 or args.max_mb < 1:
        parser.error("interval must be finite and >= 0.5; max-mb must be >= 1")
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error("duration must be finite and >= 0")
    # An IP literal avoids shell interpolation and accidental arbitrary ping options/DNS.
    import ipaddress
    try:
        ipaddress.ip_address(args.camera_ip)
    except ValueError:
        parser.error("camera-ip must be an IP address")
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: stop.set())
    directory = Path(args.log_dir)
    directory.mkdir(parents=True, exist_ok=True)
    try:
        os.nice(10)
    except OSError:
        pass
    host = BoundedLog(directory / "host_metrics.jsonl", args.max_mb * 1024 * 1024)
    probes = []
    started = time.monotonic()
    reason = "signal"
    exit_code = 0
    try:
        manifest = dict(stamp(), schema_version=1, kernel=platform.platform(), python=platform.python_version(),
                        boot_id=read_text("/proc/sys/kernel/random/boot_id"), options=vars(args),
                        units={"cpu_percent_one_core": "100 = one full CPU core; may exceed 100",
                               "network": "host/interface totals, not camera-only traffic",
                               "stream_times": "local receive time, not hardware event time"})
        if not args.no_probes:
            manifest["route"] = command_snapshot(["ip", "-j", "route", "get", args.camera_ip])
            manifest["interfaces"] = command_snapshot(["ip", "-j", "-s", "link"])
            kernel_argv, manifest["kernel_access"] = kernel_command()
            print("[DIAGNOSTICS] kernel_access=" + manifest["kernel_access"]["mode"], flush=True)
        (directory / "diagnostics_manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
        if not args.no_probes and not stop.is_set():
            commands = [("tegrastats", ["tegrastats", "--interval", str(int(args.interval * 1000))]),
                        ("kernel", kernel_argv)]
            if not args.no_ping:
                commands.append(("camera_ping", ["ping", "-n", "-D", "-O", "-i", "1", "-W", "1", args.camera_ip]))
            for name, command in commands:
                probes.append(StreamProbe(command, directory / (name + ".jsonl"),
                                          args.max_mb * 1024 * 1024).start())
        sampler = HostSampler(directory)
        next_sockets = started
        while not stop.is_set():
            if args.parent_pid and os.getppid() != args.parent_pid:
                reason = "parent_exited"
                break
            if args.duration and time.monotonic() - started >= args.duration:
                reason = "duration"
                break
            row = sampler.sample()
            if not args.no_probes and time.monotonic() >= next_sockets:
                probe_start = time.monotonic()
                row["camera_tcp"] = dict(stamp(), **command_snapshot(["ss", "-tin", "dst", args.camera_ip]))
                row["socket_probe_cost_ms"] = round((time.monotonic() - probe_start) * 1000, 3)
                next_sockets = time.monotonic() + 5
            if not host.write(row):
                reason = "host_log_error"
                exit_code = 1
                break
            # Retain space for existing flight logs; stop only this auxiliary collector.
            if isinstance(row.get("disk_free_bytes"), int) and row["disk_free_bytes"] < 256 * 1024 * 1024:
                reason = "low_disk_space"
                exit_code = 2
                break
            stop.wait(max(0, args.interval - (time.monotonic() - row["monotonic_sec"])))
    except Exception as error:
        reason = "collector_error: %s: %s" % (type(error).__name__, error)
        exit_code = 1
        print("[DIAGNOSTICS] " + reason, flush=True)
    finally:
        for probe in probes:
            probe.close()
        host.write(dict(stamp(), event="collector_exit", reason=reason,
                        probes=[{"command": p.argv, "returncode": p.process.poll() if p.process else None,
                                 "log_error": p.log.error} for p in probes]))
        host.close()
    print("[DIAGNOSTICS] stopped reason=" + reason, flush=True)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
