#!/usr/bin/env python3
"""Own one command tree; forward shutdown and reap it before reporting exit.

The command gets a separate session, so terminal Ctrl+C cannot bypass ordered
cleanup. Detached detector/capture groups are discovered by ancestry, never by
process name. Only descendants of this invocation may be signalled.
"""

import argparse
import ctypes
import os
import signal
import subprocess
import sys
import time


def process_table():
    if sys.platform.startswith('linux'):
        table = {}
        for name in os.listdir('/proc'):
            if not name.isdigit():
                continue
            try:
                with open('/proc/' + name + '/stat') as stream:
                    fields = stream.read().rsplit(')', 1)[1].split()
                # /proc stat fields 3(state), 4(ppid), 5(pgrp), 22(starttime).
                table[int(name)] = (int(fields[1]), int(fields[2]), fields[0], fields[19])
            except (OSError, IndexError, ValueError):
                continue  # Process exited during the snapshot.
        return table
    with subprocess.Popen(['ps', '-axo', 'pid=,ppid=,pgid=,stat=,lstart='],
                          stdout=subprocess.PIPE, text=True) as inspector:
        output = inspector.communicate()[0]
        if inspector.returncode:
            raise RuntimeError('cannot inspect the owned process tree')
        inspector_pid = inspector.pid
    table = {}
    for line in output.splitlines():
        parts = line.split(None, 4)
        if len(parts) == 5:
            pid, parent, group = map(int, parts[:3])
            if pid != inspector_pid:
                table[pid] = (parent, group, parts[3], parts[4])
    return table


class OwnedTree:
    def __init__(self, process):
        self.process = process
        self.known = {}

    def refresh(self):
        table = process_table()
        live = {pid for pid, birth in self.known.items()
                if pid in table and table[pid][3] == birth}
        if self.process.poll() is None:
            live.add(self.process.pid)
        # Linux subreaper ownership covers orphaned setsid capture workers too.
        live.update(pid for pid, row in table.items()
                    if row[0] == os.getpid() or row[1] == self.process.pid)
        while True:
            children = {pid for pid, row in table.items() if row[0] in live}
            new = children - live
            if not new:
                break
            live.update(new)
        self.known = {pid: table[pid][3] for pid in live if pid in table}
        return {pid: table[pid] for pid in live
                if pid in table and not table[pid][2].startswith('Z')}

    def send(self, sig):
        live = self.refresh()
        # Signal each owned PID (including setsid workers), rather than a global
        # pkill or the caller's foreground process group.
        for pid in live:
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass

    def wait(self, timeout):
        deadline = time.monotonic() + timeout
        while self.refresh():
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.05)
        return True


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--grace', type=float, default=20)
    parser.add_argument('--ignore-terminal-int', action='store_true')
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    command = args.command
    if command[:1] == ['--']:
        command = command[1:]
    if not command or args.grace < 0:
        parser.error('provide a command and a nonnegative grace period')
    requested = []
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda signum, _frame: requested.append(signum))
    if args.ignore_terminal_int:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
    if sys.platform.startswith('linux'):
        # PR_SET_CHILD_SUBREAPER: adopt orphaned capture grandchildren locally.
        libc = ctypes.CDLL(None, use_errno=True)
        if libc.prctl(36, 1, 0, 0, 0) != 0:
            raise OSError(ctypes.get_errno(), 'cannot enable child subreaper')
    process_table()  # Fail before launching anything if ancestry cannot be read.
    process = subprocess.Popen(command, start_new_session=True)
    tree = OwnedTree(process)
    try:
        while process.poll() is None and not requested:
            tree.refresh()
            time.sleep(0.1)
    finally:
        # Capture descendants before parents can disappear during shutdown.
        tree.send(signal.SIGINT)
        if not tree.wait(args.grace):
            print('[supervisor] Shutdown grace expired; sending TERM', flush=True)
            tree.send(signal.SIGTERM)
            if not tree.wait(3):
                print('[supervisor] Killing remaining owned processes', flush=True)
                tree.send(signal.SIGKILL)
                tree.wait(2)
        process.wait(timeout=2)
    return 128 + requested[0] if requested else process.returncode


if __name__ == '__main__':
    raise SystemExit(main())
