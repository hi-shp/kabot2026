#!/usr/bin/env python3
"""Stop this boat's bringup and its children, including detached launches."""
import os
from pathlib import Path
import signal
import time


def snapshot():
    result = {}
    for proc in Path('/proc').glob('[0-9]*'):
        try:
            fields = (proc / 'stat').read_text().rsplit(')', 1)[1].split()
            args = (proc / 'cmdline').read_bytes().split(b'\0')
            result[int(proc.name)] = (int(fields[1]), fields[19], fields[0], args)
        except (OSError, ValueError, IndexError):
            pass
    return result


def matches(args):
    launch = (any(a.endswith(b'/ros2') for a in args)
              and b'launch' in args and b'mechaship_bringup' in args
              and any(a in args for a in (b'all_sensors.launch.py', b'mechaship_bringup.launch.py')))
    standalone = any(a.startswith(os.fsencode(Path.home() / 'ros2_ws/install') + b'/')
                     and a.rsplit(b'/', 1)[-1] in (b'mode_test', b'arduino_bridge', b'sensor_status.py')
                     for a in args)
    return launch or standalone


def stop():
    processes = snapshot()
    roots = {pid for pid, value in processes.items() if matches(value[3])}
    targets = set(roots)
    while True:
        expanded = targets | {pid for pid, value in processes.items() if value[0] in targets}
        if expanded == targets:
            break
        targets = expanded
    targets.discard(os.getpid())
    identities = {pid: processes[pid][1] for pid in targets}

    def remaining():
        current = snapshot()
        return {pid for pid, start in identities.items()
                if pid in current and current[pid][1] == start and current[pid][2] != 'Z'}

    # Stop respawn supervisors first. Their children also receive a clean SIGINT.
    for sig, delay in ((signal.SIGINT, 7), (signal.SIGTERM, 2), (signal.SIGKILL, 1)):
        active = remaining()
        if not active:
            break
        for pid in sorted(active, key=lambda p: p not in roots):
            try:
                os.kill(pid, signal.SIGCONT)
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + delay
        while remaining() and time.monotonic() < deadline:
            time.sleep(.1)
    active = remaining()
    if active:
        raise SystemExit('STOP_FAILED: ' + str(sorted(active)))
    print('BRINGUP_STOPPED:', len(targets), 'processes; device ports released')


if __name__ == '__main__':
    stop()
