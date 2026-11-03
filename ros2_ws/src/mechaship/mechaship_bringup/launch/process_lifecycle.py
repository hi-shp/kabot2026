"""Own and reap only this sensor bringup's processes, with PID-reuse protection."""
import fcntl
import json
import os
from pathlib import Path
import signal
import time


def processes():
    result = {}
    for path in Path('/proc').glob('[0-9]*'):
        try:
            if path.stat().st_uid != os.getuid():
                continue
            fields = (path / 'stat').read_text().rsplit(')', 1)[1].split()
            result[int(path.name)] = (int(fields[1]), fields[19], fields[0],
                                      (path / 'cmdline').read_bytes().split(b'\0'))
        except (OSError, ValueError, IndexError):
            pass
    return result


def descendants(table, roots):
    found = set(roots)
    while True:
        extended = found | {pid for pid, value in table.items() if value[0] in found}
        if extended == found:
            return found
        found = extended


def terminate(identities):
    """SIGINT now; bound cleanup even when a driver ignores signals."""
    identities = dict(identities)
    identities.pop(os.getpid(), None)

    def remaining():
        table = processes()
        live = {pid for pid, start in identities.items()
                if pid in table and table[pid][1] == start and table[pid][2] != 'Z'}
        for pid in descendants(table, live) - {os.getpid()}:
            if pid in table:
                identities.setdefault(pid, table[pid][1])
        return {pid for pid, start in identities.items()
                if pid in table and table[pid][1] == start and table[pid][2] != 'Z'}

    for sig, grace in ((signal.SIGINT, 2.0), (signal.SIGTERM, .7), (signal.SIGKILL, .3)):
        active = remaining()
        if not active:
            break
        for pid in sorted(active):
            try:
                os.kill(pid, signal.SIGCONT)
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + grace
        while remaining() and time.monotonic() < deadline:
            time.sleep(.04)
    return remaining()


class BringupSession:
    def __init__(self, cache=None):
        self.cache = Path(cache) if cache is not None else Path.home() / '.cache/mechaship_bringup'
        self.journal = self.cache / 'all_sensors.processes.json'
        self.owned = {}
        self.lock = None
        self.closing = False

    def acquire(self):
        self.cache.mkdir(parents=True, exist_ok=True)
        with (self.cache / 'all_sensors.start.lock').open('a+') as startup:
            try:
                fcntl.flock(startup, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                raise RuntimeError('Another sensor bringup start is in progress')
            table = processes()
            own_start = int(table[os.getpid()][1])
            roots = {pid for pid, value in table.items()
                     if pid != os.getpid() and int(value[1]) < own_start
                     and any(a.endswith(b'/ros2') for a in value[3])
                     and all(a in value[3] for a in (b'launch', b'mechaship_bringup', b'all_sensors.launch.py'))}
            prior = {pid: table[pid][1] for pid in descendants(table, roots) if pid in table}
            try:
                saved = json.loads(self.journal.read_text())
                prior.update({int(pid): str(start) for pid, start in saved.items()})
            except (OSError, ValueError, TypeError, AttributeError):
                pass
            if terminate(prior):
                raise RuntimeError('Previous sensor bringup did not stop; new devices were not opened')
            lock = (self.cache / 'all_sensors.lock').open('a+')
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                lock.close()
                raise RuntimeError('Sensor bringup ownership lock is still held')
            lock.seek(0)
            lock.truncate()
            lock.write(str(os.getpid()))
            lock.flush()
            self.lock = lock
            self._save()

    def _save(self):
        temporary = self.journal.with_suffix('.%s.tmp' % os.getpid())
        temporary.write_text(json.dumps(self.owned))
        temporary.replace(self.journal)

    def started(self, event, context):
        table = processes()
        if event.pid in table:
            self.owned[event.pid] = table[event.pid][1]
            self._save()
            if self.closing or context.is_shutdown:
                terminate({event.pid: self.owned[event.pid]})
        return None

    def shutdown(self, event=None, context=None):
        self.closing = True
        table = processes()
        # Catch children whose ProcessStarted event has not been handled yet.
        for pid in descendants(table, {os.getpid()}) - {os.getpid()}:
            if pid in table:
                self.owned.setdefault(pid, table[pid][1])
        remaining = terminate(self.owned)
        table = processes()
        self.owned = {pid: table[pid][1] for pid in remaining if pid in table}
        self._save()
        if self.lock is not None:
            self.lock.close()
            self.lock = None
        print('[bringup] Sensor process cleanup complete; remaining=%s' % sorted(remaining), flush=True)
        return None
