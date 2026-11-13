"""Explicit, persistent GPS-path recording permission, independent of SSH."""
import json
from pathlib import Path
import time


class GpsRecordingGate:
    def __init__(self, path=None):
        self.path=Path(path) if path else Path.home()/'water_trials/gps_paths/.active.json'
        self.checked=-float('inf');self.version=None;self.active=None

    def session(self):
        now=time.monotonic()
        if now-self.checked<.25:return self.active
        self.checked=now
        try:
            stat=self.path.stat();version=(stat.st_mtime_ns,stat.st_size)
            if version!=self.version:
                value=json.loads(self.path.read_text(encoding='utf-8'))
                self.active=value if value and value.get('recording') is True else None
                self.version=version
        except (OSError,ValueError,AttributeError):self.active=None;self.version=None
        return self.active
