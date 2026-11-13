"""Choose dashboard SSH routes from the laptop's current Wi-Fi SSID."""
from copy import deepcopy
import locale
import os
import re
import subprocess
import threading
import time

REQUIRED_SSIDS = {'hotspot': 'Kk', 'starlink': 'welcome_CU'}
LABELS = {'hotspot': '\ud56b\uc2a4\ud31f', 'starlink': '\uc2a4\ud0c0\ub9c1\ud06c'}


def parse_ssid(text):
    # Anchoring excludes both BSSID and the remembered profile name.
    values = {line.strip() for line in re.findall(r'^\s*SSID\s*:\s*([^\r\n]*)', text, re.MULTILINE) if line.strip()}
    return next(iter(values)) if len(values) == 1 else None


def read_ssid():
    if os.name != 'nt':
        return None
    result = subprocess.run(['netsh', 'wlan', 'show', 'interfaces'], capture_output=True,
        timeout=2, check=True, creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
    try:
        text = result.stdout.decode('utf-8')
    except UnicodeDecodeError:
        text = result.stdout.decode(locale.getpreferredencoding(False), errors='replace')
    return parse_ssid(text)


def policy_for_ssid(ssid):
    allowed = {route: ssid == expected for route, expected in REQUIRED_SSIDS.items()}
    current = ssid or '\uc640\uc774\ud30c\uc774 \ubbf8\uc5f0\uacb0'
    reasons = {route: '' if allowed[route] else
        f'{LABELS[route]} \uc5f0\uacb0\uc740 {expected} Wi-Fi\uc5d0\uc11c\ub9cc \uac00\ub2a5\ud569\ub2c8\ub2e4. \ud604\uc7ac: {current}'
        for route, expected in REQUIRED_SSIDS.items()}
    return dict(ssid=ssid, allowed=allowed, required_ssids=dict(REQUIRED_SSIDS), reasons=reasons)


class WifiRoutes:
    def __init__(self, reader=None, clock=None, ttl=.75):
        self.reader = reader or read_ssid
        self.clock = clock or time.monotonic
        self.ttl = ttl
        self.lock = threading.RLock()
        self.cached = None
        self.updated = -float('inf')

    def snapshot(self, refresh=False):
        with self.lock:
            if self.cached is None or refresh or self.clock()-self.updated >= self.ttl:
                try:
                    ssid = self.reader()
                except (OSError, ValueError, subprocess.SubprocessError):
                    ssid = None
                self.cached = policy_for_ssid(ssid)
                self.updated = self.clock()
            return deepcopy(self.cached)

    def require(self, route):
        policy = self.snapshot()
        if route not in REQUIRED_SSIDS:
            raise ValueError('Unsupported connection route')
        if not policy['allowed'][route]:
            raise ValueError(policy['reasons'][route])
