"""Output ownership and short-lived navigation commands; no ROS or device I/O."""
from pathlib import Path
import math
import os
import threading


def acquire_output_lock():
    import fcntl
    root = Path(__file__).resolve().parents[3]
    domain = str(int(os.environ.get('ROS_DOMAIN_ID', '0')))
    directory = root / '.runtime' / 'output_locks'
    directory.mkdir(parents=True, exist_ok=True)
    handle = (directory / ('actuators_' + domain + '.lock')).open('a')
    try:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        handle.close()
        raise RuntimeError('Another project mission or thruster test owns actuator output') from None
    return handle


class CommandLease:
    """A separate output timer may use only a recent, completed navigation result."""
    def __init__(self, max_age=.25):
        self.max_age = max_age
        self.lock = threading.Lock()
        self.stamp = -math.inf
        self.pwm = (1500, 1500)
        self.allowed = False

    def publish(self, now, pwm, allowed):
        with self.lock:
            self.stamp, self.pwm, self.allowed = now, tuple(pwm), bool(allowed)

    def read(self, now):
        with self.lock:
            if not self.allowed:
                return (1500, 1500), 'navigation not ready'
            if not 0 <= now - self.stamp <= self.max_age:
                return (1500, 1500), 'navigation command expired'
            return self.pwm, ''


OUTPUT_NODE_NAMES = {'course_mission', 'shape_follow', 'gps_dock', 'shape_navigation',
                     'rc_auto_mode_test', 'outdoor_water_trial', 'indoor_water_trial'}


def conflicting_output_nodes(names, own_name):
    return sorted(name for name in OUTPUT_NODE_NAMES
                  if names.count(name) > (1 if name == own_name else 0))
