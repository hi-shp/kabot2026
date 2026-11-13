"""Persistent limits and a monotonic deadline for GPS targets 7 and 8."""
import json
import math

DEFAULT_LIMITS = {'point7_s': 60., 'orbit8_s': 60.}


def validate_limits(values):
    if not isinstance(values, dict):
        raise ValueError('Timeout limits must be an object')
    result = {}
    for key in DEFAULT_LIMITS:
        value = values[key]
        if type(value) not in (int, float) or not math.isfinite(value) or not 1 <= value <= 86400:
            raise ValueError('Timeout must be between 1 and 86400 seconds')
        result[key] = float(value)
    return result


def read_limits(path):
    try:
        return validate_limits(json.loads(path.read_text(encoding='utf-8')))
    except FileNotFoundError:
        return dict(DEFAULT_LIMITS)


class WaypointTimeouts:
    def __init__(self):
        self.limits = dict(DEFAULT_LIMITS)
        self.reset()

    def reset(self):
        self.key = None
        self.started = None
        self.limit_key = None

    def update(self, route, index, arrivals, executing, now):
        kinds = route.get('waypoint_types', []) if route else []
        kind = kinds[index] if 0 <= index < len(kinds) else None
        limit_key = 'point7_s' if index == 6 and kind == 'point' else 'orbit8_s' if index == 7 and kind == 'orbit' else None
        if not executing or limit_key is None:
            self.reset()
            return False
        key = (index, arrivals)
        if key != self.key:
            self.key = key
            self.started = now
        self.limit_key = limit_key
        return now - self.started >= self.limits[limit_key]

    def snapshot(self, now):
        elapsed = max(0., now-self.started) if self.started is not None else 0.
        limit = self.limits[self.limit_key] if self.limit_key else None
        return dict(limits=dict(self.limits), active_index=self.key[0] if self.key else None,
                    elapsed_s=elapsed, remaining_s=max(0., limit-elapsed) if limit is not None else None)
