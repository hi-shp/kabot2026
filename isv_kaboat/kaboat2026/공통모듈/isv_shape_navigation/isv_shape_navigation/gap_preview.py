"""Read-only V2 gap observations while GPS/heading cannot authorize navigation.

Uses the existing collision/width/scan-validity checks. The planner's returned
heading and all scores are discarded; no controller or output state is changed.
"""
from .avoidance_v2 import choose_gap


class GapPreview:
    WAIT_STATES = ('WAIT_GPS', 'GPS_UNCERTAIN', 'WAIT_HEADING', 'GPS_GOAL_TOO_FAR')

    def __init__(self):
        self.key = None
        self.value = None

    def snapshot(self, now, scan, raw_scan, route, follow, state):
        if state not in self.WAIT_STATES:
            return None
        base = dict(observation_only=True, selected_gap=None, command_deg=None,
                    state=state, gaps=[], obstacles=[], reason='라이다 최신 관측 대기')
        if scan is None or not -.1 <= now - scan.stamp <= follow.scan_timeout:
            self.key, self.value = None, None
            return dict(base, status='WAIT_SCAN', stamp=None)
        raw = raw_scan if (raw_scan is not None
            and -.1 <= now - raw_scan.stamp <= follow.scan_timeout
            and abs(raw_scan.stamp - scan.stamp) <= follow.sync_tolerance) else None
        radius = follow.footprint_radius + follow.clearance
        key = (id(scan), scan.stamp, id(raw), raw.stamp if raw else None, id(route), radius)
        if key != self.key:
            try:
                # Zero is a disposable reference for the existing pure function.
                # Never expose its selected angle, score, or WP_DIRECT decision.
                _, report = choose_gap(scan, 0., route, radius, raw_scan=raw)
                rows = [dict(index=i, left_deg=g['left_deg'], right_deg=g['right_deg'],
                    centre_deg=g['center_deg'], width_m=g['width_m'], selected=False)
                    for i, g in enumerate(report['gaps'], 1)]
                self.value = dict(gaps=rows, obstacles=report['obstacles'],
                    check_reason=report['reason'], diagnostics=report['diagnostics'])
            except (ValueError, RuntimeError, OverflowError):
                self.key, self.value = None, None
                return dict(base, status='INVALID', stamp=scan.stamp)
            self.key = key
        value = dict(base)
        value.update(self.value)
        count = len(value['gaps'])
        value.update(status='LIVE', stamp=scan.stamp, age_s=max(0., now-scan.stamp),
            reason=('LiDAR 관측 후보 '+str(count)+'개 · GPS/방위 입력 대기로 경로 미선택'
                if count else 'LiDAR 관측 후보 없음 · '+value['check_reason']+' · 이동 명령 없음'))
        return value
