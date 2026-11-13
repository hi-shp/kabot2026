"""Nonblocking paired service driver. No queued propulsion or automatic re-arm."""
import math
import threading


def telemetry_ready(payload):
    return (isinstance(payload, dict) and payload.get('receiving') is True
            and payload.get('mode') == 2 and payload.get('estop_active') is False
            and payload.get('stop_latched', False) is False)


class OutputReadiness:
    """Wait out the existing MCU's 7s ESC initialization without retrying arm."""
    def __init__(self, settle_seconds=7.5):
        self.settle_seconds = settle_seconds
        self.since = None
        self.previous_rc_valid = None
        self.lock = threading.RLock()

    def observe(self, now, telemetry, age):
        """Track reception and receiver reconnects even while navigation is busy."""
        with self.lock:
            fresh = (isinstance(telemetry, dict) and telemetry.get('receiving') is True
                     and math.isfinite(age) and 0 <= age <= .5)
            if not fresh:
                self.since = None
                self.previous_rc_valid = None
                return
            rc_valid = telemetry.get('rc_valid')
            if self.since is None or (self.previous_rc_valid is False and rc_valid is True):
                self.since = now
            self.previous_rc_valid = rc_valid

    def check(self, now, telemetry, age, services_ready):
        with self.lock:
            return self._check(now, telemetry, age, services_ready)

    def _check(self, now, telemetry, age, services_ready):
        self.observe(now, telemetry, age)
        fresh = (isinstance(telemetry, dict) and telemetry.get('receiving') is True
                 and math.isfinite(age) and 0 <= age <= .5)
        if not fresh:
            self.since = None
            return 'MCU 상태 수신 대기 / 통신 지연'
        if self.since is None:
            self.since = now
        if telemetry.get('estop_active') is not False:
            return 'MCU 비상정지 상태 확인 필요'
        if telemetry.get('stop_latched', False):
            return 'MCU 정지 잠금 상태'
        if telemetry.get('mode') != 2:
            return 'RC AUTO 모드 대기'
        remaining = self.settle_seconds - (now - self.since)
        if remaining > 0:
            return f'MCU / ESC 초기화 대기 {remaining:.1f}초'
        if not services_ready:
            return '추진기 arm / stop / 좌우 PWM 서비스 대기'
        return ''


class PairedOutput:
    """Adapters supply arm(bool), pair(left,right), stop(), ready().

    Both PWM futures must be submitted together before either is awaited: the
    upstream bridge pairs requests within 100 ms. A rejection latches a fault.
    """
    def __init__(self, arm, pair, stop, ready, timeout=.3, stop_ready=None):
        self.arm, self.pair, self.stop, self.ready = arm, pair, stop, ready
        self.stop_ready = stop_ready or ready
        self.timeout = timeout
        self.pending = []
        self.pending_kind = None
        self.sent_at = -math.inf
        self.active = self.armed = self.owned = self.stopping = False
        self.error = ''
        self.arm_ack_at = -math.inf
        self.stop_ack = False
        self.last_ack_pwm = None
        self.last_ack_at = None
        self.pending_pwm = None
        self.lock = threading.RLock()

    def start(self):
        with self.lock:
            return self._start()

    def _start(self):
        if self.active or self.pending or self.stopping:
            return False
        self.active = True
        self.armed = False
        self.error = ''
        self.stop_ack = False
        self.last_ack_pwm = self.last_ack_at = None
        return True

    def request_stop(self, reason='operator stop', *, force=False):
        with self.lock:
            self.active = False
            self.armed = False
            self.error = reason
            self.stopping = self.stopping or self.owned or force
            if force:
                self.stop_ack = False

    def _send(self, futures, kind, now):
        self.pending = list(futures)
        self.pending_kind, self.sent_at = kind, now

    def tick(self, now, pwm, telemetry, telemetry_age):
        with self.lock:
            try:
                self._tick(now, pwm, telemetry, telemetry_age)
            except (RuntimeError, OSError, ValueError) as exc:
                self.pending = []
                self.request_stop('service error: '+str(exc))

    def _tick(self, now, pwm, telemetry, telemetry_age):
        if self.pending:
            if self.pending_kind != 'stop' and now-self.sent_at > self.timeout:
                for future in self.pending:
                    if not future.done():
                        future.cancel()
                self.pending = []
                self.request_stop('service timeout')
            elif all(f.done() for f in self.pending):
                ok = all(not f.cancelled() and f.exception() is None and f.result() is not None
                         and bool(getattr(f.result(), 'success' if self.pending_kind != 'pair' else 'status', False))
                         for f in self.pending)
                kind = self.pending_kind
                self.pending = []
                if kind == 'stop':
                    self.stop_ack = ok
                    self.stopping = not ok
                    if not ok:
                        self.error = 'STOP_UNCONFIRMED'
                    return
                if not ok:
                    self.request_stop(kind+' rejected')
                elif kind == 'arm' and self.active:
                    self.armed = True
                    self.arm_ack_at = now
                elif kind == 'pair':
                    self.last_ack_pwm = self.pending_pwm
                    self.last_ack_at = now
            elif now-self.sent_at > self.timeout:
                for future in self.pending:
                    future.cancel()
                self.pending = []
                self.request_stop('service timeout')
            else:
                return
        if self.stopping:
            if self.stop_ready():
                self._send([self.stop()], 'stop', now)
            return
        if not self.active:
            return
        if (not self.ready() or telemetry_age > .5 or telemetry_age < 0
                or not telemetry_ready(telemetry)):
            self.request_stop('actuator telemetry unavailable or AUTO deselected')
            return
        if not self.armed:
            self.owned = True
            self._send([self.arm(True)], 'arm', now)
            return
        if telemetry.get('armed') is not True:
            if now-self.arm_ack_at > .5:
                self.request_stop('AUTO disarmed')
            return
        if (len(pwm) != 2 or any(type(p) is not int or not 1100 <= p <= 1900 for p in pwm)):
            self.request_stop('invalid PWM')
            return
        self.pending_pwm = tuple(pwm)
        self._send(self.pair(*pwm), 'pair', now)
