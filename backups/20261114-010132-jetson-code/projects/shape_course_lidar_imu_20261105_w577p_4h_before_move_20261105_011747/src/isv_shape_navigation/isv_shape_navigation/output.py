"""Nonblocking paired service driver. No queued propulsion or automatic re-arm."""
import math


def telemetry_ready(payload):
    return (isinstance(payload, dict) and payload.get('receiving') is True
            and payload.get('mode') == 2 and payload.get('estop_active') is False
            and payload.get('stop_latched', False) is False)


class PairedOutput:
    """Adapters supply arm(bool), pair(left,right), stop(), ready().

    Both PWM futures must be submitted together before either is awaited: the
    upstream bridge pairs requests within 100 ms. A rejection latches a fault.
    """
    def __init__(self, arm, pair, stop, ready, timeout=.65, stop_ready=None):
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

    def start(self):
        if self.active or self.pending or self.stopping:
            return False
        self.active = True
        self.armed = False
        self.error = ''
        self.stop_ack = False
        return True

    def request_stop(self, reason='operator stop'):
        self.active = False
        self.armed = False
        self.error = reason
        self.stopping = self.owned

    def _send(self, futures, kind, now):
        self.pending = list(futures)
        self.pending_kind, self.sent_at = kind, now

    def tick(self, now, pwm, telemetry, telemetry_age):
        try:
            self._tick(now, pwm, telemetry, telemetry_age)
        except (RuntimeError, OSError, ValueError) as exc:
            self.pending = []
            self.request_stop('service error: '+str(exc))

    def _tick(self, now, pwm, telemetry, telemetry_age):
        if self.pending:
            if all(f.done() for f in self.pending):
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
        self._send(self.pair(*pwm), 'pair', now)
