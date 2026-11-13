"""Independent paired PWM sender adapted from the existing mode_test lifecycle.
Uses existing arm/throttle services, signal handling, ESC reinitialization and neutral ACK.
Importing this module never initializes ROS or commands actuators.
"""
import time
import math
from isv_water_trials.shutdown import neutralize

def validate_pair(left_pwm,right_pwm):
    pair=(left_pwm,right_pwm)
    if any(type(value) is not int or not 1100<=value<=1900 for value in pair):
        raise ValueError('LEFT_PWM and RIGHT_PWM must be integer logical PWM values in 1100..1900')
    return pair

class NeutralFirst:
    """Hold initialization PWM after its first acknowledgment, then allow thrust."""
    def __init__(self, pwm, seconds):
        self.pwm = validate_pair(*pwm)
        if type(seconds) not in (int, float) or not math.isfinite(seconds) or seconds <= 0:
            raise ValueError('INITIALIZE_SECONDS must be positive and finite')
        self.seconds = float(seconds)
        self.restart()

    def restart(self):
        self.accepted_since = None
        self.reported_phase = None

    def initializing(self, now):
        return self.accepted_since is None or now-self.accepted_since < self.seconds

    def acknowledge(self, ok, now):
        if not ok:
            self.restart()
        elif self.accepted_since is None:
            self.accepted_since = now


def run_pair(left_pwm,right_pwm, *, initialize_pwm=(1481,1481), initialize_seconds=3.0):
    from pathlib import Path
    import sys
    # The sensor-only shell sources the existing underlay, so locate this project's helper.
    source = Path(__file__).resolve().parents[1]/'공통모듈'/'isv_shape_navigation'
    if str(source) not in sys.path:
        sys.path.insert(0, str(source))
    from isv_shape_navigation.output_runtime import acquire_output_lock
    pair=validate_pair(left_pwm,right_pwm)
    initialization=NeutralFirst(initialize_pwm,initialize_seconds)
    with acquire_output_lock():
        return _send_pair(pair,initialization)

def _send_pair(pwm, initialization):
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from std_srvs.srv import SetBool, Trigger
    from mechaship_interfaces.srv import ThrottlePulseWidth
    from isv_water_trials.shutdown import ExitSignals
    from isv_shape_navigation.output_runtime import conflicting_output_nodes

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    exits = ExitSignals(ctrl_c_only=True)
    node = Node('rc_auto_mode_test')
    inflight = []
    conflict_checked = -float('inf')
    owned = False

    def reinitialize_test(request, response):
        initialization.restart()
        response.success = True
        response.message = 'Neutral initialization requested: %.1fs after MCU accepts initialization PWM; output then resumes automatically' % initialization.seconds
        return response

    reinitialize_service = node.create_service(Trigger, '/water_trial/reinitialize_test', reinitialize_test)
    arm = node.create_client(SetBool, '/actuators/arm')
    clients = [node.create_client(ThrottlePulseWidth, '/actuators/throttle_' + side + '/set_pulse_width')
               for side in ('left', 'right')]

    def stopping():
        return exits.requested

    def wait(futures, seconds=.6):
        inflight.extend(futures)
        until = time.monotonic() + seconds
        while not stopping() and not all(f.done() for f in futures) and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.01)
        inflight[:] = [f for f in inflight if not f.done()]
        if stopping():
            return False
        if not all(f.done() and not f.cancelled() and f.exception() is None
                   and f.result() is not None for f in futures):
            for f in futures:
                if not f.done():f.cancel()
            inflight[:] = [f for f in inflight if not f.done()]
            return False
        return True

    def drive():
        if stopping():
            return False
        initializing = initialization.initializing(time.monotonic())
        target = initialization.pwm if initializing else pwm
        futures = [c.call_async(ThrottlePulseWidth.Request(pulse_width=value))
                   for c, value in zip(clients, target)]
        ok = wait(futures) and all(f.result().status for f in futures)
        if initializing:
            initialization.acknowledge(ok, time.monotonic())
        elif not ok:
            initialization.restart()
        phase = 'INITIALIZING' if initializing else 'THRUST'
        if ok and phase != initialization.reported_phase:
            initialization.reported_phase = phase
            print(('INITIALIZING: logical PWM left=%d right=%d for %.1fs before thrust'
                   % (*initialization.pwm, initialization.seconds)) if initializing else
                  'THRUST: initialization hold completed; applying configured PWM', flush=True)
        return ok

    failed = False
    try:
        print('PWM stream: left=%dus right=%dus; continuous until Ctrl+C.' % pwm, flush=True)
        need_arm = True
        last_error = None
        while not stopping():
            tick = time.monotonic() + .1
            if time.monotonic()-conflict_checked>=.5:
                conflicts=conflicting_output_nodes(node.get_node_names(),'rc_auto_mode_test')
                conflict_checked=time.monotonic()
                if conflicts:
                    raise RuntimeError('Competing actuator sender: '+', '.join(conflicts))
            if not all(c.service_is_ready() for c in [arm, *clients]):
                need_arm = True
                initialization.restart()
            else:
                try:
                    if need_arm and not stopping():
                        initialization.restart()
                        owned = True
                        future = arm.call_async(SetBool.Request(data=True))
                        wait([future])
                    # Rejections do not end the stream; MCU owns RC/AUTO selection.
                    need_arm = not drive()
                    last_error = None
                except (RuntimeError,OSError) as exc:
                    need_arm = True
                    initialization.restart()
                    if str(exc)!=last_error:
                        print('ACTUATOR_RETRY: '+str(exc),flush=True)
                    last_error = str(exc)
            while not stopping() and time.monotonic() < tick:
                rclpy.spin_once(node, timeout_sec=.01)
    finally:
        ok = False
        try:
            # Drain requests already submitted before the final stop; a late arm
            # must not overtake the stop service when Ctrl+C arrives mid-request.
            until = time.monotonic() + .8
            while any(not f.done() for f in inflight) and time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.01)
            ok = neutralize(node) if owned else True
            print('No output acquired; no actuator commands sent.' if not owned else
                  'Neutral acknowledged. AUTO disarmed; RC resumes after sticks are centered for 0.5s.'
                  if ok else 'STOP_UNCONFIRMED: use physical EMO.', flush=True)
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            exits.restore()
        if not ok:
            failed = True
    if failed:
        raise SystemExit(1)

