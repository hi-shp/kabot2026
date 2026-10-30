"""Continuous PWM sender until Ctrl+C. RC/AUTO arbitration belongs to MCU."""
import argparse
import math
import sys
import time
from .shutdown import neutralize


def test_pwm(percent, side='both'):
    if side not in ('left', 'right', 'both'):
        raise ValueError('--side must be left, right or both')
    if not math.isfinite(percent) or not -100 <= percent <= 100:
        raise ValueError('--percent must be -100..100')
    pulse = 1500 + round(400 * percent / 100)
    return (pulse if side in ('left', 'both') else 1500,
            pulse if side in ('right', 'both') else 1500)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--percent', type=float, default=10., help='Signed PWM percentage (-100..100)')
    parser.add_argument('--seconds', type=float, default=None,
                        help='Legacy argument accepted but ignored; only explicit stop ends execution')
    parser.add_argument('--side', choices=('left', 'right', 'both'), default='both')
    args = parser.parse_args(argv)
    try:
        pwm = test_pwm(args.percent, args.side)
    except ValueError as exc:
        parser.error(str(exc))

    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from std_srvs.srv import SetBool, Trigger
    from mechaship_interfaces.srv import ThrottlePulseWidth
    from .shutdown import ExitSignals

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    exits = ExitSignals(ctrl_c_only=True)
    node = Node('rc_auto_mode_test')
    reinitialize_pending = False
    reinitialize_until = 0.
    inflight = []

    def reinitialize_test(request, response):
        nonlocal reinitialize_pending
        reinitialize_pending = True
        response.success = True
        response.message = 'Neutral initialization requested: 7s after MCU accepts neutral; output then resumes automatically'
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
        nonlocal reinitialize_pending, reinitialize_until
        if stopping():
            return False
        initializing = reinitialize_pending or time.monotonic() < reinitialize_until
        target = (1500, 1500) if initializing else pwm
        futures = [c.call_async(ThrottlePulseWidth.Request(pulse_width=value))
                   for c, value in zip(clients, target)]
        ok = wait(futures) and all(f.result().status for f in futures)
        if ok and initializing and reinitialize_pending:
            reinitialize_pending = False
            reinitialize_until = time.monotonic() + 7.
        return ok

    failed = False
    try:
        print('PWM stream: left=%dus right=%dus; continuous until Ctrl+C.' % pwm, flush=True)
        if args.seconds is not None:
            print('--seconds is ignored; automatic time-based exit is disabled.',flush=True)
        need_arm = True
        last_error = None
        while not stopping():
            tick = time.monotonic() + .1
            if not all(c.service_is_ready() for c in [arm, *clients]):
                need_arm = True
            else:
                try:
                    if need_arm and not stopping():
                        future = arm.call_async(SetBool.Request(data=True))
                        wait([future])
                    # Rejections do not end the stream; MCU owns RC/AUTO selection.
                    need_arm = not drive()
                    last_error = None
                except (RuntimeError,OSError) as exc:
                    need_arm = True
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
            ok = neutralize(node)
            print('Neutral acknowledged. AUTO disarmed; RC resumes after sticks are centered for 0.5s.'
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


if __name__ == '__main__':
    main()
