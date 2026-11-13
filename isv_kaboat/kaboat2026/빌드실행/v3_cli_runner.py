"""Own one V3 process and issue one guarded start, without a browser."""
import json
import math
import os
import signal
import subprocess
import sys
import time


def ready_to_start(report, received_at, now, launched_at):
    output = report.get('output') or {}
    stamp = report.get('stamp')
    return (isinstance(stamp, (int, float)) and math.isfinite(stamp)
            and stamp >= launched_at and 0 <= now - received_at <= .5
            and report.get('stage') == 'TO_WP1'
            and output.get('enabled') is True
            and output.get('start_ready') is True
            and not output.get('active') and not output.get('armed')
            and not output.get('error') and not output.get('blocker'))


def main():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    rclpy.init(args=[])
    node = rclpy.create_node('v3_cli_start_' + str(os.getpid()))
    latest = {}
    received_at = -float('inf')
    stopping = False
    child = None
    def receive(message):
        nonlocal latest, received_at
        try:
            value = json.loads(message.data)
            if isinstance(value, dict):
                latest = value
                received_at = time.monotonic()
        except (ValueError, TypeError):
            pass
    def stop(signum, frame):
        nonlocal stopping
        stopping = True
    subscription = node.create_subscription(String, '/course/status', receive, qos_profile_sensor_data)
    client = node.create_client(Trigger, '/course/start')
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        launched_at = time.time()
        child = subprocess.Popen([sys.executable, '-m', 'isv_shape_navigation.course_mission_v3', *sys.argv[1:]], start_new_session=True)
        deadline = time.monotonic() + 90
        issued = False
        request = None
        request_at = None
        announced_active = False
        next_print = 0
        print('V3 CLI: waiting for fresh sensors, MCU and RC AUTO; starts once when ready. Ctrl+C stops.', flush=True)
        while not stopping and child.poll() is None:
            rclpy.spin_once(node, timeout_sec=.1)
            now = time.monotonic()
            output = latest.get('output') or {}
            fresh = now - received_at <= .5 and latest.get('stamp', 0) >= launched_at
            if not issued:
                if fresh and output.get('active'):
                    issued = True
                    print('V3 already started by operator; no second start request.', flush=True)
                elif client.service_is_ready() and ready_to_start(latest, received_at, now, launched_at):
                    request = client.call_async(Trigger.Request())
                    request_at = now
                    issued = True
                    print('V3 CLI: one start request sent.', flush=True)
                elif now >= deadline:
                    print('V3 CLI: readiness timeout; no start sent. ' + str(output.get('blocker', 'no status')), flush=True)
                    return 2
                elif now >= next_print:
                    print('V3 WAIT: ' + str(output.get('blocker') or 'waiting for mission status'), flush=True)
                    next_print = now + 5
            if request is not None:
                if request.done():
                    response = request.result()
                    print('V3 START: ' + str(response.success) + ' ' + response.message, flush=True)
                    request = None
                    if not response.success:
                        return 2
                elif now - request_at > 10:
                    print('V3 START: response timeout; stopping this process; no retry.', flush=True)
                    return 2
            if fresh and output.get('active') and not announced_active:
                announced_active = True
                print('V3 RUNNING: propulsion command path active. No automatic restart after stop/fault.', flush=True)
        return child.returncode if child.poll() is not None else 0
    finally:
        if child is not None and child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            try:
                child.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGTERM)
                child.wait(timeout=5)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
