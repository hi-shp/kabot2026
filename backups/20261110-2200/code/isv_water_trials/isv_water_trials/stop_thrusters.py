"""Request MCU neutral. A continuously running sender can subsequently re-arm."""
import argparse
import sys
import time


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--release',action='store_true',help='Legacy compatibility; no stop latch is used')
    parser.add_argument('--port',default='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0')
    args=parser.parse_args(argv)
    import rclpy
    from rclpy.node import Node
    from std_srvs.srv import Trigger
    from std_msgs.msg import Empty
    node=None;ok=False
    rclpy.init()
    try:
        node=Node('release_thruster_stop' if args.release else 'stop_thrusters_now')
        stop_test=node.create_publisher(Empty,'/water_trial/stop',10)
        name='/actuators/release_stop' if args.release else '/actuators/stop'
        client=node.create_client(Trigger,name)
        # Confirm trial cancellation before disarming the MCU. Topic and status
        # delivery have no ordering guarantee across independent DDS topics.
        if not args.release:
            cancel=node.create_client(Trigger,'/water_trial/stop_test')
            future=None
            matched=False
            deadline=time.monotonic()+2.
            while time.monotonic()<deadline:
                stop_test.publish(Empty())
                count=stop_test.get_subscription_count()
                matched=matched or count>0
                if future is None and cancel.service_is_ready():
                    future=cancel.call_async(Trigger.Request())
                rclpy.spin_once(node,timeout_sec=.02)
                if future is not None and future.done():
                    if future.exception() is None and future.result().success:break
                if matched and count==0:break
        if client.wait_for_service(timeout_sec=5.):
            for _ in range(3):
                future=client.call_async(Trigger.Request())
                until=time.monotonic()+.7
                while not future.done() and time.monotonic()<until:
                    rclpy.spin_once(node,timeout_sec=.01)
                if future.done() and not future.exception() and future.result().success:
                    ok=True;break
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    if not ok:
        # When bringup is off, talk to the UNO directly. Never steal an open port.
        from .arduino_bridge import SerialTransport
        transport=None
        try:
            transport=SerialTransport(args.port)
            until=time.monotonic()+3.
            while time.monotonic()<until:
                if args.release:
                    ok=transport.exchange('D') and transport.exchange('R')
                else:
                    ok=transport.exchange('S')
                if ok:break
                time.sleep(.03)
        except Exception as exc:
            print('STOP_UNCONFIRMED: '+str(exc),file=sys.stderr)
        finally:
            if transport is not None:transport.close()
    if not ok:
        print('MCU 응답이 없어 정지를 확인하지 못했습니다. 물리 EMO로 정지해 주세요.',file=sys.stderr)
        raise SystemExit(1)
    print('중립 명령 응답 확인. 계속 송신 중인 mode_test를 정지하려면 해당 터미널에서 Ctrl+C를 누르세요.')


if __name__=='__main__':
    main()
