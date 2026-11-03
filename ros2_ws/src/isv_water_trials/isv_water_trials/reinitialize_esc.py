"""Explicit ESC initialization request; the running sender resumes automatically."""
import time


def main(argv=None):
    import argparse
    p=argparse.ArgumentParser(description=__doc__)
    p.parse_args(argv)
    import rclpy
    from rclpy.node import Node
    from std_srvs.srv import Trigger
    rclpy.init();n=Node('reinitialize_esc')
    initialize=n.create_client(Trigger,'/water_trial/reinitialize_test')
    stop=n.create_client(Trigger,'/actuators/stop')
    def call(client):
        f=client.call_async(Trigger.Request());end=time.monotonic()+2
        while not f.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
        if not (f.done() and f.exception() is None and f.result() is not None and f.result().success):
            raise RuntimeError('ESC initialization request not acknowledged')
        return f.result().message
    try:
        deadline=time.monotonic()+5
        while time.monotonic()<deadline and not initialize.service_is_ready():
            rclpy.spin_once(n,timeout_sec=.02)
        if initialize.service_is_ready():
            print(call(initialize),flush=True)
        else:
            if not stop.wait_for_service(timeout_sec=3):raise RuntimeError('Actuator service unavailable')
            print(call(stop),flush=True)
            print('No current sender initialization service; neutral requested. Start the updated mode_test if needed.',flush=True)
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
