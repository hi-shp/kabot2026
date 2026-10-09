import math
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from .common import FieldNode, spin, stamp, qos_profile_sensor_data
from .core import StateGate, yaw

DEFAULTS = dict(scan_frame='base_scan', imu_frame='imu_link', odom_frame='odom',
                base_frame='base_footprint', max_age=0.5, future_tolerance=0.05,
                max_sensor_skew=0.12, min_scan_fraction=0.15, min_scan_sectors=4,
                max_position_variance=1.0, max_heading_variance=0.25,
                allow_unknown_covariance=False, min_dt=0.01, max_dt=0.3,
                max_speed=3.0, max_yaw_rate=1.5, max_yaw_disagreement=0.6)


class StateMonitor(FieldNode):
    def __init__(self):
        super().__init__('state_monitor')
        self.gate = StateGate({k: self.param(k, v) for k, v in DEFAULTS.items()})
        self.sign = self.param('imu_yaw_sign', 1.0)
        self.mount = self.param('imu_mount_yaw', 0.0)
        self.bias = self.param('imu_yaw_rate_bias', 0.0)
        if self.sign not in (-1.0, 1.0):
            raise ValueError('imu_yaw_sign must be +1 or -1')
        self.pub = self.json_publisher('/field/state')
        for kind, topic, cb in (
            (LaserScan, self.param('scan_topic', '/scan'), self.scan_cb),
            (Imu, self.param('imu_topic', '/imu'), self.imu_cb),
            (Odometry, self.param('odom_topic', '/scan/odom'), self.odom_cb)):
            self.create_subscription(kind, topic, cb, qos_profile_sensor_data)
        self.create_timer(0.25, self.tick)

    def scan_cb(self, msg):
        valid = [(i, v) for i, v in enumerate(msg.ranges)
                 if math.isfinite(v) and msg.range_min < v < msg.range_max]
        n = len(msg.ranges)
        sectors = len({min(7, int(((msg.angle_min+i*msg.angle_increment+math.pi) % (2*math.pi))/(math.pi/4)))
                       for i, _ in valid})
        self.gate.set_scan(dict(stamp=stamp(msg), received=self.now(), frame=msg.header.frame_id,
                                fraction=len(valid)/max(n, 1), sectors=sectors))

    def imu_cb(self, msg):
        try:
            heading = self.sign*yaw(msg.orientation)+self.mount if msg.orientation_covariance[0] >= 0 else None
        except ValueError:
            heading = None
        self.gate.set_imu(dict(stamp=stamp(msg), received=self.now(), frame=msg.header.frame_id,
                               r=self.sign*msg.angular_velocity.z-self.bias, heading=heading,
                               available=msg.angular_velocity_covariance[0] >= 0))

    def odom_cb(self, msg):
        try:
            heading = yaw(msg.pose.pose.orientation)
        except ValueError:
            heading = float('nan')
        cov = msg.pose.covariance
        self.gate.set_odom(dict(stamp=stamp(msg), received=self.now(), frame=msg.header.frame_id,
                                child=msg.child_frame_id, x=msg.pose.pose.position.x,
                                y=msg.pose.pose.position.y, heading=heading,
                                covariance=[cov[0], cov[7], cov[35]]))
        self.tick()

    def tick(self):
        state = self.gate.evaluate(self.now())
        # Covariance may be nonfinite in failed estimators; keep JSON standards-compliant.
        if state['covariance']:
            state['covariance'] = [v if math.isfinite(v) else None for v in state['covariance']]
        self.publish_json(self.pub, state)


def main(args=None):
    spin(StateMonitor, args)
