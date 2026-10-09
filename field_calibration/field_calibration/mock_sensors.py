"""Development-only numeric sensor source. NEVER run on the vessel ROS domain."""
import math
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from rclpy.qos import QoSProfile, DurabilityPolicy
from .common import FieldNode, spin


class MockSensors(FieldNode):
    def __init__(self):
        super().__init__('field_mock_sensors')
        self.started = self.now()
        self.x = self.y = self.heading = self.u = self.r = 0.0
        self.pub_scan = self.create_publisher(LaserScan, '/scan', 10)
        self.pub_imu = self.create_publisher(Imu, '/imu', 10)
        self.pub_odom = self.create_publisher(Odometry, '/scan/odom', 10)
        # Commands are under /field/mock/*: the test fixture can never reach actuators.
        self.pub_throttle = self.create_publisher(Float64, '/field/mock/throttle', 10)
        self.pub_steering = self.create_publisher(Float64, '/field/mock/steering', 10)
        self.pub_tf = self.create_publisher(TFMessage, '/tf_static',
                                            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id, transform.child_frame_id = 'base_footprint', 'base_scan'
        transform.transform.rotation.w = 1.
        self.pub_tf.publish(TFMessage(transforms=[transform]))
        for k in ('drop_scan', 'drop_imu', 'drop_odom', 'zero_covariance'):
            self.param(k, False)
        self.param('imu_stamp_offset', 0.0)
        self.param('phase', 0.0)
        self.create_timer(0.05, self.tick)

    def tick(self):
        t = self.now()-self.started+self.get_parameter('phase').value
        throttle = 10+8*math.sin(t*0.45)+2*math.sin(t*1.8)
        steering = 90+15*math.sin(t*0.7)
        self.u += (-0.7*self.u+0.045*throttle)*0.05
        self.r += (-1.2*self.r+1.8*self.u*math.radians(steering-90))*0.05
        self.heading += self.r*0.05
        self.x += self.u*math.cos(self.heading)*0.05
        self.y += self.u*math.sin(self.heading)*0.05
        stamp = self.get_clock().now().to_msg()
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = stamp, 'base_scan'
        scan.angle_min, scan.angle_max, scan.angle_increment = -math.pi, math.pi, math.pi/180
        scan.range_min, scan.range_max = 0.05, 15.0
        scan.ranges = [4.0]*360
        imu = Imu()
        ns = int((self.now()+self.get_parameter('imu_stamp_offset').value)*1e9)
        imu.header.stamp.sec, imu.header.stamp.nanosec = ns//1000000000, ns % 1000000000
        imu.header.frame_id = 'imu_link'
        imu.orientation.z, imu.orientation.w = math.sin(self.heading/2), math.cos(self.heading/2)
        imu.angular_velocity.z = self.r
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, 'odom', 'base_footprint'
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation = imu.orientation
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = self.u, self.r
        if not self.get_parameter('zero_covariance').value:
            odom.pose.covariance[0] = odom.pose.covariance[7] = odom.pose.covariance[35] = 0.01
        if not self.get_parameter('drop_scan').value:
            self.pub_scan.publish(scan)
        if not self.get_parameter('drop_imu').value:
            self.pub_imu.publish(imu)
        if not self.get_parameter('drop_odom').value:
            self.pub_odom.publish(odom)
        self.pub_throttle.publish(Float64(data=throttle))
        self.pub_steering.publish(Float64(data=steering))


def main(args=None):
    spin(MockSensors, args)
