"""Seed RF2O with a valid quaternion; publish current LiDAR pose in tank_map."""
import math
import json


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener, TransformException
    rclpy.init()

    class TankPose(Node):
        def __init__(self):
            super().__init__('tank_pose')
            self.tf = Buffer(); self.listener = TransformListener(self.tf, self)
            self.seed = self.create_publisher(Odometry, '/tank/init_odom', 1)
            self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/tank/pose', 10)
            self.declare_parameter('session_id','unset'); self.declare_parameter('mode','mapping')
            self.session_pub = self.create_publisher(String,'/tank/session',1)
            self.create_timer(1.,lambda:self.session_pub.publish(String(data=json.dumps({
                'session_id':self.get_parameter('session_id').value,'mode':self.get_parameter('mode').value}))))
            self.create_timer(.1, self.update)

        def update(self):
            seed = Odometry(); seed.header.stamp = self.get_clock().now().to_msg()
            seed.header.frame_id = 'tank_odom'; seed.child_frame_id = 'velodyne'
            seed.pose.pose.orientation.w = 1.0
            # RF2O consumes only its first seed; repeating also permits a node restart.
            self.seed.publish(seed)
            try:
                tf = self.tf.lookup_transform('tank_map', 'velodyne', rclpy.time.Time())
            except TransformException:
                return
            age = (self.get_clock().now() - rclpy.time.Time.from_msg(tf.header.stamp)).nanoseconds / 1e9
            if age < -.1 or age > .5:
                return
            q = tf.transform.rotation
            if not .9 < math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w) < 1.1:
                return
            msg = PoseWithCovarianceStamped(); msg.header = tf.header
            msg.pose.pose.position.x = tf.transform.translation.x
            msg.pose.pose.position.y = tf.transform.translation.y
            msg.pose.pose.orientation = q
            # SLAM provides no calibrated pose covariance here. Do not invent one.
            self.pose_pub.publish(msg)

    node = TankPose()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
