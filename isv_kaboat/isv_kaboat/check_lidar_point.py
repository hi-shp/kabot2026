#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class PrintXYZ(Node):
    def __init__(self):
        super().__init__("print_xyz_live")

        # Sensor Data용 QoS (BestEffort, Volatile, KeepLast)
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        # 또는: from rclpy.qos import QoSPresetProfiles
        # qos = QoSPresetProfiles.SENSOR_DATA.value

        self.sub = self.create_subscription(
            PointCloud2,
            "/ouster/points",
            self.callback,
            qos_profile=qos
        )

    def callback(self, msg: PointCloud2):
        print("\n--- New Frame ---")
        for i, (x, y, z) in enumerate(
            pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ):
            print(f"x={x:.3f}, y={y:.3f}, z={z:.3f}")
            if i >= 19:   # 프레임당 20개만 출력
                break

def main():
    rclpy.init()
    node = PrintXYZ()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
