import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)

from std_msgs.msg import Float32MultiArray
import math


class FusionSubscriber(Node):
    def __init__(self):
        super().__init__("fusion_subscriber")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1,
        )

        self.subscription = self.create_subscription(
            Float32MultiArray,
            "/fusion/roi_center",
            self.fusion_callback,
            qos_profile,
        )

        # track_id별 마지막 좌표/시간 저장
        self.last_pose = {}  # track_id -> (x, y, z)
        self.last_time = {}  # track_id -> t_sec

        self.get_logger().info("Started fusion_subscriber node, listening to /fusion/roi_center")

    def fusion_callback(self, msg: Float32MultiArray):
        """
        /fusion/roi_center 구독 (8열 고정)
        [u, v, class_id, track_id, x, y, z, range] * rows
        """
        data = list(msg.data) if msg.data else []

        # cols/rows 확인
        if msg.layout and msg.layout.dim and len(msg.layout.dim) >= 2:
            try:
                rows = int(msg.layout.dim[0].size)
                cols = int(msg.layout.dim[1].size)
            except Exception:
                rows, cols = (0, 8)
        else:
            cols = 8
            rows = len(data) // cols if cols else 0

        if cols != 8:
            self.get_logger().warn(f"/fusion/roi_center expects 8 cols, but got cols={cols}")
            return

        if rows == 0:
            self.get_logger().info("[fusion] no rows in this message")
            return

        objs = []
        for i in range(rows):
            base = i * 8
            try:
                u, v, classid, track_id, x, y, z, rng = data[base:base+8]
                obj = {
                    "u": float(u),
                    "v": float(v),
                    "class_id": int(classid),
                    "track_id": int(track_id),
                    "x": float(x), "y": float(y), "z": float(z),
                    "range": float(rng),
                }
                objs.append(obj)
            except Exception as e:
                self.get_logger().warn(f"fusion row parse fail at {i}: {e}")
                continue

        t_now = self.get_clock().now().nanoseconds * 1e-9  # sec

        for obj in objs:
            tid = obj["track_id"]
            cid = obj["class_id"]   # ★ class_id 추가
            x, y, z = obj["x"], obj["y"], obj["z"]
            rng = obj["range"]

            if tid in self.last_pose and tid in self.last_time:
                px, py, pz = self.last_pose[tid]
                t_prev = self.last_time[tid]

                dx, dy, dz = x - px, y - py, z - pz
                dist = math.sqrt(dx*dx + dy*dy + dz*dz)

                dt = t_now - t_prev
                if dt <= 0.0:
                    dt = 1e-6

                vx, vy, vz = dx / dt, dy / dt, dz / dt
                speed = dist / dt

                # ★ class_id 표시 추가
                self.get_logger().info(
                    f"[track {tid:3d} | class {cid}] "
                    f"xyz=({x:.3f},{y:.3f},{z:.3f}) m| R={rng:.2f} m"
                )
            else:
                # 첫 관측 로그도 class_id 표시
                self.get_logger().info(
                    f"[track {tid:3d} | class {cid}] first seen | "
                    f"xyz=({x:.3f},{y:.3f},{z:.3f}) m | R={rng:.2f} m"
                )

            self.last_pose[tid] = (x, y, z)
            self.last_time[tid] = t_now


def main(args=None):
    rclpy.init(args=args)
    node = FusionSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
