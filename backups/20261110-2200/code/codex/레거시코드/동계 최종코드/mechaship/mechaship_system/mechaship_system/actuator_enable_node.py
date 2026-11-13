import subprocess

import rclpy
from mechaship_interfaces.srv import ActuatorEnable
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from ros2node.api import get_node_names


class ActuatorEnableNode(Node):
    def __init__(self) -> None:
        super().__init__(
            "actuator_enable_node", automatically_declare_parameters_from_overrides=True
        )

        # 파라미터 가져오기
        self.__key_min_degree = (
            self.get_parameter_or(
                "key_min_degree",
                Parameter("key_min_degree", Parameter.Type.DOUBLE, 0.0),
            )
            .get_parameter_value()
            .double_value
        )

        self.__key_max_degree = (
            self.get_parameter_or(
                "key_max_degree",
                Parameter("key_max_degree", Parameter.Type.DOUBLE, 180.0),
            )
            .get_parameter_value()
            .double_value
        )

        self.__key_pulse_0_degree = (
            self.get_parameter_or(
                "key_pulse_0_degree",
                Parameter("key_pulse_0_degree", Parameter.Type.INTEGER, 500),
            )
            .get_parameter_value()
            .integer_value
        )

        self.__key_pulse_180_degree = (
            self.get_parameter_or(
                "key_pulse_180_degree",
                Parameter("key_pulse_180_degree", Parameter.Type.INTEGER, 2500),
            )
            .get_parameter_value()
            .integer_value
        )

        self.__thruster_pulse_0_percentage = (
            self.get_parameter_or(
                "thruster_pulse_0_percentage",
                Parameter("thruster_pulse_0_percentage", Parameter.Type.INTEGER, 1500),
            )
            .get_parameter_value()
            .integer_value
        )

        self.__thruster_pulse_100_percentage = (
            self.get_parameter_or(
                "thruster_pulse_100_percentage",
                Parameter(
                    "thruster_pulse_100_percentage", Parameter.Type.INTEGER, 1900
                ),
            )
            .get_parameter_value()
            .integer_value
        )

        self.timer_cb = MutuallyExclusiveCallbackGroup()
        a = self.create_timer(
            1, self.__watch_node_timer_callback, callback_group=self.timer_cb
        )
        self.__last_node_status = False
        self.__last_warning = 0  # 0: no error, 1: domain id mismatch, 2: other error
        self.__err_cnt = 0

        self.msg = ActuatorEnable.Request()

    def __enable_actuator(self):
        self.__actuator_enable_client_hd = self.create_client(
            ActuatorEnable, "system/actuator/enable"
        )
        while not self.__actuator_enable_client_hd.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning(
                "actuator service not available, waiting again..."
            )

        self.msg.key_min_degree = self.__key_min_degree
        self.msg.key_max_degree = self.__key_max_degree
        self.msg.key_pulse_0_degree = self.__key_pulse_0_degree
        self.msg.key_pulse_180_degree = self.__key_pulse_180_degree
        self.msg.thruster_pulse_0_percentage = self.__thruster_pulse_0_percentage
        self.msg.thruster_pulse_100_percentage = self.__thruster_pulse_100_percentage

        self.future = self.__actuator_enable_client_hd.call_async(self.msg)
        self.executor.spin_until_future_complete(self.future)

        self.get_logger().info(
            "\033[32m메인보드와 연결되었습니다. (액추에이터 활성화됨)\033[0m"
        )

        self.__actuator_enable_client_hd.destroy()

    def __get_mcu_ros_domain_id(self) -> int:
        mcu_ros_domain_id = -1
        try:
            result = subprocess.run(
                ["socat", "-", "UNIX-CONNECT:/tmp/mechaship_service.sock"],
                input="get_mcu_ros_domain_id\n",
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=1,  # 1초 후 TimeoutExpired 예외 발생
            )
            mcu_ros_domain_id = int(result.stdout.strip())
        except:
            return mcu_ros_domain_id

        return mcu_ros_domain_id

    def __get_sbc_ros_domain_id(self) -> int:
        result = subprocess.run(
            [
                "bash",
                "-c",
                "source /home/ubuntu/ros2_ws/install/setup.bash && echo $ROS_DOMAIN_ID",
            ],
            stdout=subprocess.PIPE,
            text=True,
        )
        sbc_ros_domain_id = int(result.stdout.strip())
        return sbc_ros_domain_id

    def __watch_node_timer_callback(self):
        available_nodes = get_node_names(node=self)
        for name, namespace, full_name in available_nodes:
            if name == "mcu_node":
                if self.__last_node_status is False:
                    self.__last_node_status = True
                    self.__enable_actuator()
                self.__last_warning = 0
                self.__err_cnt = 0
                return
        self.__last_node_status = False

        # uROS disconnected
        mcu_ros_domain_id = self.__get_mcu_ros_domain_id()
        sbc_ros_domain_id = self.__get_sbc_ros_domain_id()

        if (
            mcu_ros_domain_id != -1
            and sbc_ros_domain_id != -1
            and mcu_ros_domain_id != sbc_ros_domain_id
        ):
            if self.__last_warning == 1:
                return

            if self.__err_cnt < 3:
                self.__err_cnt += 1
                return
            self.__last_warning = 1
            self.__err_cnt = 0

            # ROS_DOMAIN_ID mismatch
            self.get_logger().error(
                "메인보드와 SBC의 ROS_DOMAIN_ID가 서로 일치하지 않습니다."
            )
            self.get_logger().error(f"\tSBC: {sbc_ros_domain_id}")
            self.get_logger().error(f"\t메인보드: {mcu_ros_domain_id}")
            self.get_logger().error(f"")
            self.get_logger().error(
                f"만약 SBC의 ROS_DOMAIN_ID를 메인보드와 같게 변경하려면 Ctrl+C를 입력해 launch를 종료한 뒤 아래 명령을 차례대로 실행해주세요."
            )
            self.get_logger().error(
                f"\t$ sed -i 's/^export ROS_DOMAIN_ID=.*/export ROS_DOMAIN_ID={mcu_ros_domain_id}/' ~/ros2_ws/src/mechaship/mechaship_bringup/hooks/mechaship_bringup.sh.in"
            )
            self.get_logger().error(f"\t$ cb")

        else:
            if self.__last_warning == 2:
                return

            if self.__err_cnt < 3:
                self.__err_cnt += 1
                return

            self.__last_warning = 2
            self.__err_cnt = 0

            # other error
            self.get_logger().warning(
                "메인보드와 연결할 수 없습니다. 아래 내용을 점검해주세요."
            )
            self.get_logger().warning("\t 1. 조종기 설정이 UROS 모드인지 확인해주세요.")
            self.get_logger().warning(
                "\t 2. SBC와 메인보드 간의 USB 케이블이 올바르게 연결되어 있는지 확인해주세요."
            )


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = ActuatorEnableNode()

        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()

    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()


if __name__ == "__main__":
    main()
