#include "iahrs_ros2_driver/iahrs_ros2_driver_node.hpp"

IAHRSDriverNode::IAHRSDriverNode(const std::string &node_name) : Node(node_name) {
  // -----------------------------------
  //  Get Params
  RCLCPP_INFO(this->get_logger(), "iAHRS Driver");
  RCLCPP_INFO(this->get_logger(), "Configuration:");

  this->declare_parameter("port", "/dev/ttyIMU");
  this->get_parameter("port", port_);
  RCLCPP_INFO(this->get_logger(), "\tport: \"%s\"", port_.c_str());

  this->declare_parameter("frame_id", "imu_link");
  this->get_parameter("frame_id", frame_id_);
  RCLCPP_INFO(this->get_logger(), "\tframe_id: \"%s\"", frame_id_.c_str());

  this->declare_parameter("publish_tf", false);
  this->get_parameter("publish_tf", publish_tf_);
  RCLCPP_INFO(this->get_logger(), "\tpublish_tf: %s", publish_tf_ ? "true" : "false");

  this->declare_parameter("sync_period_ms", 1000);
  this->get_parameter("sync_period_ms", sync_period_ms_);
  RCLCPP_INFO(this->get_logger(), "\tsync_period_ms: %s", sync_period_ms_ ? "true" : "false");

  this->declare_parameter("sync_sensor_accel", false);
  this->get_parameter("sync_sensor_accel", sync_sensor_accel_);
  RCLCPP_INFO(this->get_logger(), "\tsync_sensor_accel: %s", sync_sensor_accel_ ? "true" : "false");

  this->declare_parameter("sync_sensor_gyro", false);
  this->get_parameter("sync_sensor_gyro", sync_sensor_gyro_);
  RCLCPP_INFO(this->get_logger(), "\tsync_sensor_gyro: %s", sync_sensor_gyro_ ? "true" : "false");

  this->declare_parameter("sync_sensor_mag", false);
  this->get_parameter("sync_sensor_mag", sync_sensor_mag_);
  RCLCPP_INFO(this->get_logger(), "\tsync_sensor_mag: %s", sync_sensor_mag_ ? "true" : "false");

  this->declare_parameter("sync_sensor_quaternion", false);
  this->get_parameter("sync_sensor_quaternion", sync_sensor_quaternion_);
  RCLCPP_INFO(this->get_logger(), "\tsync_sensor_quaternion: %s", sync_sensor_quaternion_ ? "true" : "false");

  this->declare_parameter("enable_filter", false);
  this->get_parameter("enable_filter", enable_filter_);
  RCLCPP_INFO(this->get_logger(), "\tenable_filter: %s", enable_filter_ ? "true" : "false");

  // -----------------------------------
  //  iAHRS Init
  imu_driver_ = std::make_unique<IAHRSDriver>(port_);
  bool ret = imu_driver_.get()->initialize();
  if (ret != true) {
    stop_node_("Failed to connect IMU port");
  }
  ret = imu_driver_.get()->reboot();
  if (ret != true) {
    stop_node_("Failed to init IMU");
  }

  RCLCPP_INFO(this->get_logger(), "iAHRS Version:");
  RCLCPP_INFO(this->get_logger(),
              "\tH/W Version: %d.%d",
              imu_driver_.get()->get_version().hw.major, imu_driver_.get()->get_version().hw.minor);
  RCLCPP_INFO(this->get_logger(),
              "\tS/W Version: %d.%d",
              imu_driver_.get()->get_version().sw.major, imu_driver_.get()->get_version().sw.minor);

  ret = imu_driver_.get()->set_option(enable_filter_);
  if (ret != true) {
    stop_node_("Failed to init IMU");
  }

  if (is_sync_enabled_in_param()) {
    enable_sync_(get_sync_flag_from_param());
    sync_flag_ = imu_driver_.get()->get_sync_flag();
  }

  // -----------------------------------
  //  ROS Init
  auto qos = rclcpp::QoS(rclcpp::SensorDataQoS());

  imu_msg_.header.frame_id = frame_id_;
  mag_msg_.header.frame_id = frame_id_;
  imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", qos);
  mag_publisher_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", qos);

  restart_service_ =
      this->create_service<iahrs_ros2_driver_msgs::srv::Restart>(
          "imu/restart",
          [this](const std::shared_ptr<iahrs_ros2_driver_msgs::srv::Restart::Request> request,
                 std::shared_ptr<iahrs_ros2_driver_msgs::srv::Restart::Response> response) {
            this->restart_service_callback_(request, response);
          });

  reset_orientation_service_ =
      this->create_service<iahrs_ros2_driver_msgs::srv::ResetOrientation>(
          "imu/reset_heading",
          [this](const std::shared_ptr<iahrs_ros2_driver_msgs::srv::ResetOrientation::Request> request,
                 std::shared_ptr<iahrs_ros2_driver_msgs::srv::ResetOrientation::Response> response) {
            this->reset_orientation_service_callback_(request, response);
          });

  if (publish_tf_) {
    enable_tf_publish_();
  }

  RCLCPP_INFO(this->get_logger(), "iAHRS Driver Node has been started.");
}

void IAHRSDriverNode::enable_sync_(iahrs_driver_sync_flag_t data_flag) {
  (void)data_flag;
  RCLCPP_INFO(this->get_logger(), "Enable sync: ");
  if (sync_sensor_accel_)
    RCLCPP_INFO(this->get_logger(), "\tIAHRS_DRIVER_SYNC_FLAG_SENSOR_ACCEL");
  if (sync_sensor_gyro_)
    RCLCPP_INFO(this->get_logger(), "\tIAHRS_DRIVER_SYNC_FLAG_SENSOR_GYRO");
  if (sync_sensor_mag_)
    RCLCPP_INFO(this->get_logger(), "\tIAHRS_DRIVER_SYNC_FLAG_SENSOR_MAG");
  if (sync_sensor_quaternion_)
    RCLCPP_INFO(this->get_logger(), "\tIAHRS_DRIVER_SYNC_FLAG_QUATERNION");

  bool ret = imu_driver_.get()->set_sync(get_sync_flag_from_param(), (uint16_t)sync_period_ms_);
  if (ret == false) {
    stop_node_("Failed to start sync");
  }

  if (sync_data_timer_ == nullptr || sync_data_timer_.get()->is_canceled() == true) {
    sync_data_timer_ = this->create_wall_timer(
        std::chrono::microseconds(500),
        [this]() {
          this->sync_data_callback_();
        });
  }
}

void IAHRSDriverNode::enable_tf_publish_(void) {
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() {
        this->tf_publish_callback_();
      });
}

void IAHRSDriverNode::sync_data_callback_(void) {
  iahrs_driver_sync_data_t data_buff;
  bool ret;

  ret = imu_driver_.get()->fetch_sync_data(data_buff);
  if (ret == false) {
    return;
  }

  for (iahrs_driver_sync_flag_t i = static_cast<iahrs_driver_sync_flag_t>(IAHRS_DRIVER_SYNC_FLAG_NONE + 1);
       i < IAHRS_DRIVER_SYNC_FLAG_MAX;
       i = static_cast<iahrs_driver_sync_flag_t>(i << 1)) {

    if ((sync_flag_ & i) != i) {
      continue;
    }

    switch (i) {
    case IAHRS_DRIVER_SYNC_FLAG_SENSOR_ACCEL:
    case IAHRS_DRIVER_SYNC_FLAG_SENSOR_GYRO:
    case IAHRS_DRIVER_SYNC_FLAG_QUATERNION:
      imu_msg_.linear_acceleration.x = data_buff.accel.x * 9.80665;
      imu_msg_.linear_acceleration.y = data_buff.accel.y * 9.80665;
      imu_msg_.linear_acceleration.z = data_buff.accel.z * 9.80665;

      imu_msg_.angular_velocity.x = data_buff.gyro.x * (M_PI / 180.0);
      imu_msg_.angular_velocity.y = data_buff.gyro.y * (M_PI / 180.0);
      imu_msg_.angular_velocity.z = data_buff.gyro.z * (M_PI / 180.0);

      imu_msg_.orientation.x = data_buff.quaternion_angle.x;
      imu_msg_.orientation.y = data_buff.quaternion_angle.y;
      imu_msg_.orientation.z = data_buff.quaternion_angle.z;
      imu_msg_.orientation.w = data_buff.quaternion_angle.w;

      imu_msg_.header.stamp = get_clock().get()->now();
      imu_publisher_.get()->publish(imu_msg_);
      break;

    case IAHRS_DRIVER_SYNC_FLAG_SENSOR_MAG:
      mag_msg_.magnetic_field.x = data_buff.mag.x;
      mag_msg_.magnetic_field.y = data_buff.mag.y;
      mag_msg_.magnetic_field.z = data_buff.mag.z;

      mag_msg_.header.stamp = get_clock().get()->now();
      mag_publisher_.get()->publish(mag_msg_);
      break;

    default:
      break;
    }
  }
}

void IAHRSDriverNode::tf_publish_callback_(void) {
  geometry_msgs::msg::TransformStamped transformStamped;

  transformStamped.header.stamp = this->get_clock().get()->now();
  transformStamped.header.frame_id = "base_link";
  transformStamped.child_frame_id = frame_id_;

  transformStamped.transform.translation.x = 0.0;
  transformStamped.transform.translation.y = 0.0;
  transformStamped.transform.translation.z = 0.0;
  transformStamped.transform.rotation.x = 0.0;
  transformStamped.transform.rotation.y = 0.0;
  transformStamped.transform.rotation.z = 0.0;
  transformStamped.transform.rotation.w = 1.0;

  tf_broadcaster_->sendTransform(transformStamped);
}

void IAHRSDriverNode::stop_node_(const char *msg) {
  RCLCPP_ERROR(this->get_logger(), "Node has been stopped!");
  RCLCPP_ERROR(this->get_logger(), msg);
  throw msg;
}

void IAHRSDriverNode::restart_service_callback_(
    const std::shared_ptr<iahrs_ros2_driver_msgs::srv::Restart::Request> request,
    std::shared_ptr<iahrs_ros2_driver_msgs::srv::Restart::Response> response) {
  RCL_UNUSED(request);

  bool ret = imu_driver_.get()->reboot();
  if (ret) {
    imu_driver_.get()->set_option(enable_filter_);
  }
  if (is_sync_enabled_in_param()) {
    enable_sync_(get_sync_flag_from_param());
    sync_flag_ = imu_driver_.get()->get_sync_flag();
  }
  if (ret == true) {
    RCLCPP_INFO(this->get_logger(), "IMU Restarted.");
  } else {
    RCLCPP_WARN(this->get_logger(), "IMU Restart failed");
  }
  response.get()->result = ret;
}

void IAHRSDriverNode::reset_orientation_service_callback_(
    const std::shared_ptr<iahrs_ros2_driver_msgs::srv::ResetOrientation::Request> request,
    std::shared_ptr<iahrs_ros2_driver_msgs::srv::ResetOrientation::Response> response) {
  RCL_UNUSED(request);

  bool ret = imu_driver_.get()->reset_euler_angle();
  response.get()->result = ret;

  if (ret == true) {
    RCLCPP_INFO(this->get_logger(), "IMU Restarted.");
  } else {
    RCLCPP_WARN(this->get_logger(), "Heading reset failed.");
  }
}

int main(int argc, char *argv[]) {
  // ROS2 초기화
  rclcpp::init(argc, argv);

  std::shared_ptr<IAHRSDriverNode> node;

  // 노드 실행
  try {
    node = std::make_shared<IAHRSDriverNode>("iahrs_driver_node");
    rclcpp::spin(node);
  } catch (const char *e) {
    ;
  }

  // ROS2 종료
  rclcpp::shutdown();
  return 0;
}
