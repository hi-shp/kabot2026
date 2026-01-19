#ifndef INCLUDE_IAHRS_ROS2_DRIVER_IAHRS_ROS2_DRIVER_NODE
#define INCLUDE_IAHRS_ROS2_DRIVER_IAHRS_ROS2_DRIVER_NODE

#include <iostream>
#include <stdbool.h>

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include "iahrs_ros2_driver_msgs/srv/reset_orientation.hpp"
#include "iahrs_ros2_driver_msgs/srv/restart.hpp"

#include "iahrs_driver.hpp"
#include "rclcpp/rclcpp.hpp"

class IAHRSDriverNode : public rclcpp::Node {
public:
  IAHRSDriverNode(const std::string &node_name);

private:
  std::unique_ptr<IAHRSDriver> imu_driver_;

  rclcpp::TimerBase::SharedPtr sync_data_timer_;
  rclcpp::TimerBase::SharedPtr tf_timer_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  sensor_msgs::msg::Imu imu_msg_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_publisher_;
  sensor_msgs::msg::MagneticField mag_msg_;
  rclcpp::Service<iahrs_ros2_driver_msgs::srv::Restart>::SharedPtr restart_service_;
  rclcpp::Service<iahrs_ros2_driver_msgs::srv::ResetOrientation>::SharedPtr reset_orientation_service_;
  iahrs_ros2_driver_msgs::srv::Restart restart_srv_;
  iahrs_ros2_driver_msgs::srv::ResetOrientation reset_orientation_srv_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  iahrs_driver_sync_flag_t sync_flag_;

  std::string port_;
  std::string frame_id_;
  bool publish_tf_;
  int sync_period_ms_;
  bool sync_sensor_accel_;
  bool sync_sensor_gyro_;
  bool sync_sensor_mag_;
  bool sync_sensor_quaternion_;
  bool enable_filter_;

  inline bool is_sync_enabled_in_param(void) {
    return sync_sensor_accel_ | sync_sensor_gyro_ | sync_sensor_mag_ | sync_sensor_quaternion_;
  }

  inline iahrs_driver_sync_flag_t get_sync_flag_from_param(void) {
    return (iahrs_driver_sync_flag_t)((sync_sensor_accel_ ? IAHRS_DRIVER_SYNC_FLAG_SENSOR_ACCEL : 0) |
                                      (sync_sensor_gyro_ ? IAHRS_DRIVER_SYNC_FLAG_SENSOR_GYRO : 0) |
                                      (sync_sensor_mag_ ? IAHRS_DRIVER_SYNC_FLAG_SENSOR_MAG : 0) |
                                      (sync_sensor_quaternion_ ? IAHRS_DRIVER_SYNC_FLAG_QUATERNION : 0));
  }

  void enable_sync_(iahrs_driver_sync_flag_t data_flag);
  void enable_tf_publish_(void);

  void stop_node_(const char *msg);

  void sync_data_callback_(void);
  void tf_publish_callback_(void);

  void restart_service_callback_(
      const std::shared_ptr<iahrs_ros2_driver_msgs::srv::Restart::Request> request,
      std::shared_ptr<iahrs_ros2_driver_msgs::srv::Restart::Response> response);
  void reset_orientation_service_callback_(
      const std::shared_ptr<iahrs_ros2_driver_msgs::srv::ResetOrientation::Request> request,
      std::shared_ptr<iahrs_ros2_driver_msgs::srv::ResetOrientation::Response> response);
};

#endif /* INCLUDE_IAHRS_ROS2_DRIVER_IAHRS_ROS2_DRIVER_NODE */
