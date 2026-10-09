"""Independent RF2O and optional GPS-free EKF. Never launches actuator enable."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('field_calibration')
    return LaunchDescription([
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('odom_topic', default_value='/field/rf2o_odom'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_ekf', default_value='false',
                              description='Requires qualified odometry with measured covariance; see guide.'),
        DeclareLaunchArgument('ekf_config', default_value=os.path.join(share, 'config', 'ekf_no_gps.yaml')),
        Node(package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node', name='field_rf2o',
             output='screen', parameters=[{
                 'laser_scan_topic': LaunchConfiguration('scan_topic'),
                 'odom_topic': LaunchConfiguration('odom_topic'),
                 'base_frame_id': LaunchConfiguration('base_frame'),
                 'odom_frame_id': LaunchConfiguration('odom_frame'), 'publish_tf': False,
                 'init_pose_from_topic': '', 'freq': 10.0,
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}]),
        Node(package='robot_localization', executable='ekf_node', name='field_ekf', output='screen',
             condition=IfCondition(LaunchConfiguration('enable_ekf')),
             parameters=[LaunchConfiguration('ekf_config'), {
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}],
             remappings=[('odometry/filtered', '/field/odom')]),
    ])
