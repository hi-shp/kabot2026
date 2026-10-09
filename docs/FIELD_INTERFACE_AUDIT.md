# Field interface audit

Date: 2026-10-09 Asia/Seoul. Baseline remote field-calibration commit:
`d31054800c28b6a8372cae9fb7c96ee1bfd0d09e`.

## Source-confirmed facts

| Interface | Message / configuration | Evidence |
| --- | --- | --- |
| /scan | sensor_msgs/msg/LaserScan, base_scan, 10 Hz, 0.05–15 m | mechaship_bringup/param/mechaship_lidar.yaml and ydlidar driver source |
| /imu | sensor_msgs/msg/Imu, imu_link, nominal 30 ms | bringup launch remaps imu/data; mechaship_imu.yaml |
| IMU angular velocity | Raw gyro degrees/s converted to radians/s; quaternion copied from device | iahrs_ros2_driver/src/iahrs_ros2_driver_node.cpp:167–176 |
| IMU timestamp | Computer ROS receipt clock, not device acquisition clock | Same source, header.stamp = now() |
| /imu/yaw_refined | Float32, clockwise positive degrees, initial yaw removed | bringup/scripts/imu_reformatter.py |
| /actuator/thruster/percentage | std_msgs/msg/Float64 | joystick/keyboard teleop and isv test_thruster |
| /actuator/key/degree | std_msgs/msg/Float64 | joystick/keyboard teleop and isv test_key |
| Servo configuration | 30–150 degrees, 500–2500 microsecond pulse endpoints | bringup/param/mechaship_actuator.yaml |
| Servo teleop conventions | Neutral 90 degrees, teleop limits 60–120 degrees | teleop source; physical neutral unverified |
| Thruster configuration | 0 percent → 1500 microseconds; 100 percent → 2000 microseconds | bringup actuator YAML |
| Thruster joystick | Can produce negative percentages (up to -100), despite minimum constant 0 | joystick axis constrain; hardware reverse behavior unverified |
| Actuator enable | system/actuator/enable, mechaship_interfaces/srv/ActuatorEnable | system/actuator_enable_node.py auto-enables on mcu_node discovery |
| Actuator disable | mechaship_interfaces/srv/ActuatorDisable definition exists | Service definition only; runtime server and fail-safe behavior unverified |
| Emergency state | /sensor/emo/status, std_msgs/msg/Bool; true described as active | isv/launch_isv/test_code/test_EMO.py; hardware stop unverified |
| RF2O standalone | /odom_rf2o, nav_msgs/msg/Odometry; base_link → odom, 20 Hz, TF on | rf2o_laser_odometry/launch/rf2o_laser_odometry.launch.py |
| RF2O in existing EKF launch | /scan/odom; base_footprint → odom, 10 Hz, TF on | mechaship_slam/launch/ekf.launch.py |
| RF2O limitations | All covariance zero; twist.linear.y fixed 0; stamp = last scan used | CLaserOdometry2DNode.cpp publish() |
| Existing EKF | /scan/odom + /gps/odom + imu; outputs /odom, TF on | mechaship_slam/param/ekf.yaml and ekf.launch.py |
| Existing SLAM | Includes existing GPS/EKF launch before slam_toolbox | mechaship_slam/launch/slam_toolbox.launch.py |
| Body frames | base_footprint → base_link fixed; body height 0.139 m in model | mechaship_description/models/mechaship/model.sdf |
| Sensor TF definitions | Fixed base_link → imu_link and base_scan joints in SDF | Same model; actual installed publisher behavior must be checked |

RF2O and existing EKF both enable odom TF publication in the original launch:
verify actual TF authority before running that path. New gps_free launch disables
both broadcasters, uses private RF2O output, and does not start GPS, drivers,
actuator enable or competition code. No default launch files are overwritten.

The original state publisher passes SDF text to robot_state_publisher. Whether
the installed version accepts this description must be checked on the boat;
do not invent an identity sensor transform to hide a missing TF.

## Not established by source

The repository does not declare a dependable boat ROS distribution/Python
version. Tracked Python 3.12 cache files are not a platform specification.
Confirm ROS_DISTRO, python3 and rclpy on the actual boat. The development
machine uses ROS Lyrical/Python 3.14; this does not establish boat compatibility.

Missing physical evidence: actual servo neutral/port sign, allowable continuous
thruster output/reverse, IMU mounting axes and bias, LiDAR inversion/orientation,
hardware EMO operation, RC priority, MCU command expiry and output on ROS/SSH
loss. MCU firmware is absent from the relevant repository structure. Host
actuator_enable_node watches connectivity but does not enforce command expiry.
Automatic actuation is therefore unavailable, with no ARM override.

Raw sensor monitoring uses REP-103 (+x forward, +y port, +z up, CCW yaw).
It deliberately does not reuse the competition README's clockwise heading
coordinate convention or /imu/yaw_refined. imu_yaw_sign and imu_mount_yaw are
explicit planar corrections; tilted/downward IMU mounting needs a full verified
transform before use. No hardware geometry is inferred from a simulator model.

## Optional GPS-free EKF

config/ekf_no_gps.yaml and gps_free_odometry.launch.py provide isolated settings:
only qualified LiDAR position/yaw and raw IMU yaw rate, output /field/odom,
publish_tf false. The qualified input must include covariance established by
measurement or an estimator with meaningful covariance. This package does not
assign fictitious covariance to RF2O's zeros. Raw RF2O and validity-gated interval
state monitoring remain the immediate field path. EKF is off by default and
requires robot_localization installed separately. It has not been verified
against this vessel's data.
