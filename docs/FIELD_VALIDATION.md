# Field calibration software validation

Date: 2026-10-09 Asia/Seoul. No real vessel or sensor hardware was used.
Baseline d310548 had no independent field_calibration package. Existing isv,
drivers, teleop and SLAM/EKF files have zero changes.

## Verified environment

Ubuntu 26.04, installed ROS2 Lyrical, Python 3.14. The host's default python3 is
3.10 and cannot load that rclpy extension. Tests/builds used the matching
/usr/bin/python3.14 without modifying the system Python or ROS installation.
Python 3.10 and 3.14 syntax compilation both pass; runtime compatibility on a
different boat distribution is not established by compilation.

## Executed checks

| Check | Result |
| --- | --- |
| Python compile, package.xml parse, all YAML parse | PASS |
| colcon build --symlink-install --packages-select field_calibration | PASS, isolated build/install under /tmp |
| Installed console scripts | 7 present |
| ros2 run motion_identifier --help | PASS |
| ros2 run field_logger | PASS, SIGINT exit 0; CSV/bag finalized, model snapshot identical |
| ros2 launch field_calibration field_test.launch.py | PASS, real DDS and installed nodes |
| Pure unit tests | 22 passed |
| Mock scan/imu/odom and command subscriptions | PASS |
| LiDAR loss, IMU loss, RF2O loss | INVALID then recovery, no invented position |
| IMU +0.8 s timestamp fault | INVALID then recovery |
| RF2O zero covariance | INVALID under default policy |
| Missing/uncalibrated model | Labelled baseline; missing file warning does not terminate predictor |
| Prediction/error CSV | PASS, 0.5/1/2 s time-aligned observations |
| PASSIVE actuator publishers | 0 on both actual actuator topics |
| automatic_drive true parameter | Rejected |
| Late /tf_static subscription | Latched message recorded with original QoS metadata |
| Raw rosbag2 read | 1433 messages in final telemetry integration fixture |
| ros2 bag play | At least 10 remapped scans received |
| /clock raw replay | States recomputed, synthetic model loaded, predictions/errors written |
| Viewer | Numeric DDS subscriptions, 4 Hz; PNG inspected for labels, font size and clipping |
| Viewer exit | Recording scan count continued to increase |
| Shutdown | All launch nodes exited cleanly; metadata closed true, bag metadata exists |

Final full telemetry integration artifacts:
/tmp/field-integration-6emi818l/result.json, viewer.png, data/, recomputed/.
Standalone logger artifacts:
/tmp/field-standalone-o_2i2qn_/20261009T132652Z_d71cd178/.
These raw synthetic bags and PNGs are local test artifacts, not boat measurements
or repository source. The test script recreates them with an isolated domain.

## Comparable synthetic identification results

Independent 90 s deterministic training/holdout input sequences, 0.05 s sampling.
No noise, waves or current. The predictor assumes current commands are held;
the synthetic actual commands continue changing. Baseline and identified model
use exactly the same prediction origins, valid segments and scored targets.

| Horizon | Scored samples | Baseline position RMSE m | Identified model position RMSE m | Baseline heading RMSE rad | Model heading RMSE rad |
| --- | ---: | ---: | ---: | ---: | ---: |
| 0.5 s | 344 | 0.022193 | 0.004868 | 0.015630 | 0.003939 |
| 1.0 s | 342 | 0.084185 | 0.025253 | 0.058633 | 0.021732 |
| 2.0 s | 338 | 0.328867 | 0.154850 | 0.215799 | 0.125373 |

Known synthetic surge/yaw coefficients were recovered within 1e-8. A separate
test recovered independent 0.2 s throttle and 0.3 s steering delays. This verifies
the software calculation and held-out scoring, not real-vessel model accuracy.
No synthetic coefficients are shipped as a default boat model.

## Compatibility evidence and unverified items

The rosbag metadata adapter accounts for serialized YAML QoS in Humble and
native QoS objects in newer ROS releases; source interfaces were checked against
[Humble rosbag2 bindings](https://raw.githubusercontent.com/ros2/rosbag2/humble/rosbag2_py/src/rosbag2_py/_storage.cpp)
and [Jazzy rosbag2 bindings](https://raw.githubusercontent.com/ros2/rosbag2/jazzy/rosbag2_py/src/rosbag2_py/_storage.cpp).
Only Lyrical was actually built and executed here. The resilient SQLite preset
and immediate writer cache configuration were exercised locally. Hard power
loss, storage exhaustion and damaged SQLite recovery were not tested.

Not verified: real /scan geometry and fixed landmarks; IMU mounting, acquisition
delay, sign or bias; installed boat ROS/Python; servo neutral and port direction;
thruster neutral/output/reverse; EMO operation; MCU timeout; RC priority;
radio/Wi-Fi dropouts; long-duration storage throughput. A viewer process exit
was tested, not a physical network failure. DDS numeric telemetry is BEST_EFFORT
to avoid reliable remote acknowledgement waits in local recording callbacks.

rf2o_laser_odometry and robot_localization are not installed in this development
ROS prefix. gps_free_odometry.launch.py and ekf_no_gps.yaml passed syntax/config
checks but their external nodes were not run. The optional EKF requires qualified
odometry covariance, verified IMU mounting and separate on-boat validation.
It is disabled by default. No automatic drive is available.

Next ticket: source the actual boat environment, verify sensor/command topic
receipts in PASSIVE, record a 30–60 s stationary experiment, check frames/bias/
drift and RC command observability, then collect independent low-speed manual
training and holdout experiments. Do not infer model quality from these synthetic
software tests.
