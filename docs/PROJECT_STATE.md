# Project state — field calibration

## Objective

Prepare hi-shp/kabot2026 field-calibration for immediate on-boat PASSIVE recording,
LiDAR/IMU motion monitoring and numeric laptop visualization. Autonomous mission
algorithms and the simulator are outside this task.

## Completed milestone (2026-10-09)

Added standalone ament_python field_calibration, independent launch/YAML profiles,
raw rosbag2/CSV logging, state validity gates, manual test annotations, least-squares
identification, labelled uncalibrated baseline and identified-model prediction,
time-aligned future error scoring, laptop plots and isolated ROS integration tests.
Added FIELD_TEST_GUIDE.md, interface audit and software validation record.
Changed no existing isv/mechaship mission, teleop, actuator or SLAM/EKF files.

Decisions: default PASSIVE with no actuator publisher/service; automatic drive
unavailable because MCU expiry/RC priority are not established. Raw RF2O zeros
are unknown covariance, rejected by default; explicit indoor opt-in remains
limited quality, outdoor does not opt in. Derive interval body surge/sway from
odometry positions instead of trusting hardcoded zero RF2O sway. Raw IMU radians
and CCW yaw are separate from competition clockwise-degree conventions. Numeric
telemetry is BEST_EFFORT; raw recording stays onboard. Store model snapshots and
full startup parameters. GPS-free EKF is optional and off by default.

Validation: ROS2 Lyrical/Python 3.14 build/run/launch pass, 22 unit tests pass.
Real DDS synthetic integration passes loss/skew/covariance gates, PASSIVE publisher
checks, bag recording/replay, /clock recomputation, CSV cleanup, static TF latching,
viewer rendering and continued recording after viewer exit. 1433 raw messages
read in the final integration fixture; no real-vessel accuracy claim. Details
and comparable synthetic holdout metrics are in docs/FIELD_VALIDATION.md.

Known issues: boat ROS/Python and sensor/hardware conventions unverified; default
unknown-covariance RF2O produces INVALID while logging continues. Scan features
and consistency gates cannot detect all drift. Direct monitor uses RF2O heading
and aligned IMU yaw rate rather than an independently validated fusion estimator.
Optional RF2O/EKF external-node launch has not been run in the local ROS prefix.
No verified MCU watchdog, RC arbitration or automatic motor control.

Exact next action: follow FIELD_TEST_GUIDE.md to install only the new package as
an overlay, confirm actual topic/frame/command receipts, record stationary for
30–60 s with propulsion safely off, and inspect bias/drift before any low-speed
manual training/holdout collection. Code delivery uses the existing remote
field-calibration branch; no changes to main or the separate /home/soonhong/kaboat
simulation checkout.
