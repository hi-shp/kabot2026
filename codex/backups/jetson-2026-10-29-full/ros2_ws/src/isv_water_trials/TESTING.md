# Verification status

Work date: 2026-10-29 Asia/Seoul.

- Live SSH and ROS Humble environment: observed.
- Existing workspace includes user modifications: preserved.
- Actual water trial, motor actuation, sensor mounting, calibrated thrust, hydrodynamic identification: NOT performed.
- Velodyne exact model, IP, live packet/pointcloud/scan reception: NOT confirmed.
- Existing user MPC implementation: NOT located in inspected paths; no claim of live MPC integration.
- Added recorder is passive by default. Trial is dry-run by default. Offline MPC never calls actuator services.
- `colcon build --packages-select isv_water_trials mechaship_bringup`: PASS on Jetson. Package manifests parsed as `ament_python` / `ament_cmake`. `ros2 pkg prefix isv_water_trials` resolves the installed package.
- `python3 -m unittest discover -s tests -p 'test_*.py' -v`: 23 tests PASS on Jetson (35.178 seconds): 19 numerical/configuration/default-safety tests and 4 actual ROS transport fixtures.
- Numerical tests include 30 seeded inverse least-squares recoveries and 50 sampled MPC rollout scenarios. These are synthetic numerical verification, NOT repeated physical boat trials or proof of controller stability.
- ROS fixtures use `ROS_DOMAIN_ID=217`, `ROS_LOCALHOST_ONLY=1`, and ONLY `/fixture/left`, `/fixture/right` fake actuator services. They verify raw GPS/scan/command recording, offline report generation, rejected acknowledgement handling, stale sensor abort with neutral requests, and forwarding/logging through the opt-in proxy. No real actuator services were called.
- Source package compile check and local 19-test numerical run: PASS.
- Main bringup `--show-args`: PASS; defaults expose Velodyne model/IP/ring and optional sensor toggles. Existing GPS/IMU defaults are preserved; they are not asserted to match the new recorder's configured topics.
- Sensor-only `velodyne_water_trial.launch.py` was run for 6 seconds in localhost domain 216: driver, transform, and laserscan processes STARTED; ended by intentional timeout (exit 124). UDP packet polling timed out. **Actual packet/pointcloud/scan receipt: NOT verified / no data observed.** No actuator nodes were launched by this check.
- Default-domain `preflight --seconds 3`: expected NOT_READY, exit 2. No publishers observed on `/smc_2000/fix`, `/vectornav/raw/common`, `/scan`, `/velodyne_points`; GPS/IMU/E-stop data missing. Frame configuration remains unconfirmed.
- Test/build logs remain locally on Jetson at `/home/isv/water_trial_validation/20261029/`. Real data and device logs are not committed.

## Practical limits

The physical boat has not been driven by these tools. Hardware E-stop wiring, MCU timeout behavior, motor direction/neutral/deadband, reverse behavior, live power state, antenna/IMU/LiDAR extrinsics, heading offset, clock alignment, propeller thrust calibration, currents/waves and actual mass loading all require field confirmation. The trial runner can only attempt a software neutral command while communication and the process remain functional.

The optional logging proxy changes the request path when explicitly remapped; timeout/failure is reported, but it cannot guarantee that a delayed hardware request was not applied. It is not a hardware watchdog. The offline MPC ignores sway/coupled dynamics and never enables deployment automatically. A distinct held-out data session is required for its replay validator.
