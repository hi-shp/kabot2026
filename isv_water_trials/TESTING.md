# Verification status

## Current checkpoint: 2026-10-30

The operator confirms current RC/AUTO left/right forward/reverse, mode handover,
EMO operation, bringup, continuous test sender and ESC initialization as working.
The current operating contract is [BOAT_BASELINE_2026-10-30](../docs/BOAT_BASELINE_2026-10-30.md).
Earlier NOT-performed/failed entries below are historical checks, not a claim that
the now-confirmed actuator functions remain unverified.

- This documentation/checkpoint task did not change control behavior, reflash the UNO or actuate motors.
- Read-only SSH audit: 35/36 selected source/config files match exactly. The remaining deployed bringup CMake file differs from the local deploy copy only in a comment. No ROS sensor/actuator nodes were running in domain50 during this audit; current topic receipt/rates were not remeasured.
- Local Python suite before documentation: 52 collected, 32 passed, 20 skipped because ROS or POSIX serial fixtures need their target environment.
- Immediately preceding Jetson fake-service run: 6 selected ROS fixtures passed (SIGHUP/SIGTERM persistence, external-stop ignoring, explicit initialization and automatic resumption, transient stop followed by fresh commands, Ctrl+C neutral cleanup). These fixtures do not drive motors.
- Previous current-firmware AVR compile/upload verification and C++ control/protocol checks passed; this checkpoint did not perform a new flash readback.
- Navigation/MPC performance, calibrated thrust and hydrodynamic identification remain separate from the operator-confirmed actuator operation.

## Historical checks

Work date: 2026-10-29 Asia/Seoul.

- Live SSH and ROS Humble environment: observed.
- Existing workspace includes user modifications: preserved.
- Actual water trial, motor actuation, sensor mounting, calibrated thrust, hydrodynamic identification: NOT performed.
- Velodyne live verification was completed later on the same day; see the latest checks below.
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

Latest live checks (2026-10-29, about 21:02 KST): `all_sensors.launch.py` built and running without a camera or actuator-enable nodes. GPS `/gps/fix` received at 5 Hz with valid status 0; VectorNav VN-100S-CR raw common/standard IMU at 20 Hz; Velodyne packets/scan about 9.9 Hz. A received packet product ID was 0x22 and cloud ring IDs were exactly 0 through 15. `/scan` automatically uses ring 8. Pointcloud monitor reception varied (about 7.7 Hz in one window), so lossless capture is not claimed. A 12-second passive recorder run saved actual GPS, VN, IMU and scan messages; offline report generation was checked. Battery/E-stop telemetry has no publisher yet: micro-ROS agents opened THS1, and bounded probes of USB0/THS2 received no MCU session. User reports an Arduino RC/automatic mode switch; its firmware/protocol and actuator control are not verified. Camera is intentionally disconnected. Earlier no-data entries above describe pre-connection checks, not the current GPS/IMU/LiDAR state.

Bounded rosbag sensor-regex check: stopped cleanly with SIGINT (timeout exit 124), metadata readable, 9.306 seconds / 61.3 MiB / 3099 messages, including 85 pointclouds, 86 packet scans, 85 LaserScans, 42 GPS fixes and 186 standard IMUs. Source/config mirror equality, XML parsing, Python compile checks and `git diff --check` passed. Trial surge preview printed its sequence without actuator execution. These checks do not claim every advertised VectorNav topic is a supported VN-100 measurement; its GNSS/position/body-velocity outputs are not used as navigation truth.

The physical boat has not been driven by these tools. Hardware E-stop wiring, MCU timeout behavior, motor direction/neutral/deadband, reverse behavior, live power state, antenna/IMU/LiDAR extrinsics, heading offset, clock alignment, propeller thrust calibration, currents/waves and actual mass loading all require field confirmation. The trial runner can only attempt a software neutral command while communication and the process remain functional.

The optional logging proxy changes the request path when explicitly remapped; timeout/failure is reported, but it cannot guarantee that a delayed hardware request was not applied. It is not a hardware watchdog. The offline MPC ignores sway/coupled dynamics and never enables deployment automatically. A distinct held-out data session is required for its replay validator.
