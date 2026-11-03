# Verification — 2026-11-02

Upstream reference: `https://github.com/hi-shp/kabot2026`, commit
`6797c384d4ab1ea99d9f23dc918a448d25d0c89e`.

## Completed locally

- **52 unittest cases passed** on Windows with Python 3.14, NumPy 2.5.4,
  SciPy 1.18.1 and OpenCV 5.0.0.
- Camera contours, sensor timestamp rejection, camera/scan extrinsics,
  clockwise scan ordering, mixed depth rejection, camera calibration checks.
- Occupancy grid inflation, blocked unknown space, map origin rotation,
  no corner cutting, narrow-gap rejection and obstacle detours.
- Stable marker locking, duplicate frame rejection, temporary occlusion,
  target expiry, two-stage arrival, stale/missing sensors and obstacle stop.
- Paired service driver with fake futures: preview never arms, concurrent
  paired submissions, arm ordering, partial rejection, timeout, RC handoff,
  stale telemetry, and stopping when PWM services disappear.
- ROS adapter callback tests using message/node doubles: default topics,
  camera-to-world fusion and path, ambiguous markers, invalid detections,
  SLAM session reset, backward replay clock, stop-topic echo and preview gate.
  These tests do **not** exercise DDS or actual ROS type serialization.
- Synthetic end-to-end demo completed in 169 ticks:
  `SEARCH_SHAPE -> APPROACH_SHAPE -> ENTER_REGION -> REGION_REACHED`.
  Marker estimated at approximately `(8.000, 4.987)` metres; final position
  `(6.576, 4.990)` metres is within the configured entry-goal tolerance.
- A separate controller test completed an obstacle detour while remaining
  inside the inflated free map, including `ALIGN_PATH` rotation.
- Python compilation, Python 3.10 syntax parsing, package XML and configuration
  consistency, CLI help and archive integrity checked during packaging.

## Source preservation

User-supplied `docking_geometry.py` was copied without changes. Both files have
SHA256 `7792eb443a92936eb09914fed1c438d59fdf544f0a44b95a5e66b1d4f95f3812`.
The U-shaped docking helpers are included for the next stage and are not
called by the shape navigation mission.

## Not verified here

The workspace has no ROS2 installation, connected Jetson, live camera, LiDAR,
GPS/IMU or actuators. No colcon build, DDS test, rosbag capture/export against
real bags, propulsion operation, or water trial was performed. Camera
intrinsics/extrinsics, vessel footprint relative to the LiDAR, marker layout,
speed, stopping distance and current/wind effects require field calibration.
The package defaults to preview and refuses hardware output with simulated
ROS time. No trained weights or labeled field dataset are supplied.
