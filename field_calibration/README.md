# Field Calibration ROS2 Package

Independent ament_python package. Default launch runs PASSIVE recording, validity
monitoring, manual-test annotation and prediction comparison. No node in this
package publishes an actuator command or calls an actuator service. Automatic
drive is disabled because the repository does not establish a hardware command
watchdog, RC arbitration or disconnect-safe motor stop.

See [FIELD_TEST_GUIDE.md](../FIELD_TEST_GUIDE.md) for installation and field steps,
and [interface audit](../docs/FIELD_INTERFACE_AUDIT.md) for source-confirmed topics,
frames and hardware unknowns. Existing mission, teleop and driver files are unchanged.

## Installation into an existing workspace

Source the actual ROS and boat workspace underlays first. Do not put the entire
repository under an existing workspace's src tree.

```bash
git clone -b field-calibration --single-branch https://github.com/hi-shp/kabot2026.git ~/kabot2026
export FIELD_WS=~/field_calibration_ws
mkdir -p "$FIELD_WS/src"
ln -s ~/kabot2026/field_calibration "$FIELD_WS/src/field_calibration"
cd "$FIELD_WS"
rosdep install --from-paths src/field_calibration --ignore-src -r -y
colcon build --symlink-install --packages-select field_calibration
source install/setup.bash
ros2 launch field_calibration field_test.launch.py
```

For an existing checkout use git status, fetch and switch the existing tracking
branch as documented in the guide. Preserve local field configuration outside
the repository. Only the new package is linked and built. Python must match the
ROS distribution's rclpy extension; no particular boat distribution is assumed.

## Commands

```bash
ros2 launch field_calibration field_test.launch.py config:=/absolute/path/boat.yaml profile:=indoor
ros2 launch field_calibration field_test.launch.py config:=/absolute/path/boat.yaml profile:=outdoor
ros2 run field_calibration field_logger --ros-args --params-file /absolute/path/boat.yaml
ros2 run field_calibration motion_identifier --summarize /path/to/experiment
ros2 run field_calibration motion_identifier --train /path/train1 /path/train2 \
  --validate /path/holdout --output /path/model.json --servo-neutral 90 --throttle-neutral 0
ros2 run field_calibration trajectory_monitor
```

Logger alone records raw topics, commands and CSV sensor timestamps. Derived
states and predictions require the other nodes, normally started by field_test.
The numeric viewer runs on the laptop, uses no LaserScan/image subscriptions,
updates at 4 Hz, and never affects recording or control.
Numeric telemetry uses BEST_EFFORT QoS so a disconnected laptop does not add
reliable DDS acknowledgement waits to the on-boat logger. Raw storage is local.

## Data contract (schema 1)

Each launch creates UTC timestamp + random experiment ID, metadata.json,
states.csv, commands.csv, sensors.csv, predictions.csv, errors.csv and bag/.
The rosbag stores CDR messages and receipt timestamps. Header timestamps remain
inside raw messages and sensors.csv. Command topics have no header: timestamps
are on-boat receipt time, not actuator execution time. RC commands not mirrored
onto ROS cannot be recovered: they remain missing, never zero-filled.

State and prediction topics use std_msgs/msg/String containing compact JSON.
Coordinates use odom metres, REP-103 body x forward/y port, heading CCW radians,
surge/sway m/s, yaw rate rad/s. Servo degrees and throttle percent are commands,
not measured shaft thrust or measured rudder angles. A valid odometry delta is
rotated by interval midpoint heading to estimate body surge and sway; RF2O's
hardcoded zero sway is not used. This is an interval measurement, not a full EKF.
Frame IDs, epochs, timestamps, validity, measurement ages, covariance and
quality are explicit. INVALID x/y/heading/u/v/r are null.

| Topic | Payload |
| --- | --- |
| /field/state | Validity-gated measured state, raw IMU diagnostics |
| /field/command | Receipt stamp, kind, command value |
| /field/prediction | Initial state, held inputs, version, dense path, 0.5/1/2 s endpoints |
| /field/prediction_error | Interpolated same-frame same-epoch future truth and numerical errors |
| /field/test_profile | PASSIVE test label and manual operator instructions |
| /field/recording_status | Experiment ID/path, counts, missing topics, writer status |

Use this contract as the future simulator adapter boundary. No simulator
coefficients, GPS or competition navigation changes are introduced here.

## Validity and model limits

RF2O in this repository publishes unknown (zero) covariance. The default gate
therefore reports INVALID while still recording everything. After stationary
and known-motion checks, indoor operators may explicitly allow unknown
covariance in boat.yaml; quality stays LIMITED_UNKNOWN_COVARIANCE. Outdoor
profile always rejects it. Return count, sector coverage, age, jump and IMU
agreement checks cannot prove scan matching correctness or detect all drift.

No model is supplied without measurements. The uncalibrated constant body
velocity/yaw-rate baseline is clearly labelled and scored at all horizons.
Least squares estimates dissipative surge and yaw response, independent delays
(0 to 1 s in 0.1 s increments), and separate positive/negative steering gains.
Training experiments must differ from holdout experiments. Poor excitation,
rank deficiency, nondissipative fits or insufficient horizon validation produce
UNCALIBRATED and exit status 2. The JSON includes data hashes, units, bounds,
derivative validation and trajectory errors compared with the baseline.
CALIBRATED means an empirical fit with held-out measurements, not verified
operational accuracy. Inspect holdout error metrics before using it.

Future commands and sway are held constant. Outside training command bounds or
with missing/stale inputs the predictor falls back to the labelled baseline.
Errors compare past predictions with time-interpolated actual observations;
gaps, resets, frame changes and excessive sample intervals are not scored.

## Development tests

```bash
cd ~/kabot2026
python3 -m compileall -q field_calibration
PYTHONPATH=field_calibration python3 -m pytest field_calibration/tests -q
# Source matching ROS first; integration launches only mock topics in an isolated domain.
python3 field_calibration/tests/ros_integration.py
```

The mock source publishes commands only under /field/mock/*. Run it only in a
development ROS domain. It is a deterministic synthetic fixture, not evidence
of real-vessel accuracy. See docs/FIELD_VALIDATION.md for the actual tested
environment, results and remaining hardware checks.
