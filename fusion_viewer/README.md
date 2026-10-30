# fusion_viewer (ROS 2 Humble, ament_python)

Small viewer that overlays YOLO detections on a camera image and (optionally) projects Velodyne points when calibration is provided.

## Quick start

```bash
cd ~/ros2_ws/src
# unzip this folder here so it becomes ~/ros2_ws/src/fusion_viewer
colcon build --symlink-install --packages-select fusion_viewer
source ~/ros2_ws/install/setup.bash

# Without projection (no calibration)
ros2 launch fusion_viewer fusion_viewer.launch.py \  image_topic:=/camera1/image_raw \  detections_topic:=/yolo/detections \  cloud_topic:=/velodyne_points \  enable_projection:=false

# With projection (replace calib file content with your real intrinsics/extrinsics)
ros2 launch fusion_viewer fusion_viewer.launch.py \  image_topic:=/camera1/image_raw \  detections_topic:=/yolo/detections \  cloud_topic:=/velodyne_points \  calib_file:=calib_example.yaml \  enable_projection:=true
```
