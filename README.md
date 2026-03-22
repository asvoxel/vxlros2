# VxlROS2

ROS2 driver for [VxlSense](https://github.com/asvoxel/VxlSense) depth cameras.

> Chinese documentation: [docs/README_zh.md](docs/README_zh.md)

## Features

- **RGBD output** — Synchronized RGB + Depth in a single topic (default)
- **RGB+Depth output** — Separate color and depth topics with camera_info
- **Point cloud** — Optional PointCloud2 generation from depth
- **Multi-camera** — Multiple cameras with TF prefix isolation
- **ROS2 services** — Device info, option get/set, hardware reset
- **Component node** — Supports `rclcpp_components` for intra-process zero-copy

## Prerequisites

- ROS2 Humble or later
- VxlSense SDK ([releases](https://github.com/asvoxel/VxlSense/releases))

### Install ROS2 Dependencies

```bash
sudo apt install \
  ros-humble-image-transport \
  ros-humble-camera-info-manager \
  ros-humble-cv-bridge \
  ros-humble-tf2-ros \
  ros-humble-xacro \
  ros-humble-robot-state-publisher
```

### Install VxlSense SDK

```bash
cd VxlROS2/vxlsense-sdk/
wget https://github.com/asvoxel/VxlSense/releases/download/vX.Y.Z/asvxl-sdk-X.Y.Z-linux-x86_64.tar.gz
tar xzf asvxl-sdk-X.Y.Z-linux-x86_64.tar.gz --strip-components=1

# Verify
ls include/vxl.hpp lib/linux/libasvxl.a
```

## Build

```bash
mkdir -p ~/vxlros2_ws/src && cd ~/vxlros2_ws/src

# Symlink packages (or copy)
ln -s /path/to/VxlROS2/vxl_camera_msgs .
ln -s /path/to/VxlROS2/vxl_camera .
ln -s /path/to/VxlROS2/vxl_description .
ln -s /path/to/VxlROS2/vxlsense-sdk ../vxlsense-sdk

cd ~/vxlros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select vxl_camera_msgs && source install/setup.bash
colcon build --packages-select vxl_camera vxl_description && source install/setup.bash
```

**Custom SDK path** (optional):

```bash
colcon build --cmake-args -DVXL_SDK_DIR=/opt/vxlsense/sdk
```

SDK path priority: CMake arg > `VXL_SDK_DIR` env var > `../vxlsense-sdk/`

## Usage

### RGBD Mode (default)

```bash
ros2 launch vxl_camera vxl_camera.launch.py
```

Published topics:

| Topic | Type | Description |
|-------|------|-------------|
| `~/rgbd` | `vxl_camera_msgs/RGBD` | Synchronized RGB + Depth + CameraInfo |
| `~/extrinsics/depth_to_color` | `vxl_camera_msgs/Extrinsics` | Depth-to-color extrinsics (latched) |

### RGB+Depth Mode

```bash
ros2 launch vxl_camera vxl_camera.launch.py output_mode:=rgb+depth
```

Published topics:

| Topic | Type |
|-------|------|
| `~/color/image_raw` | `sensor_msgs/Image` (BGR8) |
| `~/color/camera_info` | `sensor_msgs/CameraInfo` |
| `~/depth/image_raw` | `sensor_msgs/Image` (16UC1, mm) |
| `~/depth/camera_info` | `sensor_msgs/CameraInfo` |

### Debug Modes

```bash
ros2 run vxl_camera vxl_camera_node --ros-args -p output_mode:=ir
ros2 run vxl_camera vxl_camera_node --ros-args -p output_mode:=all
```

Available modes: `rgbd` | `rgb+depth` | `ir` | `depth_only` | `color_only` | `all`

### Point Cloud

Enable with parameter:

```bash
ros2 launch vxl_camera vxl_camera.launch.py \
  --ros-args -p point_cloud.enabled:=true
```

Topic: `~/depth/points` (`sensor_msgs/PointCloud2`)

### Multi-Camera

```bash
ros2 launch vxl_camera multi_camera.launch.py \
  serial_1:=VXL435_001 serial_2:=VXL435_002
```

### Services

```bash
# Device info
ros2 service call /vxl_camera/get_device_info vxl_camera_msgs/srv/GetDeviceInfo

# Get/set camera option (by option ID)
ros2 service call /vxl_camera/get_option vxl_camera_msgs/srv/GetInt32 "{option_name: '1'}"
ros2 service call /vxl_camera/set_option vxl_camera_msgs/srv/SetInt32 "{option_name: '1', value: 5000}"

# Hardware reset
ros2 service call /vxl_camera/hw_reset std_srvs/srv/Trigger
```

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `output_mode` | `"rgbd"` | Output mode (see above) |
| `device_serial` | `""` | Device serial number (empty = auto) |
| `color.width` | `1280` | Color stream width |
| `color.height` | `720` | Color stream height |
| `color.fps` | `30` | Color stream framerate |
| `depth.width` | `640` | Depth stream width |
| `depth.height` | `480` | Depth stream height |
| `depth.fps` | `30` | Depth stream framerate |
| `point_cloud.enabled` | `false` | Enable PointCloud2 |
| `point_cloud.color` | `true` | Attach RGB to point cloud |
| `sync_mode` | `"strict"` | Frame sync: strict / approximate / none |
| `tf_prefix` | `""` | TF frame prefix |
| `publish_tf` | `true` | Publish static TF frames |

## TF Frames

```
{tf_prefix}vxl_camera_link
  +-- {tf_prefix}vxl_color_optical_frame
  +-- {tf_prefix}vxl_depth_optical_frame
  +-- {tf_prefix}vxl_ir_optical_frame
```

## Package Structure

```
VxlROS2/
├── vxlsense-sdk/          # VxlSense SDK (extract release here)
├── vxl_camera/             # Main driver node
├── vxl_camera_msgs/        # Custom messages and services
├── vxl_description/        # URDF camera model
└── docs/                   # Design documents
```

## Testing

```bash
# Unit tests (no device required)
colcon test --packages-select vxl_camera
colcon test-result --all --verbose

# Integration test (device required)
ros2 launch vxl_camera test/test_integration.launch.py
```

## License

Apache-2.0
