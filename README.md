# VxlROS2

ROS2 driver for [VxlSense](https://github.com/asvoxel/VxlSense) depth cameras.

> Chinese documentation: [docs/README_zh.md](docs/README_zh.md)

## Features

- **RGBD output** — Synchronized RGB + Depth in a single topic (default)
- **RGB+Depth output** — Separate color and depth topics with camera_info
- **Point cloud** — Async worker pipeline with min/max-z, decimation, organized/dense modes
- **Per-stream metadata** — `~/<stream>/metadata` with timestamp / sequence / exposure / gain
- **Dynamic parameters** — Runtime exposure/gain/white-balance/etc. via `ros2 param set`
- **Lifecycle node** — `VxlCameraLifecycleNode` with USB hotplug auto-recovery
- **Diagnostics** — `/diagnostics` reports per-stream fps, drops, connection state
- **Multi-camera** — Per-process or single-container (intra-process zero-copy) launches
- **ROS2 services** — Device info, option get/set, hardware reset
- **Component node** — Both variants registered as `rclcpp_components` plugins

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

**Option A — Build from sibling `vxlsdk` source (recommended for development):**

```bash
# Layout: asVoxel/{vxlros2, vxlsdk}/  (vxlsdk checked out next to vxlros2)
cd ../vxlsdk
./scripts/release-linux.sh    # or release-macos.sh
# Output: vxlsdk/sdk/current/ — auto-detected by vxlros2 CMake
```

**Option B — Extract a release tarball:**

```bash
cd VxlROS2/vxlsdk/
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

cd ~/vxlros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select vxl_camera_msgs && source install/setup.bash
colcon build --packages-select vxl_camera vxl_description && source install/setup.bash
```

**Custom SDK path** (optional):

```bash
colcon build --cmake-args -DVXL_SDK_DIR=/opt/vxlsense/sdk
```

SDK path priority: CMake arg > `VXL_SDK_DIR` env var > sibling `../vxlsdk/sdk/current/` > in-repo `vxlsdk/`

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

### Lifecycle Mode (auto USB hotplug recovery)

```bash
# Auto-activates by default; survives USB unplug/replug
ros2 launch vxl_camera vxl_camera_lifecycle.launch.py

# Hand control to an external lifecycle manager (e.g., nav2_lifecycle_manager)
ros2 launch vxl_camera vxl_camera_lifecycle.launch.py auto_activate:=false
```

Standard lifecycle services are exposed:
```bash
ros2 lifecycle set /vxl_camera deactivate
ros2 lifecycle set /vxl_camera activate
ros2 service call /vxl_camera/get_state lifecycle_msgs/srv/GetState
```

### Multi-Camera

```bash
# Per-process (max isolation, default)
ros2 launch vxl_camera multi_camera.launch.py \
  serial_1:=VXL435_001 serial_2:=VXL435_002

# Single process via ComposableNodeContainer (intra-process zero-copy)
ros2 launch vxl_camera multi_camera_composable.launch.py \
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

### Cold (require restart)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `output_mode` | `"rgbd"` | Output mode (see above) |
| `device_serial` | `""` | Device serial number (empty = auto) |
| `color.width` / `.height` / `.fps` | 1280×720@30 | Color stream config |
| `depth.width` / `.height` / `.fps` | 640×480@30 | Depth stream config |
| `ir.width` / `.height` / `.fps` | 640×480@30 | IR stream config |
| `point_cloud.enabled` | `false` | Enable PointCloud2 generation |
| `sync_mode` | `"strict"` | Frame sync: strict / approximate / none |
| `tf_prefix` | `""` | TF frame prefix |
| `publish_tf` | `true` | Publish static TF frames |

### Hot (settable at runtime via `ros2 param set`)

| Parameter | Description |
|-----------|-------------|
| `color.exposure` / `.gain` / `.auto_exposure` | Color sensor exposure controls |
| `color.brightness` / `.contrast` / `.saturation` / `.sharpness` | Color image |
| `color.white_balance` / `.auto_white_balance` / `.gamma` / `.hue` | Color WB/tone |
| `depth.exposure` / `.gain` / `.auto_exposure` | Depth sensor exposure |
| `depth.min_distance` / `.max_distance` | Depth working range (mm) |
| `depth.ir_enable` | Toggle IR projector |
| `point_cloud.color` | Attach RGB to point cloud |
| `point_cloud.min_z` / `.max_z` | Range filter (meters; 0 = disabled) |
| `point_cloud.decimation` | Pixel skip (1 = every pixel) |
| `point_cloud.organized` | true = preserve grid with NaN; false = dense |
| `auto_recover_on_reconnect` | (lifecycle) re-activate after USB reconnect |

Hot params are auto-discovered from device capability — only those the connected
device supports are declared. `ros2 param describe <name>` shows the SDK range.

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
├── vxlsdk/                # SDK placeholder (fallback for release tarball)
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
