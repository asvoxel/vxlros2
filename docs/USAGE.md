# VxlROS2 Usage Guide

Currently supports **VXL435 / VXL615** (VXL605 pending PID assignment). Two
node variants ship:
- `vxl_camera_node` — plain node, works the moment it starts
- `vxl_camera_lifecycle_node` — managed-lifecycle node with USB hotplug
  auto-recovery (**recommended for production**)

---

## 1. Quick start

```bash
# After building vxl_camera_msgs + vxl_camera + vxl_description
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch with per-product defaults
ros2 launch vxl_camera vxl435.launch.py
# or
ros2 launch vxl_camera vxl615.launch.py
```

By default the node publishes to `/vxl_camera/rgbd` (a synchronized RGBD
composite message).

```bash
# What's being published
ros2 topic list | grep vxl

# Actual frame rate
ros2 topic hz /vxl_camera/rgbd

# Peek at a single frame
ros2 topic echo /vxl_camera/rgbd --once --field rgb.encoding
```

---

## 2. Output modes

Controlled by the `output_mode` parameter, set at launch:

```bash
ros2 launch vxl_camera vxl615.launch.py --ros-args -p output_mode:=rgb+depth
```

| Mode | Default | Topics published | Use case |
|---|---|---|---|
| `rgbd` | ✅ | `~/rgbd` (sync composite) | Downstream consumers that need RGB + depth together (SLAM, point cloud fusion) |
| `rgb+depth` | | `~/color/image_raw` + `~/color/camera_info`<br>`~/depth/image_raw` + `~/depth/camera_info` | Standard ROS2 camera topics — compatible with image_view etc. |
| `ir` | | `~/ir/image_raw` + `~/ir/camera_info` | Debugging IR projector |
| `depth_only` | | Just `~/depth/...` | When you don't need color |
| `color_only` | | Just `~/color/...` | Use as a regular USB camera |
| `all` | | Everything | Full debug |

Each stream also publishes metadata:
- `~/color/metadata`, `~/depth/metadata`, `~/ir/metadata` —
  `vxl_camera_msgs/Metadata` with `timestamp_us` / `sequence` /
  `exposure_us` / `gain`

Extrinsics (latched via TRANSIENT_LOCAL QoS):
- `~/extrinsics/depth_to_color`

---

## 3. Parameters

### Cold parameters (set at launch, can't be changed at runtime)

| Parameter | Default | Description |
|---|---|---|
| `device_serial` | `""` | Serial number; empty = auto-pick first |
| `output_mode` | `"rgbd"` | See table above |
| `color.{width,height,fps}` | 1280×720@30 | Color stream config |
| `depth.{width,height,fps}` | 640×480@30 | Depth stream config |
| `ir.{width,height,fps}` | 640×480@30 | IR stream config |
| `point_cloud.enabled` | `false` | Enable PointCloud2 output |
| `sync_mode` | `"strict"` | `strict` / `approximate` / `none` |
| `tf_prefix` | `""` | Multi-camera isolation, e.g. `cam1_` |
| `publish_tf` | `true` | Publish static TF |

### Hot parameters (`ros2 param set` takes effect immediately)

#### Sensor options (auto-discovered from device capability)
- `color.exposure` / `color.gain` / `color.auto_exposure`
- `color.white_balance` / `color.auto_white_balance`
- `color.brightness` / `contrast` / `saturation` / `sharpness` / `gamma` / `hue`
- `depth.exposure` / `depth.gain` / `depth.auto_exposure`
- `depth.min_distance` / `depth.max_distance` / `depth.ir_enable`

**Mutual-exclusion guard**: when `auto_exposure=1`, setting `exposure` or
`gain` is rejected with a clear reason. To switch to manual values, either
disable auto first or change both atomically:

```bash
# ❌ Rejected (auto is still on)
ros2 param set /vxl_camera color.exposure 5000

# ✅ Atomic batch: turn auto off + set manual
ros2 service call /vxl_camera/set_parameters rcl_interfaces/srv/SetParameters \
  "{parameters: [{name: color.auto_exposure, value: {type: 2, integer_value: 0}},
                 {name: color.exposure, value: {type: 2, integer_value: 5000}}]}"
```

#### Depth-to-color alignment
- `align_depth.enabled` (bool) — enables an extra pair of topics
  `~/aligned_depth_to_color/{image_raw,camera_info}`. Depth is reprojected
  into the color view; camera_info uses the color intrinsics.
- `align_depth.scale` (double) — raw → mm conversion factor. **VXL435 = 1.0**
  (vxl435.launch.py default), **VXL615 = 8.0** (vxl615.launch.py default),
  VXL605 = 16.0.

```bash
ros2 param set /vxl_camera align_depth.enabled true
```

#### Host-side depth filters (available on all SKUs, costs CPU)
Apply order: decimation → threshold → spatial → temporal → hole_filling

| Parameter | Default | Description |
|---|---|---|
| `filters.decimation.{enabled,scale}` | off, 2 | Subsample 2/4/8 |
| `filters.threshold.{enabled,min_mm,max_mm}` | off, 100, 5000 | Range clip |
| `filters.spatial.{enabled,magnitude,alpha,delta}` | off | Edge-preserving smoothing |
| `filters.temporal.{enabled,alpha,delta}` | off | Multi-frame temporal smoothing |
| `filters.hole_filling.{enabled,mode}` | off, 0 | Hole filling; mode=0/1/2 = nearest/farthest/fill_left |

```bash
# Recommended for indoor navigation: enable hole_filling
ros2 param set /vxl_camera filters.hole_filling.enabled true
```

#### Device-side filters (VXL6X5 only, zero CPU; silently ignored on VXL435)

| Parameter | Default (vxl615) | Description |
|---|---|---|
| `filters.device.denoise.{enabled,level}` | true, 2 | Device denoise, level 1-3 |
| `filters.device.median.{enabled,kernel_size}` | true, 3 | Median filter, kernel 3 or 5 |
| `filters.device.outlier_removal.enabled` | true | Outlier removal |

VXL615 recommended setup: device filters all on + host filters off (avoid
double-processing). VXL435 has only host-side available.

#### Point cloud (when enabled)
- `point_cloud.color` (bool) — RGB colorization
- `point_cloud.{min_z,max_z}` (m) — range filter (0 = no limit)
- `point_cloud.decimation` (int) — pixel skip
- `point_cloud.organized` (bool) — true = W×H grid with NaN; false = dense (valid points only)

#### Other
- `auto_recover_on_reconnect` (bool, lifecycle node only) — auto re-activate after USB reconnect

---

## 4. Services

| Service | Type | Purpose |
|---|---|---|
| `~/get_device_info` | `vxl_camera_msgs/GetDeviceInfo` | Device name / S/N / firmware |
| `~/get_option` | `vxl_camera_msgs/GetInt32` | Read by `vxl_option_t` ID |
| `~/set_option` | `vxl_camera_msgs/SetInt32` | Write by ID (bypasses ROS parameter dependency check) |
| `~/hw_reset` | `std_srvs/Trigger` | Hardware reset |

```bash
ros2 service call /vxl_camera/get_device_info vxl_camera_msgs/srv/GetDeviceInfo
ros2 service call /vxl_camera/hw_reset std_srvs/srv/Trigger
```

---

## 5. Lifecycle operations

`vxl_camera_lifecycle_node` auto-runs `configure → activate` on launch by
default. To take manual control:

```bash
ros2 launch vxl_camera vxl_camera_lifecycle.launch.py auto_activate:=false
# Then:
ros2 lifecycle list /vxl_camera
ros2 lifecycle set /vxl_camera configure
ros2 lifecycle set /vxl_camera activate
ros2 lifecycle set /vxl_camera deactivate    # pause streaming, keep node alive
ros2 lifecycle set /vxl_camera activate      # resume
ros2 lifecycle set /vxl_camera cleanup       # close device, back to UNCONFIGURED
```

When the USB cable is unplugged the node automatically transitions to
INACTIVE; reconnecting the device triggers automatic re-activation
(governed by `auto_recover_on_reconnect`).

State topic:
- `~/connection_state` (`std_msgs/String`) — values like
  `ACTIVE/CONNECTED`, `INACTIVE/DISCONNECTED`

---

## 6. Multi-camera

```bash
# Multi-process (maximum isolation)
ros2 launch vxl_camera multi_camera.launch.py \
  serial_1:=VXL435_001 serial_2:=VXL615_002

# Single-process ComposableNodeContainer (intra-process zero-copy)
ros2 launch vxl_camera multi_camera_composable.launch.py \
  serial_1:=VXL435_001 serial_2:=VXL615_002
```

Each camera lives in its own namespace (`/camera_1/...`, `/camera_2/...`)
with TF prefix (`cam1_*` / `cam2_*`).

To mix SKUs, copy `config/vxl435.yaml` / `config/vxl615.yaml` into your own
workspace, edit, then point each launch's `parameters: [...]` at your file.

---

## 7. Diagnostics

`/diagnostics` (1 Hz) carries:
- Per-stream measured fps + expected fps (low fps triggers WARN)
- Device name / serial / firmware / connection state
- output_mode

```bash
ros2 topic echo /diagnostics --once
# Or graphical:
rqt_robot_monitor
```

---

## 8. Troubleshooting

### `No ASVXL camera found`
Device not enumerated. Check:
```bash
lsusb | grep -iE 'vxl|asvxl|0567'
ls -l /dev/bus/usb/  # permissions
```
If the device is listed but inaccessible, install the udev rule:
`scripts/99-vxl-cameras.rules` (lives in the vxlsdk repo).

### Log says `Filter chain: off` but I enabled hole_filling
Verify the actual parameter value:
`ros2 param get /vxl_camera filters.hole_filling.enabled`. If it's still
false, the YAML in your launch file probably overrides your set call —
YAML files are loaded once at launch.

### auto_exposure rejects manual values
See section 3, "Mutual-exclusion guard". Set both atomically or disable
auto first.

### Device-side filters have no effect on VXL615
Make sure you launched with `vxl615.launch.py` (the VXL435 launch leaves
device filters off). Confirm via:
```bash
ros2 param get /vxl_camera filters.device.denoise.enabled
```

### Aligned depth looks wrong
Check `align_depth.scale` matches the device: VXL435=1.0, VXL615=8.0,
VXL605=16.0. The product-specific launches set this for you; the generic
launches don't.

### How to verify the environment end-to-end
```bash
./scripts/smoke_test_real.sh           # default: rgbd, 15s
./scripts/smoke_test_real.sh --align   # also exercise depth-to-color
./scripts/smoke_test_real.sh --duration 60
```

---

## 9. Build / runtime environments

| Platform | Path |
|---|---|
| **Linux native** | Ubuntu 22.04 + ROS Humble via apt install (production-recommended) |
| **macOS dev** | Lima VM (Ubuntu 22.04 ARM64) — see `docs/DEV_VM.md` |
| **macOS native** | Not a ROS 2 Tier 1 platform — **don't go this route** |

Build:
```bash
# Assumes vxlsdk in ../vxlsdk has been built
colcon build --packages-select vxl_camera_msgs vxl_camera vxl_description
```

---

## TL;DR

**Default to `vxl<sku>.launch.py`, change anything you need with
`ros2 param set`, and use the `vxl_camera_lifecycle_node` launches in
production for hotplug recovery.** All current behavior is covered by
102 unit tests, but the project has **zero real-device hours yet** — run
`smoke_test_real.sh` against hardware before deploying.
