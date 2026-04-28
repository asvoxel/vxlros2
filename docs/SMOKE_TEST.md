# Real-Device Smoke Test

`scripts/smoke_test_real.sh` runs an end-to-end check against a physical
VXL camera. It verifies that the lifecycle node reaches ACTIVE, the expected
topics carry data at the configured rate, and `deactivate→activate` transitions
work cleanly.

Designed for two situations:
- **Manual one-off**: developer plugs a camera in to confirm a build is healthy.
- **HiL CI**: triggered by `.github/workflows/hil.yml` on a self-hosted runner
  with a permanently-attached camera (see `docs/HIL_SETUP.md`).

## Quick start (Linux host with camera)

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select vxl_camera_msgs vxl_camera
source install/setup.bash

./scripts/smoke_test_real.sh                   # default: rgbd, 15 s
./scripts/smoke_test_real.sh --mode rgb+depth  # separate color/depth topics
./scripts/smoke_test_real.sh --align           # also exercise depth-to-color
./scripts/smoke_test_real.sh --duration 30     # longer soak
```

Exits 0 on success, 1 on failure. Writes node stderr to a temp file and tails
it on failure for triage.

## What it checks

1. **Pre-flight** — `ROS_DISTRO` set, `vxl_camera` installed, `lsusb` lists a
   VxlSense device.
2. **Launch** — `vxl_camera_lifecycle_node` starts and reaches ACTIVE within 10 s.
3. **Topic rates** — for each expected topic in the chosen mode, average Hz
   over a 5-message rolling window must hit a minimum threshold:
   - `rgbd` mode: `~/rgbd`, `~/color/metadata`, `~/depth/metadata` ≥ 10 Hz
   - `rgb+depth`: `~/color/image_raw`, `~/depth/image_raw` ≥ 10 Hz
   - `ir`: `~/ir/image_raw` ≥ 10 Hz
   - `all`: color + depth + ir ≥ 10 Hz
   - `--align` adds: `~/aligned_depth_to_color/image_raw` ≥ 10 Hz
   - Always: `/diagnostics` ≥ 2 messages in the run window
4. **Lifecycle stress** — `deactivate` (transition id 4) puts node in INACTIVE,
   `activate` (id 3) puts it back in ACTIVE.

## Running against the Lima VM

The Lima VM (`scripts/ros-humble.yaml`) does not pass USB devices through by
default — Lima is best for headless build/unit-test work. For real-device
tests, use one of:

- **Parallels** (paid) — supports USB 3.0 passthrough; configure the device
  from VM Settings → Hardware → USB & Bluetooth.
- **Native Linux** — dual-boot or a separate machine with the camera attached.
- **Self-hosted GitHub Actions runner** — see `docs/HIL_SETUP.md`. The
  smoke script is what `hil.yml` runs.

## Adapting to other devices

Pre-flight checks `lsusb | grep -iE 'vxl|asvxl|0567'` (covers VXL435 PID 0017
and VXL615 VID 0567). When VXL605's PID is finalized, add it to that grep
pattern.

For VXL6X5 family, also pass `align_depth.scale:=8.0` (raw → mm conversion):

```bash
ros2 run vxl_camera vxl_camera_lifecycle_node \
  --ros-args -p output_mode:=rgbd -p align_depth.enabled:=true -p align_depth.scale:=8.0
```

## Why no automated unit-test substitute

The mock-driven launch_test (`test_lifecycle_launch.py`) covers the same
state-machine + topic-plumbing surface but with synthesized frames — it
doesn't catch issues that only show up against real USB/UVC traffic:

- Frame timing jitter from the kernel's USB stack
- Real depth-to-color sync drift over minutes
- libuvc bandwidth allocation conflicts when multiple cameras share a controller
- SDK firmware quirks specific to the connected SKU

Run the smoke script before merging significant SDK-integration changes.
