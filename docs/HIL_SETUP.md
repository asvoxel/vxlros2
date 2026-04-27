# Hardware-in-the-Loop (HiL) CI Setup

The `.github/workflows/hil.yml` workflow runs the integration test suite
against a real VxlSense camera. It targets a self-hosted runner labeled
`vxl-hardware`. This document describes the one-time setup.

## Prerequisites for the runner machine

- Ubuntu 22.04 (Humble) or 24.04 (Jazzy)
- USB 3.0+ port with a VxlSense camera permanently connected
- Network access to `github.com` for repo checkout and Actions communication
- Stable power (the test takes ~20 min; the camera should not drop out mid-run)

## 1. Install ROS 2

```bash
# Humble (22.04)
sudo apt update && sudo apt install -y \
  ros-humble-desktop \
  ros-humble-image-transport \
  ros-humble-camera-info-manager \
  ros-humble-cv-bridge \
  ros-humble-tf2-ros \
  ros-humble-diagnostic-updater \
  python3-colcon-common-extensions \
  python3-launch-testing-ament-cmake \
  build-essential cmake git pkg-config libusb-1.0-0-dev
```

For Jazzy on 24.04, replace `humble` → `jazzy`.

## 2. Install the VxlSDK

The HiL workflow checks `/opt/vxlsdk/current/include/vxl.hpp` first. Pre-install
the SDK there to skip the source build on every run:

```bash
sudo mkdir -p /opt/vxlsdk
git clone https://github.com/asvoxel/vxlsdk /tmp/vxlsdk-src
cd /tmp/vxlsdk-src
./scripts/release-linux.sh
sudo cp -r sdk/asvxl-* /opt/vxlsdk/
sudo ln -sfn /opt/vxlsdk/asvxl-X.Y.Z /opt/vxlsdk/current  # match the built version
```

If `/opt/vxlsdk/current` is missing, the workflow falls back to building from
source on each run (slower but works).

## 3. Install udev rules (camera permissions)

```bash
sudo cp src/vxlsdk-src/scripts/99-vxl-cameras.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

The user that runs the GitHub Actions agent must be in the `plugdev` group:

```bash
sudo usermod -a -G plugdev $USER
# Re-login required for group change to apply
```

Verify:

```bash
lsusb | grep -i -E "vxl|asvoxel"   # should list the camera
ls -l /dev/bus/usb/<bus>/<dev>      # rw permissions for plugdev
```

## 4. Register the GitHub Actions runner

In the repo settings → Actions → Runners → "New self-hosted runner". Follow
the linker steps. **Add the `vxl-hardware` label** during configuration:

```bash
./config.sh \
  --url https://github.com/asvoxel/vxlros2 \
  --token <your-token> \
  --labels self-hosted,vxl-hardware,Linux,X64
```

Install as a service so it survives reboots:

```bash
sudo ./svc.sh install
sudo ./svc.sh start
```

## 5. Trigger the workflow

```bash
# Auto-runs on every push to main. Manual trigger:
gh workflow run hil.yml -f output_mode=rgbd
gh workflow run hil.yml -f output_mode=ir
```

Test results upload as the `hil-logs-<run_id>` artifact on every run
(success or failure).

## Maintenance

- **SDK updates**: bump `/opt/vxlsdk/current` symlink to point at the new
  versioned dir. The workflow picks it up automatically on the next run.
- **Camera swap**: re-run `udevadm trigger` if device permissions change.
- **Runner OS upgrade**: the workflow currently targets Ubuntu 22.04 (Humble);
  upgrading to 24.04 means switching to Jazzy in the workflow's `apt install`
  step too.

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| `No VxlSense USB device detected` | Camera disconnected or kernel reset; replug |
| `Permission denied /dev/bus/usb/...` | udev rule not applied or user not in `plugdev` |
| Test times out at "Waiting for RGBD topic" | SDK can't open device; check `dmesg` for USB errors |
| `VxlSense library not found` | `VXL_SDK_DIR` env var not set or path stale |
