#!/bin/bash
# Install ROS 2 Humble + dev tools inside the Lima VM.
# Run from inside the VM (limactl shell ros-humble).
#
# Idempotent — re-running is safe.

set -eo pipefail
# NB: avoid `set -u` — ROS 2 setup.bash references unbound vars (AMENT_TRACE_*).

# Strip any host-injected proxy env (Lima may forward Mac proxy that doesn't
# listen on the bridge IP); apt and curl can reach mirrors directly from the VM.
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY no_proxy NO_PROXY

# Colors
g() { printf '\033[0;32m✓\033[0m %s\n' "$*"; }
i() { printf '\033[0;34m→\033[0m %s\n' "$*"; }
w() { printf '\033[0;33m⚠\033[0m %s\n' "$*"; }

# ─── Pre-flight ─────────────────────────────────────────────────────────────
. /etc/os-release
[ "$VERSION_ID" = "22.04" ] || { echo "This script targets Ubuntu 22.04, found $VERSION_ID"; exit 1; }
[ "$(dpkg --print-architecture)" = "arm64" ] || w "Non-ARM64 detected; expected arm64"

# ─── Set locale (ROS2 requires UTF-8) ───────────────────────────────────────
i "Setting locale to en_US.UTF-8"
sudo apt-get update -qq
sudo apt-get install -y -qq locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
g "Locale set"

# ─── Add ROS 2 apt repo ─────────────────────────────────────────────────────
i "Adding ROS 2 apt repository"
sudo apt-get install -y -qq software-properties-common curl gnupg lsb-release
sudo add-apt-repository -y universe

# Idempotent key install
if [ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]; then
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
fi

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt-get update -qq
g "ROS 2 repo added"

# ─── Install ROS 2 Humble + dev tools + driver-relevant packages ────────────
i "Installing ros-humble-desktop + dev tools (this is the long step, ~5 min)"
sudo apt-get install -y \
  ros-humble-desktop \
  ros-humble-image-transport \
  ros-humble-image-transport-plugins \
  ros-humble-camera-info-manager \
  ros-humble-cv-bridge \
  ros-humble-tf2-ros \
  ros-humble-diagnostic-updater \
  ros-humble-launch-testing-ament-cmake \
  ros-humble-rmw-cyclonedds-cpp \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-pip \
  build-essential \
  cmake \
  git \
  pkg-config \
  libusb-1.0-0-dev \
  gdb
g "ROS 2 Humble + tools installed"

# ─── rosdep init/update (idempotent) ────────────────────────────────────────
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  i "Initializing rosdep"
  sudo rosdep init
fi
i "Updating rosdep cache (non-fatal — fetches from GitHub which can be slow/blocked)"
rosdep update --rosdistro=humble || w "rosdep update failed; not blocking. Re-run later: rosdep update"

# ─── Append source line to ~/.bashrc (idempotent) ───────────────────────────
if ! grep -q "source /opt/ros/humble/setup.bash" "$HOME/.bashrc"; then
  echo "" >> "$HOME/.bashrc"
  echo "# ROS 2 Humble" >> "$HOME/.bashrc"
  echo "source /opt/ros/humble/setup.bash" >> "$HOME/.bashrc"
  echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> "$HOME/.bashrc"
  g "Added ROS source to ~/.bashrc"
fi

# ─── Verify ─────────────────────────────────────────────────────────────────
i "Verifying installation"
# setup.bash uses unbound vars internally; sandbox via subshell.
( set +u; source /opt/ros/humble/setup.bash; ros2 --help > /dev/null )
g "ros2 CLI works"
( set +u; source /opt/ros/humble/setup.bash; echo "    ROS_DISTRO=$ROS_DISTRO" )
echo "    rclpy: $(python3 -c 'import rclpy; print(rclpy.__file__)')"

# ─── Optional: pre-build vxl_camera if SDK is reachable ─────────────────────
if [ -d "$HOME/Workspace/asVoxel/vxlsdk/sdk/current/include" ]; then
  g "VxlSDK detected at ~/Workspace/asVoxel/vxlsdk/sdk/current"
else
  w "VxlSDK not found — build vxl_camera will fail until you run:"
  echo "    cd ~/Workspace/asVoxel/vxlsdk && ./scripts/release-linux.sh"
fi

echo
g "All done. Next steps:"
echo "    source /opt/ros/humble/setup.bash         # or open a new shell"
echo "    cd ~/Workspace/asVoxel/vxlros2"
echo "    colcon build --packages-select vxl_camera_msgs vxl_camera"
echo "    colcon test --packages-select vxl_camera"
