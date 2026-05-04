#!/bin/bash
# Real-device smoke test for VxlCameraLifecycleNode.
#
# Prerequisites
#   - VXL435 / VXL615 plugged into a USB 3.0 port that the host (or VM via
#     passthrough) sees. Run `lsusb | grep -iE 'vxl|asvxl'` to verify.
#   - vxl_camera built locally: `colcon build --packages-select vxl_camera`
#   - ROS 2 Humble env sourced: `source /opt/ros/humble/setup.bash`
#
# Usage
#   ./scripts/smoke_test_real.sh                    # default: rgbd, 15 s
#   ./scripts/smoke_test_real.sh --mode rgb+depth   # different output mode
#   ./scripts/smoke_test_real.sh --duration 30      # longer run
#   ./scripts/smoke_test_real.sh --align            # also exercise depth-to-color
#
# Exits 0 if all checks pass, 1 otherwise. Suitable for a CI hardware runner.

set -eo pipefail

MODE=${MODE:-rgbd}
DURATION=${DURATION:-15}
ALIGN=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode) MODE="$2"; shift 2 ;;
    --duration) DURATION="$2"; shift 2 ;;
    --align) ALIGN=true; shift ;;
    -h|--help)
      sed -n '2,/^$/p' "$0" | sed 's/^# //;s/^#$//'; exit 0 ;;
    *) echo "Unknown flag: $1" >&2; exit 2 ;;
  esac
done

g() { printf '\033[0;32m✓\033[0m %s\n' "$*"; }
i() { printf '\033[0;34m→\033[0m %s\n' "$*"; }
e() { printf '\033[0;31m✗\033[0m %s\n' "$*" >&2; }

# ─── Pre-flight ─────────────────────────────────────────────────────────────
i "Pre-flight: checking environment"

if [[ -z "${ROS_DISTRO:-}" ]]; then
  e "ROS not sourced. Run: source /opt/ros/humble/setup.bash && source install/setup.bash"
  exit 1
fi
g "ROS_DISTRO=$ROS_DISTRO"

if ! ros2 pkg list 2>/dev/null | grep -q '^vxl_camera$'; then
  e "vxl_camera not found in install. Build first: colcon build --packages-select vxl_camera"
  exit 1
fi
g "vxl_camera package available"

if ! lsusb 2>/dev/null | grep -qiE 'vxl|asvxl|0567'; then
  e "No ASVXL device detected via lsusb. Check USB connection."
  echo "lsusb output:" >&2
  lsusb >&2
  exit 1
fi
g "ASVXL device detected"

# ─── Launch the node in background ───────────────────────────────────────────
i "Launching vxl_camera_lifecycle_node (output_mode=$MODE)"

LAUNCH_LOG=$(mktemp)
ALIGN_ARGS=()
if $ALIGN; then ALIGN_ARGS+=(-p align_depth.enabled:=true); fi

ros2 run vxl_camera vxl_camera_lifecycle_node \
  --ros-args \
  -p output_mode:="$MODE" \
  -p publish_tf:=true \
  "${ALIGN_ARGS[@]}" \
  > "$LAUNCH_LOG" 2>&1 &
NODE_PID=$!
trap 'kill -INT $NODE_PID 2>/dev/null; wait $NODE_PID 2>/dev/null; rm -f "$LAUNCH_LOG"' EXIT

# Wait for the node to reach ACTIVE — auto-activate path drives configure+activate.
for tries in $(seq 1 20); do
  STATE=$(ros2 service call /vxl_camera/get_state lifecycle_msgs/srv/GetState 2>/dev/null \
    | grep -oE 'label=[^,]*' | head -1 | cut -d'=' -f2 | tr -d ' "')
  if [[ "$STATE" == "active" ]]; then
    g "Node reached ACTIVE state"
    break
  fi
  sleep 0.5
done
if [[ "${STATE:-}" != "active" ]]; then
  e "Node did not reach ACTIVE within 10 s. Tail of log:"
  tail -30 "$LAUNCH_LOG" >&2
  exit 1
fi

# ─── Topic checks ───────────────────────────────────────────────────────────
i "Subscribing to expected topics for $DURATION s"

declare -A EXPECTED
case "$MODE" in
  rgbd)
    EXPECTED=( ["/vxl_camera/rgbd"]=10 ["/vxl_camera/color/metadata"]=10 ["/vxl_camera/depth/metadata"]=10 ) ;;
  rgb+depth)
    EXPECTED=( ["/vxl_camera/color/image_raw"]=10 ["/vxl_camera/depth/image_raw"]=10 ) ;;
  ir)
    EXPECTED=( ["/vxl_camera/ir/image_raw"]=10 ) ;;
  all)
    EXPECTED=( ["/vxl_camera/color/image_raw"]=10 ["/vxl_camera/depth/image_raw"]=10 ["/vxl_camera/ir/image_raw"]=10 ) ;;
  *)
    e "Unknown mode: $MODE"; exit 2 ;;
esac
if $ALIGN; then EXPECTED["/vxl_camera/aligned_depth_to_color/image_raw"]=10; fi
EXPECTED["/diagnostics"]=2  # 1Hz updater → at least 2 in $DURATION s

# Run topic hz checks in parallel.
declare -A HZ
for topic in "${!EXPECTED[@]}"; do
  (timeout "$DURATION"s ros2 topic hz --window 5 "$topic" 2>&1 \
     | grep -oE 'average rate: [0-9.]+' | tail -1 | awk '{print $3}' \
     > "/tmp/smoke_hz_$$_$(echo "$topic" | tr '/' '_')") &
done
wait

FAILED=0
for topic in "${!EXPECTED[@]}"; do
  expected_min=${EXPECTED[$topic]}
  hz_file="/tmp/smoke_hz_$$_$(echo "$topic" | tr '/' '_')"
  rate=$(cat "$hz_file" 2>/dev/null || echo "0")
  rm -f "$hz_file"
  rate_int=$(printf '%.0f' "${rate:-0}")
  if [[ "$rate_int" -ge "$expected_min" ]]; then
    g "$topic — $rate Hz (expected ≥ $expected_min)"
  else
    e "$topic — $rate Hz (expected ≥ $expected_min)  FAIL"
    FAILED=$((FAILED + 1))
  fi
done

# ─── Lifecycle stress: deactivate → activate ────────────────────────────────
i "Lifecycle stress: deactivate → activate"
ros2 service call /vxl_camera/change_state \
  lifecycle_msgs/srv/ChangeState "{transition: {id: 4}}" > /dev/null
sleep 1
STATE=$(ros2 service call /vxl_camera/get_state lifecycle_msgs/srv/GetState 2>/dev/null \
  | grep -oE 'label=[^,]*' | head -1 | cut -d'=' -f2 | tr -d ' "')
[[ "$STATE" == "inactive" ]] && g "deactivate → INACTIVE" || { e "deactivate failed (state=$STATE)"; FAILED=$((FAILED+1)); }
ros2 service call /vxl_camera/change_state \
  lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}" > /dev/null
sleep 1
STATE=$(ros2 service call /vxl_camera/get_state lifecycle_msgs/srv/GetState 2>/dev/null \
  | grep -oE 'label=[^,]*' | head -1 | cut -d'=' -f2 | tr -d ' "')
[[ "$STATE" == "active" ]] && g "activate → ACTIVE" || { e "activate failed (state=$STATE)"; FAILED=$((FAILED+1)); }

# ─── Summary ────────────────────────────────────────────────────────────────
echo
if [[ $FAILED -eq 0 ]]; then
  g "Smoke test PASSED ($MODE, $DURATION s)"
  exit 0
else
  e "Smoke test FAILED — $FAILED check(s) failed. Tail of node log:"
  tail -20 "$LAUNCH_LOG" >&2
  exit 1
fi
