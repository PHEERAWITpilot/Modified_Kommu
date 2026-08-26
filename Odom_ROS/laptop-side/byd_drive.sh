#!/usr/bin/env bash
# byd_drive.sh — one command: ensure the device cereal server is up, then
# launch the dual-track odometry visualisation.
#
#   ./byd_drive.sh                # defaults to 172.20.10.3
#   ./byd_drive.sh 192.168.1.42   # device IP is dynamic — pass it if it moved
#   ./byd_drive.sh 172.20.10.3 rviz:=false   # headless (extra args pass through)
#
# Sources the ROS overlays explicitly, so this works from a fresh terminal
# whether or not ~/.bashrc sources ~/ros2_ws/install/setup.bash.
set -euo pipefail
DEVICE_IP="${1:-172.20.10.3}"
shift || true
EXTRA=("$@")          # extra launch args pass through, e.g. rviz:=false
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

"$SCRIPT_DIR/byd_ensure_cereal_server.sh" "$DEVICE_IP"

# ROS's setup.bash references unset vars (AMENT_TRACE_SETUP_FILES and friends),
# so `set -u` makes sourcing it fail outright. Relax nounset just for these two
# lines, then restore it. Verified 2026-08-25.
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$HOME/ros2_ws/install/setup.bash"
set -u

echo "[byd-drive] launching odom + RViz against ${DEVICE_IP} ..."
exec ros2 launch byd_odom_ros odom_rviz.launch.py host:="$DEVICE_IP" "${EXTRA[@]}"
