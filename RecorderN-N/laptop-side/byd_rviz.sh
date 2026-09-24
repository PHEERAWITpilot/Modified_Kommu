#!/usr/bin/env bash
# byd_rviz.sh — open RViz2 ALONE, with the byd_odom display config already set.
#
# Companion to byd_drive.sh, which starts the odom_node AND RViz together. Use
# this one when the node is already running (started by byd_drive.sh, or left
# running from an earlier session) and you only want to open, close, or reopen
# the viewer without disturbing the integrator.
#
# WHY THIS MATTERS: closing RViz is free, but restarting odom_node resets every
# track to the origin and throws away the whole drive's accumulated path. Keep
# the node up; cycle only the viewer.
#
# Usage:
#   ./byd_rviz.sh              # default config: the package's byd_odom.rviz
#   ./byd_rviz.sh path/to.rviz # a different config file
#
# Nothing here publishes, so it is safe to run more than once — though two RViz
# windows on the same topics just doubles the subscriber load for no benefit.

set -eo pipefail

ROS_SETUP="/opt/ros/jazzy/setup.bash"
WS_SETUP="$HOME/ros2_ws/install/setup.bash"

[ -f "$ROS_SETUP" ] || { echo "[byd-rviz] missing $ROS_SETUP" >&2; exit 1; }
[ -f "$WS_SETUP" ]  || { echo "[byd-rviz] missing $WS_SETUP — run: cd ~/ros2_ws && colcon build --packages-select byd_odom_ros" >&2; exit 1; }

# ROS's own setup.bash trips over `set -u` (AMENT_TRACE_SETUP_FILES is unbound),
# so nounset is disabled just for the sourcing and restored straight after.
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
# shellcheck disable=SC1090
source "$WS_SETUP"
set -u

# Resolve the installed config rather than the source tree, so this matches what
# ros2 launch would load. colcon does a COPY install, not a symlink, so an edit
# to rviz/byd_odom.rviz under src/ does nothing until you rebuild.
if [ $# -ge 1 ]; then
  CONFIG="$1"
else
  PKG_SHARE="$(ros2 pkg prefix --share byd_odom_ros 2>/dev/null || true)"
  CONFIG="${PKG_SHARE}/rviz/byd_odom.rviz"
fi

if [ ! -f "$CONFIG" ]; then
  echo "[byd-rviz] config not found: $CONFIG" >&2
  echo "[byd-rviz] rebuild the package, or pass a config path explicitly." >&2
  exit 1
fi

echo "[byd-rviz] config: $CONFIG"

# Informational only — RViz is perfectly happy with no publisher, it just draws
# nothing. Saying so up front avoids ten minutes of staring at an empty grid.
if pgrep -f "[l]ib/byd_odom_ros/odom_node" >/dev/null 2>&1; then
  echo "[byd-rviz] odom_node is running — paths will populate."
else
  echo "[byd-rviz] NOTE: no odom_node running. RViz will open but stay empty."
  echo "[byd-rviz]       start it with:  ./byd_drive.sh <device-ip>"
fi

# On exit, ask the node to write the run out. The node also saves on its own
# shutdown, so this is for the case where the viewer is closed but the node
# keeps running: it snapshots the drive so far without disturbing it. Saving is
# once-per-run inside the node, so a snapshot here and a later shutdown do not
# produce two partial copies.
save_on_exit() {
  if ! pgrep -f "lib/byd_odom_ros/odom_node" >/dev/null 2>&1; then
    return 0
  fi
  echo "[byd-rviz] requesting odom save ..."
  if timeout 15 ros2 service call /byd/save_record std_srvs/srv/Trigger >/tmp/byd_save_record.out 2>&1; then
    grep -o "message='[^']*'" /tmp/byd_save_record.out | sed "s/^/[byd-rviz] /" || true
  else
    echo "[byd-rviz] save request failed (see /tmp/byd_save_record.out)." >&2
    echo "[byd-rviz] the node still saves on its own shutdown, so nothing is lost yet." >&2
  fi
}
trap save_on_exit EXIT

echo "[byd-rviz] starting RViz2 (close the window, or Ctrl-C here, to quit)"
# NOT exec: the EXIT trap must still run after RViz returns.
rviz2 -d "$CONFIG"
