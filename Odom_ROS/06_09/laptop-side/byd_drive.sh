#!/usr/bin/env bash
# byd_drive.sh — one command: ensure the device cereal server is up, then
# launch the dual-track odometry visualisation.
#
#   ./byd_drive.sh                # defaults to 172.20.10.2
#   ./byd_drive.sh 192.168.1.42   # device IP is dynamic — pass it if it moved
#   ./byd_drive.sh 172.20.10.2 rviz:=false   # headless (extra args pass through)
#   ./byd_drive.sh 172.20.10.2 --kill-existing  # stop any running node first
#   ./byd_drive.sh 172.20.10.2 --allow-multiple # deliberately run a second node
#
# Sources the ROS overlays explicitly, so this works from a fresh terminal
# whether or not ~/.bashrc sources ~/ros2_ws/install/setup.bash.
set -euo pipefail
DEVICE_IP="${1:-172.20.10.2}"
shift || true
EXTRA=()
KILL_EXISTING=0
ALLOW_MULTIPLE=0
for a in "$@"; do
  case "$a" in
    --kill-existing)  KILL_EXISTING=1 ;;
    --allow-multiple) ALLOW_MULTIPLE=1 ;;
    *) EXTRA+=("$a") ;;
  esac
done
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ---------------------------------------------------------------------------
# GUARD: refuse to start a SECOND odom_node.
#
# Two instances publish to the same topics with INDEPENDENT integrator state,
# so RViz draws two diverging trajectories interleaved and the path appears to
# "jump". It is invisible to `ros2 topic hz` and it silently corrupted three
# separate measurements on 2026-09-05 before being spotted. Process check, not
# a ROS check, because it must work before the ROS overlays are even sourced.
# ---------------------------------------------------------------------------
NODE_PAT='lib/byd_odom_ros/odom_node'

# `pgrep -f` matches ANY process whose command line contains the pattern —
# including the shell that invoked this script, if the user typed the path.
# That is not hypothetical: it self-matched during testing on 2026-09-05, and
# with --kill-existing it would have killed the calling shell. So filter the
# candidates down to processes that are actually a python interpreter.
find_nodes() {
  local pid out=""
  for pid in $(pgrep -f "$NODE_PAT" 2>/dev/null || true); do
    [[ "$pid" == "$$" || "$pid" == "$PPID" ]] && continue
    local comm
    comm="$(cat "/proc/$pid/comm" 2>/dev/null || true)"
    # The real node's comm is "odom_node" (ros2 run renames it); a plain
    # interpreter launch would be "python3". Anything else matching the pattern
    # is a shell/ssh/grep whose COMMAND LINE merely mentions the path — exclude
    # those, or the guard fires on its own caller. Verified on-box 2026-09-05:
    # the live node reports comm=odom_node, so an allow-list of python* alone
    # silently disabled the whole guard.
    case "$comm" in
      odom_node|python*) ;;
      *) continue ;;
    esac
    out="$out $pid"
  done
  echo "${out# }"
}
existing="$(find_nodes)"
if [[ -n "$existing" ]]; then
  echo "[byd-drive] an odom_node is ALREADY running:" >&2
  ps -o pid=,etime=,args= -p $existing 2>/dev/null | sed 's/^/    /' >&2
  if [[ "$KILL_EXISTING" == "1" ]]; then
    echo "[byd-drive] --kill-existing: stopping it" >&2
    # shellcheck disable=SC2086
    kill $existing 2>/dev/null || true
    for _ in 1 2 3 4 5 6 7 8 9 10; do
      [[ -z "$(find_nodes)" ]] && break
      sleep 0.5
    done
    if [[ -n "$(find_nodes)" ]]; then
      # shellcheck disable=SC2046
      kill -9 $(find_nodes) 2>/dev/null || true
      sleep 1
    fi
    echo "[byd-drive] stopped." >&2
  elif [[ "$ALLOW_MULTIPLE" == "1" ]]; then
    echo "[byd-drive] --allow-multiple given: starting a SECOND node anyway." >&2
    echo "[byd-drive] WARNING: both will publish to the same topics with separate" >&2
    echo "[byd-drive] integrator state. Expect the path to jump. You asked for it." >&2
  else
    echo "" >&2
    echo "[byd-drive] REFUSING to start a second node — it would publish to the same" >&2
    echo "[byd-drive] topics with its own integrator state and make the path jump." >&2
    echo "[byd-drive] Re-run with --kill-existing to replace it, or --allow-multiple" >&2
    echo "[byd-drive] if you genuinely want two. To check by hand:" >&2
    echo "[byd-drive]     ros2 topic info /byd/odom_corrected   # Publisher count must be 1" >&2
    exit 1
  fi
fi

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
exec ros2 launch byd_odom_ros odom_rviz.launch.py host:="$DEVICE_IP" ${EXTRA[@]+"${EXTRA[@]}"}
