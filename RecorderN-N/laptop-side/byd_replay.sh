#!/usr/bin/env bash
# byd_replay.sh — open a SAVED recording in RViz.
#
#   ./byd_replay.sh                     # newest recording under Odom_record
#   ./byd_replay.sh 2026/09/09/1607     # that run (path relative to Odom_record)
#   ./byd_replay.sh /full/path/odom.csv # an explicit file
#   ./byd_replay.sh --realtime          # animate instead of drawing it all at once
#   ./byd_replay.sh --realtime --speed 5
#
# Publishes the recorded tracks to the same topic names the live node uses, so
# the existing byd_odom.rviz shows a past drive with the same colours. Extra
# flags are passed through to byd_odom_replay.py.
#
# REFUSES to run while a live odom_node is up: both would publish to the same
# topics and RViz would interleave the live drive with the recording — the same
# duplicate-publisher failure byd_drive.sh guards against.

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RECORD_DIR="${BYD_RECORD_DIR:-$HOME/Desktop/Kommu.AI/Odom_record}"

TARGET=""
EXTRA=()
for a in "$@"; do
  case "$a" in
    -*) EXTRA+=("$a") ;;
    *)  if [ -z "$TARGET" ]; then TARGET="$a"; else EXTRA+=("$a"); fi ;;
  esac
done

if pgrep -f "lib/byd_odom_ros/odom_node" >/dev/null 2>&1; then
  echo "[byd-replay] a LIVE odom_node is running." >&2
  echo "[byd-replay] Replaying now would publish a recording onto the same topics" >&2
  echo "[byd-replay] as the live drive and RViz would draw both at once." >&2
  echo "[byd-replay] Stop the live node first, then re-run this." >&2
  exit 1
fi

# Resolve the recording to play.
if [ -z "$TARGET" ]; then
  [ -d "$RECORD_DIR" ] || { echo "[byd-replay] no recordings yet: $RECORD_DIR" >&2; exit 1; }
  CSV="$(find "$RECORD_DIR" -name odom.csv -printf '%T@ %p\n' 2>/dev/null | sort -rn | head -1 | cut -d' ' -f2-)"
  [ -n "$CSV" ] || { echo "[byd-replay] no odom.csv found under $RECORD_DIR" >&2; exit 1; }
  echo "[byd-replay] newest recording selected"
elif [ -f "$TARGET" ]; then
  CSV="$TARGET"
elif [ -f "$TARGET/odom.csv" ]; then
  CSV="$TARGET/odom.csv"
elif [ -f "$RECORD_DIR/$TARGET/odom.csv" ]; then
  CSV="$RECORD_DIR/$TARGET/odom.csv"
else
  echo "[byd-replay] cannot find a recording for: $TARGET" >&2
  echo "[byd-replay] tried it as a file, a directory, and as a path under $RECORD_DIR" >&2
  exit 1
fi

echo "[byd-replay] $CSV"
if [ -f "$(dirname "$CSV")/meta.json" ]; then
  sed -n 's/^  "\(started\|duration_s\|samples\|reason\)": \(.*\),\?$/[byd-replay]   \1: \2/p' \
    "$(dirname "$CSV")/meta.json" || true
fi

# ROS setup.bash trips over `set -u`, so nounset is off for the sourcing only.
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$HOME/ros2_ws/install/setup.bash"
set -u

CONFIG="$(ros2 pkg prefix --share byd_odom_ros 2>/dev/null)/rviz/byd_odom.rviz"

# `python3` on PATH is whatever venv is active, and a project venv generally
# has no rclpy — the ROS node itself runs under the system interpreter. Pick a
# python that can actually import rclpy rather than assuming.
PY="${BYD_PYTHON:-/usr/bin/python3}"
if ! "$PY" -c "import rclpy" >/dev/null 2>&1; then
  if python3 -c "import rclpy" >/dev/null 2>&1; then
    PY="python3"
  else
    echo "[byd-replay] no python with rclpy found (tried $PY and python3)." >&2
    echo "[byd-replay] If your ROS install uses a different interpreter, set:" >&2
    echo "[byd-replay]     BYD_PYTHON=/path/to/python ./byd_replay.sh ..." >&2
    exit 1
  fi
fi

cleanup() { [ -n "${REPLAY_PID:-}" ] && kill "$REPLAY_PID" 2>/dev/null || true; }
trap cleanup EXIT

"$PY" "$SCRIPT_DIR/byd_odom_replay.py" "$CSV" ${EXTRA[@]+"${EXTRA[@]}"} &
REPLAY_PID=$!

# Fixed Frame is `odom`; the replay publishes TF for it, but RViz can start
# before the first message and complain briefly. That settles on its own.
sleep 3
# Opening RViz on a dead replay just shows an empty grid and looks like the
# recording was bad, so fail loudly here instead.
if ! kill -0 "$REPLAY_PID" 2>/dev/null; then
  echo "[byd-replay] the replay node exited during startup (see the error above)." >&2
  echo "[byd-replay] not opening RViz — it would show an empty grid." >&2
  exit 1
fi

echo "[byd-replay] opening RViz (close it, or Ctrl-C here, to stop the replay)"
rviz2 -d "$CONFIG" || true
