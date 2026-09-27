#!/usr/bin/env bash
# byd_drive.sh — one command: ensure the device cereal server is up, then
# launch the dual-track odometry visualisation.
#
#   ./byd_drive.sh                # defaults to 172.20.10.2
#   ./byd_drive.sh 192.168.1.42   # device IP is dynamic — pass it if it moved
#   ./byd_drive.sh 172.20.10.2 rviz:=false   # headless (extra args pass through)
#   ./byd_drive.sh 172.20.10.2 --kill-existing  # stop any running node first
#   ./byd_drive.sh 172.20.10.2 --allow-multiple # deliberately run a second node
#   ./byd_drive.sh 172.20.10.2 --no-attach      # if one is running, do nothing
#   ./byd_drive.sh 172.20.10.2 --reverse-only   # gear sign ONLY for R, else +1
#   ./byd_drive.sh 172.20.10.2 --no-gear-sign   # A/B: ignore gear entirely
#   ./byd_drive.sh 172.20.10.2 gear_mode:=reverse-only   # launch-arg form
#   ./byd_drive.sh 192.168.1.50 --with-recorder  # also record an e2e dataset session
#
# --with-recorder (opt-in; without it nothing below applies and no extra SSH
# call is made): after byd_yawcheck.sh and byd_ensure_cereal_server.sh have
# finished, start the device-side e2e recorder (byd_recorder_lib.sh, the same
# code byd_record_session.sh uses), THEN launch the node. If the recorder cannot
# start -- disk low, CAN reader slots exhausted, device copy out of date, or a
# recorder already running -- the node is NOT launched. On Ctrl-C the node shuts
# down, then the recorder is stopped, its session pulled to
# ~/Desktop/ROSbag/end-end/<id>/ and md5-verified (this can take minutes on
# slow WiFi). If the launch ends WITHOUT Ctrl-C (the node was replaced from
# another terminal, or crashed) the recorder is left running and the command
# to stop it is printed. Refused when a node is already running, because that
# path only attaches RViz and has nowhere to stop a recorder.
#
# ACCEPTED RISK, NOT A BUG: a node restart runs byd_yawcheck.sh again, and if
# that finds carstate.py reverted it REBOOTS THE DEVICE, which kills a running
# recorder mid-bag with no clean stop and no pull. Two paths restart the node:
#   1. an explicit --kill-existing, and
#   2. the AUTOMATIC restart this script does when the running node is older
#      than the installed build (i.e. you rebuilt since launching it).
# Neither protects an in-progress --with-recorder recording from that reboot.
# If you need that protection, stop the whole session (Ctrl-C) and start a new
# one instead of restarting the node mid-recording. Re-running this script
# WITHOUT --kill-existing while a node is up only re-opens RViz: it runs no
# yawcheck and touches neither the device nor the recorder.
#
# If an odom_node is ALREADY running, this attaches to it: it opens RViz only
# and leaves the node — and the path it has accumulated — completely alone.
# Restarting the node would reset every track to the origin and throw the
# drive away, so attaching is almost always what you want mid-session.
#
# EXCEPTION: if that node started BEFORE the installed code last changed (you
# rebuilt since launching it), it is running the OLD build — new tracks such as
# /byd/path_ekf_ba_slow simply do not exist in it, and RViz would show their displays
# with nothing behind them. In that case the node is restarted on the new build
# automatically, exactly as --kill-existing would. Its drive so far is saved on
# shutdown; the live path resets to the origin. --no-attach still means "touch
# nothing", and --allow-multiple still starts a second node instead.
#
# Sources the ROS overlays explicitly, so this works from a fresh terminal
# whether or not ~/.bashrc sources ~/ros2_ws/install/setup.bash.
set -euo pipefail
DEVICE_IP="${1:-172.20.10.2}"
shift || true
EXTRA=()
KILL_EXISTING=0
ALLOW_MULTIPLE=0
NO_ATTACH=0
WANT_RVIZ=1
WITH_RECORDER=0
for a in "$@"; do
  case "$a" in
    --kill-existing)  KILL_EXISTING=1 ;;
    --with-recorder)  WITH_RECORDER=1 ;;
    --allow-multiple) ALLOW_MULTIPLE=1 ;;
    --no-attach)      NO_ATTACH=1 ;;
    # Convenience aliases. The node's setting is a launch ARGUMENT, not a
    # bare flag, because ros2 launch only accepts name:=value — so translate
    # here rather than making the user remember which form this script wants.
    --reverse-only)   EXTRA+=("gear_mode:=reverse-only") ;;
    --no-gear-sign)   EXTRA+=("gear_mode:=off") ;;
    rviz:=false|rviz:=False|rviz:=0)
                      WANT_RVIZ=0; EXTRA+=("$a") ;;
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

# The code a NEWLY launched node would run: colcon's installed copy.
INSTALLED_NODE_GLOB="$HOME/ros2_ws/install/byd_odom_ros/lib/python3.*/site-packages/byd_odom_ros/odom_node.py"

# When the installed code last actually changed on disk. CTIME (%Z), not mtime:
# colcon's copy preserves the SOURCE file's mtime, so after the ordinary
# "edit -> launch a drive -> rebuild" sequence the installed mtime predates the
# running node even though the node is running old code. ctime is set by the
# copy itself and cannot be carried over.
installed_ctime() {
  local newest=0 f m
  for f in $INSTALLED_NODE_GLOB; do
    [[ -f "$f" ]] || continue
    m="$(stat -c %Z "$f" 2>/dev/null || echo 0)"
    if (( m > newest )); then newest=$m; fi
  done
  echo "$newest"
}

# Of the PIDs given, print those that started before the installed code last
# changed, i.e. nodes still running a previous build.
stale_nodes() {
  local inst now pid et out=""
  inst="$(installed_ctime)"
  if (( inst == 0 )); then echo ""; return 0; fi
  now="$(date +%s)"
  for pid in "$@"; do
    et="$(ps -o etimes= -p "$pid" 2>/dev/null | tr -d ' ')"
    [[ -n "$et" ]] || continue
    if (( now - et < inst )); then out="$out $pid"; fi
  done
  echo "${out# }"
}

existing="$(find_nodes)"
if [[ -n "$existing" ]]; then
  echo "[byd-drive] an odom_node is ALREADY running:" >&2
  ps -o pid=,etime=,args= -p $existing 2>/dev/null | sed 's/^/    /' >&2
  # shellcheck disable=SC2086
  stale="$(stale_nodes $existing)"
  if [[ -n "$stale" && "$KILL_EXISTING" != "1" && "$ALLOW_MULTIPLE" != "1" ]]; then
    echo "" >&2
    echo "[byd-drive] that node STARTED BEFORE the installed code last changed" >&2
    echo "[byd-drive] ($(date -d @"$(installed_ctime)" '+%F %T')), so it is running an OLD build." >&2
    echo "[byd-drive] Attaching RViz to it would show displays (e.g. /byd/path_ekf_ba_slow) with" >&2
    echo "[byd-drive] nothing behind them." >&2
    if [[ "$NO_ATTACH" == "1" ]]; then
      echo "[byd-drive] --no-attach: leaving it alone. Re-run without it to restart." >&2
      exit 1
    fi
    echo "[byd-drive] restarting it on the new build. Its drive so far is saved on" >&2
    echo "[byd-drive] shutdown; the live path resets to the origin." >&2
    # Implicit restart: same accepted reboot risk as an explicit --kill-existing
    # for a recorder that is already running (see the header).
    KILL_EXISTING=1
  fi
  if [[ "$KILL_EXISTING" == "1" ]]; then
    # ACCEPTED RISK: this does NOT touch a running e2e recorder, but the restart
    # below re-runs byd_yawcheck.sh, which may reboot the device and kill that
    # recorder mid-bag with no clean stop or pull. Reached both by --kill-existing
    # and by the automatic stale-build restart above. Documented in the header.
    echo "[byd-drive] stopping it" >&2
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
    # Not an error: a node is already up, which is the normal mid-session state.
    # Starting a second one would publish to the same topics with independent
    # integrator state and make the path jump, so instead attach to the running
    # one by opening the viewer alone. Closing RViz does NOT stop the node.
    echo "" >&2
    echo "[byd-drive] not starting a second node — two would publish to the same" >&2
    echo "[byd-drive] topics with independent integrator state and make the path jump." >&2
    if [[ "$WITH_RECORDER" == "1" ]]; then
      # This path only exec's RViz, so there would be no Ctrl-C handler left to
      # stop and pull a recorder. Refuse rather than start one nobody stops.
      echo "[byd-drive] --with-recorder refused: a node is already running, and this" >&2
      echo "[byd-drive] invocation would only re-open RViz. To record, Ctrl-C the running" >&2
      echo "[byd-drive] session and start a new one with --with-recorder. To just re-open" >&2
      echo "[byd-drive] RViz, re-run without --with-recorder (touches no recorder)." >&2
      exit 1
    fi
    if [[ "$NO_ATTACH" == "1" ]]; then
      echo "[byd-drive] --no-attach: doing nothing. Use --kill-existing to replace it," >&2
      echo "[byd-drive] or --allow-multiple to run two anyway." >&2
      exit 1
    fi
    if [[ "$WANT_RVIZ" == "0" ]]; then
      echo "[byd-drive] rviz:=false was requested, so there is nothing left to do." >&2
      exit 1
    fi
    if [[ ! -x "$SCRIPT_DIR/byd_rviz.sh" ]]; then
      echo "[byd-drive] byd_rviz.sh not found next to this script — cannot attach." >&2
      echo "[byd-drive] Use --kill-existing to replace the running node instead." >&2
      exit 1
    fi
    echo "[byd-drive] ATTACHING: opening RViz only. The running node and the path" >&2
    echo "[byd-drive] it has already accumulated are left untouched." >&2
    echo "[byd-drive] (Closing RViz will NOT stop the node. --kill-existing does.)" >&2
    echo "" >&2
    # The device server is not re-checked here: the running node is already
    # consuming that stream, so it is up by definition, and a viewer-only
    # attach has no reason to touch the device over SSH.
    exec "$SCRIPT_DIR/byd_rviz.sh"
  fi
fi

# PRE-FLIGHT: carstate.py is a tracked file and the auto-updater reclaims it
# roughly daily, silently reverting CS.yawRate to 0.0. Catch that here, before
# a drive, rather than discovering it afterwards from a flat `measured` track.
# Fast path is one ssh + md5 (~0.5 s); only a mismatch costs a redeploy+reboot.
# With --with-recorder this MUST stay before the recorder start below: a reboot
# triggered here then cannot kill a recorder, because none is running yet.
"$SCRIPT_DIR/byd_yawcheck.sh" "$DEVICE_IP"

"$SCRIPT_DIR/byd_ensure_cereal_server.sh" "$DEVICE_IP"

if [[ "$WITH_RECORDER" == "1" ]]; then
  # shellcheck source=byd_recorder_lib.sh
  source "$SCRIPT_DIR/byd_recorder_lib.sh"
  RECORDER_TAG="[byd-drive/recorder]"
  if ! recorder_start "$DEVICE_IP"; then
    echo "[byd-drive] --with-recorder: the recorder did not start, so the node is NOT" >&2
    echo "[byd-drive] being launched. Fix the cause above, or drive without --with-recorder." >&2
    exit 1
  fi
fi

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
if [[ "$WITH_RECORDER" != "1" ]]; then
  exec ros2 launch byd_odom_ros odom_rviz.launch.py host:="$DEVICE_IP" ${EXTRA[@]+"${EXTRA[@]}"}
fi

# --with-recorder only: launch in the FOREGROUND (not exec, and never in the
# background -- a background job of a non-interactive shell ignores SIGINT, so
# Ctrl-C could not stop it), so this shell survives to stop the recorder after.
# Ctrl-C reaches ros2 launch as usual; the trap only records that it happened.
# `|| rc=$?`: the node sometimes exits 1 at shutdown, and set -e must not end
# this script before the recorder is stopped.
GOT_INT=0
trap 'GOT_INT=1' INT
rc=0
ros2 launch byd_odom_ros odom_rviz.launch.py host:="$DEVICE_IP" ${EXTRA[@]+"${EXTRA[@]}"} || rc=$?
if [[ "$GOT_INT" == "1" ]]; then
  echo "[byd-drive] node stopped (launch exit $rc). Stopping the recorder and pulling the session..."
  prc=0
  recorder_stop_and_pull "$DEVICE_IP" "$RECORDER_SESSION_ID" || prc=$?
  exit $prc
fi
# Ended without our Ctrl-C: replaced via --kill-existing from another terminal,
# crashed, or failed to start. Leave the recorder alone, by design.
echo "[byd-drive] launch ended WITHOUT Ctrl-C (exit $rc). The recorder is LEFT RUNNING" >&2
echo "[byd-drive] on purpose (session $RECORDER_SESSION_ID)." >&2
recorder_print_stop_help "$DEVICE_IP" "$RECORDER_REMOTE_SESSION"
exit $rc
