#!/usr/bin/env bash
# byd_record_session.sh — record one end-to-end dataset session on the device.
#
#   ./byd_record_session.sh <device-ip> [recorder args...]
#   ./byd_record_session.sh 192.168.1.50 --fps 10.5
#
# Touches the device exactly three times:
#   1. one ssh: verify the deployed recorder matches this checkout, start it
#      detached (setsid+nohup, survives this ssh session), confirm it is alive
#   2. one ssh on Ctrl-C: SIGTERM that recorder only, wait for a clean close,
#      print md5s of everything it wrote
#   3. one scp: pull the session folder to ~/Desktop/ROSbag/end-end/<id>/,
#      then verify every file against the md5s from step 2
#
# The start and stop-and-pull steps live in byd_recorder_lib.sh, which
# `byd_drive.sh --with-recorder` uses too, so both run the same code. This
# script never calls byd_drive.sh / odom_node.py and never touches the cereal
# server; the device side uses no ROS runtime.
# The device IP moves -- find it with nmap, do not assume.
#
# Refuses to start if a recorder is already running on the device, and prints
# the command to stop that one instead of taking it over.
# If the stop ssh fails, the recorder keeps recording and stops itself on low
# disk. Re-run the printed stop command later; a plain SIGTERM closes it cleanly.
# Nothing is deleted from the device.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=byd_recorder_lib.sh
source "$SCRIPT_DIR/byd_recorder_lib.sh"
DEVICE_IP="${1:-}"
[ -n "$DEVICE_IP" ] || { echo "usage: $0 <device-ip> [recorder args...]" >&2; exit 2; }
shift

recorder_start "$DEVICE_IP" "$@" || exit 1

STOP=0
trap 'STOP=1' INT TERM
echo "[session] recording. Ctrl-C to stop (the device keeps recording if this terminal dies)."
# Ctrl-C also kills this sleep (exit 130); without `|| true`, set -e would end
# the script right after the trap and the stop below would never run.
while [ "$STOP" -eq 0 ]; do sleep 1 || true; done

rc=0
recorder_stop_and_pull "$DEVICE_IP" "$RECORDER_SESSION_ID" || rc=$?
exit $rc
