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
# Independent of byd_drive.sh / odom_node.py by design: it never calls them,
# never touches the cereal server, and the device side uses no ROS runtime.
# The device IP moves -- find it with nmap, do not assume.
#
# If the stop ssh fails, the recorder keeps recording and stops itself on low
# disk. Re-run the printed stop command later; a plain SIGTERM closes it cleanly.
# Nothing is deleted from the device.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEVICE_IP="${1:-}"
[ -n "$DEVICE_IP" ] || { echo "usage: $0 <device-ip> [recorder args...]" >&2; exit 2; }
shift
DEVICE="kommu@${DEVICE_IP}"
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=10 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 "$DEVICE")

REMOTE_TOOLS=/data/kommu_tools
REMOTE_RECORDER=$REMOTE_TOOLS/byd_e2e_recorder.py
REMOTE_CAMREC=$REMOTE_TOOLS/byd_road_camera_rosbag_recorder.py
LOCAL_RECORDER=$SCRIPT_DIR/byd_e2e_recorder.py
LOCAL_CAMREC=$SCRIPT_DIR/byd_road_camera_rosbag_recorder.py
SESSION_ID="$(date +%Y%m%d-%H%M%S)"
REMOTE_SESSION=$REMOTE_TOOLS/e2e_sessions/$SESSION_ID
LOCAL_PARENT="$HOME/Desktop/ROSbag/end-end"
LOCAL_SESSION="$LOCAL_PARENT/$SESSION_ID"
PIDFILE=/tmp/byd_e2e_recorder.pid

REC_MD5="$(md5sum "$LOCAL_RECORDER" | cut -d' ' -f1)"
CAM_MD5="$(md5sum "$LOCAL_CAMREC" | cut -d' ' -f1)"
EXTRA=""
for a in "$@"; do EXTRA+=" $(printf '%q' "$a")"; done

echo "[session] $SESSION_ID -> $DEVICE:$REMOTE_SESSION"

# ---- 1. start ---------------------------------------------------------------
START_OUT="$("${SSH[@]}" "
  [ \"\$(md5sum $REMOTE_RECORDER 2>/dev/null | cut -d' ' -f1)\" = $REC_MD5 ] || { echo MD5_MISMATCH recorder; exit 10; }
  [ \"\$(md5sum $REMOTE_CAMREC 2>/dev/null | cut -d' ' -f1)\" = $CAM_MD5 ] || { echo MD5_MISMATCH camrec; exit 10; }
  mkdir -p $REMOTE_SESSION && cd /data/openpilot || exit 12
  setsid nohup env PYTHONPATH=/data/kommu_tools/pylibs:/data/kommu_tools:/data/openpilot \
    /usr/local/venv/bin/python3 -u $REMOTE_RECORDER --session-dir $REMOTE_SESSION$EXTRA \
    > $REMOTE_SESSION/recorder.log 2>&1 < /dev/null &
  sleep 4
  P=\$(cat $PIDFILE 2>/dev/null || true)
  if [ -n \"\$P\" ] && grep -qa $REMOTE_SESSION /proc/\$P/cmdline 2>/dev/null; then echo STARTED \$P; else echo START_FAILED; fi
  tail -n 5 $REMOTE_SESSION/recorder.log
")" || true
echo "$START_OUT" | sed 's/^/[device] /'
if echo "$START_OUT" | grep -q '^MD5_MISMATCH'; then
  echo "[session] the device copy differs from this checkout. Deploy first:" >&2
  echo "          scp $LOCAL_RECORDER $LOCAL_CAMREC $DEVICE:$REMOTE_TOOLS/" >&2
  exit 1
fi
echo "$START_OUT" | grep -q '^STARTED' || { echo "[session] recorder did not start (see log above)" >&2; exit 1; }

STOP_CMD="
  P=\$(cat $PIDFILE 2>/dev/null || true)
  if [ -n \"\$P\" ] && grep -qa $REMOTE_SESSION /proc/\$P/cmdline 2>/dev/null; then
    kill -TERM \$P
    for i in \$(seq 1 60); do kill -0 \$P 2>/dev/null || break; sleep 1; done
    if kill -0 \$P 2>/dev/null; then echo STILL_RUNNING; exit 11; fi
    echo STOPPED
  else
    echo NOT_RUNNING
  fi
  [ -f $REMOTE_SESSION/.complete ] && echo COMPLETE || echo NOT_COMPLETE
  cd $REMOTE_SESSION && find . -type f -exec md5sum {} + | sed 's/^/MD5 /'
"

# ---- 2. wait, then stop -----------------------------------------------------
STOP=0
trap 'STOP=1' INT TERM
echo "[session] recording. Ctrl-C to stop (the device keeps recording if this terminal dies)."
# Ctrl-C also kills this sleep (exit 130); without `|| true`, set -e would end
# the script right after the trap and the stop below would never run.
while [ "$STOP" -eq 0 ]; do sleep 1 || true; done
# A second Ctrl-C must not abort the stop or the pull halfway.
trap '' INT
echo
echo "[session] stopping... (Ctrl-C is ignored until the bag is pulled)"
if ! STOP_OUT="$("${SSH[@]}" "$STOP_CMD")"; then
  echo "$STOP_OUT" | sed 's/^/[device] /'
  echo "[session] STOP DID NOT COMPLETE. The recorder is still writing safely. Retry with:" >&2
  echo "          ssh $DEVICE 'kill -TERM \$(cat $PIDFILE)'   # recorder closes the bag within ~1 s" >&2
  echo "          then: scp -r $DEVICE:$REMOTE_SESSION $LOCAL_PARENT/" >&2
  exit 1
fi
echo "$STOP_OUT" | grep -v '^MD5 ' | sed 's/^/[device] /'

# ---- 3. pull and verify -----------------------------------------------------
mkdir -p "$LOCAL_PARENT"
scp -r -q "$DEVICE:$REMOTE_SESSION" "$LOCAL_PARENT/"
bad=0
while read -r _ sum rel; do
  [ -z "$rel" ] && continue
  got="$(md5sum "$LOCAL_SESSION/$rel" 2>/dev/null | cut -d' ' -f1 || true)"
  [ "$got" = "$sum" ] || { echo "[session] TRANSFER MISMATCH: $rel" >&2; bad=1; }
done < <(echo "$STOP_OUT" | grep '^MD5 ')
[ "$bad" -eq 0 ] && echo "[session] pulled and verified -> $LOCAL_SESSION"
if ls "$LOCAL_SESSION"/'!!'* >/dev/null 2>&1; then
  echo "[session] !!! $(cat "$LOCAL_SESSION"/'!!'*)" >&2
fi
echo "$STOP_OUT" | grep -q '^COMPLETE' || echo "[session] WARNING: no .complete marker -- bag may not have closed cleanly" >&2
echo "[session] device copy kept at $DEVICE:$REMOTE_SESSION (not deleted)"
exit $bad
