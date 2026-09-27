# byd_recorder_lib.sh — start / stop-and-pull for the device-side e2e recorder.
# SOURCED, not run. Shared by byd_record_session.sh and `byd_drive.sh --with-recorder`
# so both drive the recorder through exactly the same code.
#
#   recorder_start <device-ip> [recorder args...]   0 = started, 1 = refused/failed
#       sets RECORDER_SESSION_ID, RECORDER_REMOTE_SESSION, RECORDER_LOCAL_SESSION
#   recorder_stop_and_pull <device-ip> <session-id>  0 = pulled and md5-verified
#   recorder_print_stop_help <device-ip> <remote-session-dir>
#
# Every device touch is one ssh (start), one ssh (stop + md5 list) and one scp.
# Nothing is deleted from the device. Messages are prefixed with $RECORDER_TAG.
# Written not to rely on the caller's `set -e`: every failure is handled here.

RECORDER_TAG="${RECORDER_TAG:-[session]}"
RECORDER_REMOTE_TOOLS=/data/kommu_tools
RECORDER_PIDFILE=/tmp/byd_e2e_recorder.pid
RECORDER_LOCAL_PARENT="$HOME/Desktop/ROSbag/end-end"
_RECORDER_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

_recorder_ssh() {  # <user@host> <remote command>
  ssh -o BatchMode=yes -o ConnectTimeout=10 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 "$1" "$2"
}

recorder_print_stop_help() {  # <device-ip> <remote-session-dir>
  echo "          stop it:  ssh kommu@$1 'kill -TERM \$(cat $RECORDER_PIDFILE)'   # closes the bag within ~1 s" >&2
  echo "          then:     scp -r kommu@$1:$2 $RECORDER_LOCAL_PARENT/" >&2
}

recorder_start() {  # <device-ip> [recorder args...]
  local ip="$1"; shift
  local dev="kommu@$ip" T="$RECORDER_TAG" rec_md5 cam_md5 extra="" a out line
  local remote_recorder="$RECORDER_REMOTE_TOOLS/byd_e2e_recorder.py"
  local remote_camrec="$RECORDER_REMOTE_TOOLS/byd_road_camera_rosbag_recorder.py"
  local local_recorder="$_RECORDER_LIB_DIR/byd_e2e_recorder.py"
  local local_camrec="$_RECORDER_LIB_DIR/byd_road_camera_rosbag_recorder.py"
  RECORDER_SESSION_ID="$(date +%Y%m%d-%H%M%S)"
  RECORDER_REMOTE_SESSION="$RECORDER_REMOTE_TOOLS/e2e_sessions/$RECORDER_SESSION_ID"
  RECORDER_LOCAL_SESSION="$RECORDER_LOCAL_PARENT/$RECORDER_SESSION_ID"
  rec_md5="$(md5sum "$local_recorder" | cut -d' ' -f1)"
  cam_md5="$(md5sum "$local_camrec" | cut -d' ' -f1)"
  for a in "$@"; do extra+=" $(printf '%q' "$a")"; done

  echo "$T $RECORDER_SESSION_ID -> $dev:$RECORDER_REMOTE_SESSION"
  out="$(_recorder_ssh "$dev" "
    P=\$(cat $RECORDER_PIDFILE 2>/dev/null || true)
    if [ -n \"\$P\" ] && grep -qa byd_e2e_recorder /proc/\$P/cmdline 2>/dev/null; then
      echo ALREADY_RUNNING \$P \$(tr '\\0' '\\n' < /proc/\$P/cmdline | grep -x -A1 -- --session-dir | tail -n 1); exit 13
    fi
    [ \"\$(md5sum $remote_recorder 2>/dev/null | cut -d' ' -f1)\" = $rec_md5 ] || { echo MD5_MISMATCH recorder; exit 10; }
    [ \"\$(md5sum $remote_camrec 2>/dev/null | cut -d' ' -f1)\" = $cam_md5 ] || { echo MD5_MISMATCH camrec; exit 10; }
    mkdir -p $RECORDER_REMOTE_SESSION && cd /data/openpilot || exit 12
    setsid nohup env PYTHONPATH=/data/kommu_tools/pylibs:/data/kommu_tools:/data/openpilot \
      /usr/local/venv/bin/python3 -u $remote_recorder --session-dir $RECORDER_REMOTE_SESSION$extra \
      > $RECORDER_REMOTE_SESSION/recorder.log 2>&1 < /dev/null &
    sleep 4
    P=\$(cat $RECORDER_PIDFILE 2>/dev/null || true)
    if [ -n \"\$P\" ] && grep -qa $RECORDER_REMOTE_SESSION /proc/\$P/cmdline 2>/dev/null; then echo STARTED \$P; else echo START_FAILED; fi
    tail -n 5 $RECORDER_REMOTE_SESSION/recorder.log
  ")" || true
  echo "$out" | sed 's/^/[device] /'

  line="$(echo "$out" | grep '^ALREADY_RUNNING' || true)"
  if [ -n "$line" ]; then
    echo "$T a recorder is ALREADY running on the device (pid $(echo "$line" | cut -d' ' -f2))," >&2
    echo "          writing $(echo "$line" | cut -d' ' -f3). Not starting a second one and not" >&2
    echo "          taking it over. Stop it yourself first:" >&2
    recorder_print_stop_help "$ip" "$(echo "$line" | cut -d' ' -f3)"
    return 1
  fi
  if echo "$out" | grep -q '^MD5_MISMATCH'; then
    echo "$T the device copy differs from this checkout. Deploy first:" >&2
    echo "          scp $local_recorder $local_camrec $dev:$RECORDER_REMOTE_TOOLS/" >&2
    return 1
  fi
  echo "$out" | grep -q '^STARTED' || { echo "$T recorder did not start (see log above)" >&2; return 1; }
  return 0
}

recorder_stop_and_pull() {  # <device-ip> <session-id>
  local ip="$1" sid="$2"
  local dev="kommu@$ip" T="$RECORDER_TAG" out bad=0 _ sum rel got
  local rsess="$RECORDER_REMOTE_TOOLS/e2e_sessions/$sid"
  local lsess="$RECORDER_LOCAL_PARENT/$sid"
  local stop_cmd="
    P=\$(cat $RECORDER_PIDFILE 2>/dev/null || true)
    if [ -n \"\$P\" ] && grep -qa $rsess /proc/\$P/cmdline 2>/dev/null; then
      kill -TERM \$P
      for i in \$(seq 1 60); do kill -0 \$P 2>/dev/null || break; sleep 1; done
      if kill -0 \$P 2>/dev/null; then echo STILL_RUNNING; exit 11; fi
      echo STOPPED
    else
      echo NOT_RUNNING
    fi
    [ -f $rsess/.complete ] && echo COMPLETE || echo NOT_COMPLETE
    cd $rsess && find . -type f -exec md5sum {} + | sed 's/^/MD5 /'
  "
  # A second Ctrl-C must not abort the stop or the pull halfway. (Inherited as
  # ignored by ssh/scp too, so only closing the terminal can interrupt them.)
  trap '' INT
  echo
  echo "$T stopping... (Ctrl-C is ignored until the bag is pulled)"
  if ! out="$(_recorder_ssh "$dev" "$stop_cmd")"; then
    echo "$out" | sed 's/^/[device] /'
    echo "$T STOP DID NOT COMPLETE. The recorder is still writing safely. Retry with:" >&2
    recorder_print_stop_help "$ip" "$rsess"
    return 1
  fi
  echo "$out" | grep -v '^MD5 ' | sed 's/^/[device] /'

  mkdir -p "$RECORDER_LOCAL_PARENT"
  if ! scp -r -q "$dev:$rsess" "$RECORDER_LOCAL_PARENT/"; then
    # Previously this failed silently under set -e and could leave a truncated
    # local copy that still carried .complete. Say so, and let the md5 check
    # below list exactly what is missing or short.
    echo "$T PULL FAILED (scp error). The device copy is intact. Retry with:" >&2
    echo "          scp -r $dev:$rsess $RECORDER_LOCAL_PARENT/" >&2
    bad=1
  fi
  while read -r _ sum rel; do
    [ -z "$rel" ] && continue
    got="$(md5sum "$lsess/$rel" 2>/dev/null | cut -d' ' -f1 || true)"
    [ "$got" = "$sum" ] || { echo "$T TRANSFER MISMATCH: $rel" >&2; bad=1; }
  done < <(echo "$out" | grep '^MD5 ')
  [ "$bad" -eq 0 ] && echo "$T pulled and verified -> $lsess"
  if ls "$lsess"/'!!'* >/dev/null 2>&1; then
    echo "$T !!! $(cat "$lsess"/'!!'*)" >&2
  fi
  echo "$out" | grep -q '^COMPLETE' || echo "$T WARNING: no .complete marker -- bag may not have closed cleanly" >&2
  echo "$T device copy kept at $dev:$rsess (not deleted)"
  return $bad
}
