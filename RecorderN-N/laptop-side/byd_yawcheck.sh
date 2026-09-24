#!/usr/bin/env bash
# byd_yawcheck.sh — verify the device's carstate.py still carries the YAW_SENSOR
# patch, and redeploy it if not.
#
#   ./byd_yawcheck.sh <device-ip>          # check, heal if needed
#   ./byd_yawcheck.sh <device-ip> --check   # report only, never redeploy
#
# WHY THIS EXISTS
#   cam_lka/carstate.py is a TRACKED file in the openpilot git tree, and the
#   auto-updater runs `git checkout --force -B <branch> FETCH_HEAD` on its own
#   schedule — roughly daily. That discards the uncommitted YAW_SENSOR patch and
#   CS.yawRate silently reverts to a permanent 0.0. Confirmed happening at
#   2026-09-08 13:21 and 2026-09-09 10:17 (parent repo reflog + UpdaterLastFetchTime).
#
#   The cereal server survives the same event only because it is UNTRACKED, and
#   `git checkout --force` leaves untracked files alone.
#
#   Rather than leaving updates disabled, this heals the patch at the start of
#   every drive session, so a reverted file is caught BEFORE a drive rather than
#   discovered afterwards from data whose `measured` track was a flat zero.
#
# GROUND TRUTH is the local known-good copy below, never the device — asking the
# device what it should contain is how a reverted file gets blessed as correct.

set -eo pipefail

DEVICE_IP="${1:-172.20.10.2}"
shift || true
CHECK_ONLY=0
for a in "$@"; do
  case "$a" in
    --check|--dry-run) CHECK_ONLY=1 ;;
  esac
done

DEVICE="kommu@${DEVICE_IP}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REF="${BYD_CARSTATE_REF:-$SCRIPT_DIR/../Modified_Kommu/Odom_ROS/09_09/device-side/carstate.py}"
LIVE=/data/openpilot/opendbc_repo/opendbc/car/byd/cam_lka/carstate.py
REBOOT_TIMEOUT=180          # seconds to wait for the device to come back
VISION_TIMEOUT=180          # seconds to wait for camera/model/params to go valid

[ -f "$REF" ] || { echo "[yawcheck] known-good copy missing: $REF" >&2; exit 1; }
REF_MD5="$(md5sum "$REF" | cut -d' ' -f1)"

LIVE_MD5="$(ssh -o ConnectTimeout=10 "$DEVICE" "md5sum $LIVE 2>/dev/null | cut -d' ' -f1" || true)"
if [ -z "$LIVE_MD5" ]; then
  echo "[yawcheck] could not read $LIVE on $DEVICE_IP — is the device up?" >&2
  exit 1
fi

if [ "$LIVE_MD5" == "$REF_MD5" ]; then
  echo "[yawcheck] carstate.py OK — YAW_SENSOR patch intact (${REF_MD5:0:8})"
  exit 0
fi

echo "[yawcheck] ================================================================"
echo "[yawcheck] carstate.py MISMATCH"
echo "[yawcheck]   live      ${LIVE_MD5:0:8}"
echo "[yawcheck]   expected  ${REF_MD5:0:8}"
echo "[yawcheck] Almost certainly the daily auto-update reset: carstate.py is a"
echo "[yawcheck] TRACKED file, so 'git checkout --force' reclaims it and CS.yawRate"
echo "[yawcheck] silently reverts to 0.0."
if [ "$CHECK_ONLY" == "1" ]; then
  echo "[yawcheck] --check given: not redeploying. Run without --check to heal."
  exit 2
fi
echo "[yawcheck] Redeploying and REBOOTING. Measured at ~40-60 s including the"
echo "[yawcheck] vision check — do not start driving until this finishes."
echo "[yawcheck] ================================================================"

STAMP="$(date +%Y%m%d_%H%M%S)"
ssh "$DEVICE" "cp $LIVE ${LIVE}.bak.${STAMP} && echo '[yawcheck] backed up to ${LIVE}.bak.${STAMP}'"
scp -q "$REF" "${DEVICE}:${LIVE}"
ssh "$DEVICE" "cd \$(dirname $LIVE) && /usr/local/venv/bin/python3 -c 'import ast,io;ast.parse(io.open(\"carstate.py\",encoding=\"utf-8\").read())' && echo '[yawcheck] deployed, syntax OK'"

# A tracked-file edit only takes effect on a FULL REBOOT: a running bukapilot
# caches Python modules at import, and a bare pkill+relaunch has previously left
# the camera/VIPC pipeline stalled.
echo "[yawcheck] rebooting ${DEVICE_IP} ..."
ssh "$DEVICE" 'sudo reboot' || true
sleep 10

echo -n "[yawcheck] waiting for device to come back "
deadline=$(( $(date +%s) + REBOOT_TIMEOUT ))
until ssh -o ConnectTimeout=5 -o BatchMode=yes "$DEVICE" 'true' 2>/dev/null; do
  if [ "$(date +%s)" -ge "$deadline" ]; then
    echo
    echo "[yawcheck] device did not come back within ${REBOOT_TIMEOUT}s." >&2
    echo "[yawcheck] It may still be booting. Check power/network, then re-run." >&2
    exit 1
  fi
  echo -n "."
  sleep 5
done
echo " up"

echo -n "[yawcheck] waiting for manager "
deadline=$(( $(date +%s) + 90 ))
until [ "$(ssh -o ConnectTimeout=5 "$DEVICE" 'pgrep -cf "[m]anager.py" || true' 2>/dev/null)" -ge 1 ] 2>/dev/null; do
  [ "$(date +%s)" -ge "$deadline" ] && { echo; echo "[yawcheck] manager.py did not start." >&2; exit 1; }
  echo -n "."
  sleep 5
done
echo " up"

# Standard post-reboot check. A bare relaunch has stalled the camera before, so
# this must pass before anyone trusts a drive.
echo "[yawcheck] verifying camera/vision ..."
ssh "$DEVICE" "cd /data/openpilot && PYTHONPATH=/data/openpilot /usr/local/venv/bin/python3 -c '
import sys, time
sys.path.insert(0, \"/data/openpilot\")
import cereal.messaging as messaging
KS = [\"roadCameraState\",\"modelV2\",\"liveParameters\"]
sm = messaging.SubMaster(KS)
t0 = time.monotonic()
ok = False
while time.monotonic() - t0 < ${VISION_TIMEOUT}:
    sm.update(1000)
    if all(sm.valid[k] for k in KS):
        ok = True; break
for k in KS:
    print(\"  %-18s valid=%s\" % (k, sm.valid[k]))
sys.exit(0 if ok else 1)
'" || { echo "[yawcheck] vision did NOT come back valid — investigate before driving." >&2; exit 1; }

# The cereal server does not survive a reboot; nothing restarts it automatically.
echo "[yawcheck] restarting cereal server ..."
"$SCRIPT_DIR/byd_ensure_cereal_server.sh" "$DEVICE_IP" >/dev/null || {
  echo "[yawcheck] cereal server failed to start — run byd_ensure_cereal_server.sh by hand." >&2
  exit 1
}
echo "[yawcheck] cereal server live"

# Confirm the heal actually took, and that the RUNNING code is the new code —
# an edit on disk means nothing if the process predates it.
NEW_MD5="$(ssh "$DEVICE" "md5sum $LIVE | cut -d' ' -f1")"
[ "$NEW_MD5" == "$REF_MD5" ] || { echo "[yawcheck] post-deploy md5 STILL wrong (${NEW_MD5:0:8})." >&2; exit 1; }
ssh "$DEVICE" "
  f=\$(stat -c %Y $LIVE)
  m=\$(date -d \"\$(ps -o lstart= -p \$(pgrep -f '[m]anager.py' | head -1))\" +%s)
  if [ \"\$m\" -gt \"\$f\" ]; then
    echo \"[yawcheck] running code is newer than the edit (manager +\$((m-f))s) — live\"
  else
    echo \"[yawcheck] WARNING: manager started BEFORE the edit — not picked up\" >&2; exit 1
  fi
"
echo "[yawcheck] healed and verified (${REF_MD5:0:8})"
