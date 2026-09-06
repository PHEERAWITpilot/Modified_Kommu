#!/usr/bin/env bash
# byd_ensure_cereal_server.sh — verify/restore/start the device cereal server
# before launching ROS.
#
# Restores from /data/kommu_tools/ (which survives the updater's finalized-tree
# swap) rather than re-scp'ing from the laptop, per the 2026-08-25 diagnostic:
# launch_chffrplus.sh replaces /data/openpilot wholesale with the finalized
# tree, so EVERY untracked file deployed there is lost on an update swap.
# /data/kommu_tools is outside that blast radius.
#
# Idempotent: safe to run repeatedly; will not start a second server instance.
set -euo pipefail
DEVICE_IP="${1:-172.20.10.3}"
DEVICE="kommu@${DEVICE_IP}"
MASTER=/data/kommu_tools/byd_cereal_server.py
LIVE=/data/openpilot/byd_cereal_server.py

# NOTE: the pgrep pattern MUST be bracket-escaped. `pgrep -f byd_cereal_server.py`
# run over ssh matches the remote `bash -c` wrapper carrying that same string in
# its command line, so it always reports "running" and the server never starts.
# Verified on-device 2026-08-25.
PGREP_PAT='[b]yd_cereal_server\.py'

echo "[ensure-cereal] checking device ${DEVICE_IP}..."

if ! ssh "$DEVICE" "test -f $MASTER"; then
    echo "[ensure-cereal] FATAL: master copy missing at $MASTER — seed it first with:" >&2
    echo "    scp ~/Desktop/Kommu.AI/claude/byd_cereal_server.py ${DEVICE}:${MASTER}" >&2
    exit 1
fi

if ssh "$DEVICE" "test -f $LIVE"; then
    echo "[ensure-cereal] $LIVE present"
else
    echo "[ensure-cereal] $LIVE missing (likely wiped by updater swap) — restoring from kommu_tools"
    ssh "$DEVICE" "cp $MASTER $LIVE"
fi

if ssh "$DEVICE" "pgrep -f '$PGREP_PAT' > /dev/null"; then
    echo "[ensure-cereal] server already running — leaving it alone"
else
    echo "[ensure-cereal] server not running — starting"
    ssh "$DEVICE" "cd /data/openpilot && screen -dmS cereal bash -c 'PYTHONPATH=/data/kommu_tools/pylibs:/data/openpilot /usr/local/venv/bin/python3 -u byd_cereal_server.py > /tmp/cereal_server.log 2>&1'"
    sleep 2
fi

echo "[ensure-cereal] verifying stream responds..."
RESULT=$(ssh "$DEVICE" "timeout 3 nc localhost 5556 2>/dev/null | head -1" || true)
if [[ -z "$RESULT" ]]; then
    echo "[ensure-cereal] FATAL: server did not respond after start attempt" >&2
    echo "----- /tmp/cereal_server.log -----" >&2
    ssh "$DEVICE" "cat /tmp/cereal_server.log" >&2 || true
    exit 1
fi
if [[ "$RESULT" != *"yaw_rate"* ]]; then
    echo "[ensure-cereal] WARNING: stream responded but 'yaw_rate' not in payload." >&2
    echo "[ensure-cereal] The measured track will hold heading. Check ${LIVE} against ${MASTER}." >&2
fi

echo "[ensure-cereal] OK — cereal server confirmed live on ${DEVICE_IP}:5556"
