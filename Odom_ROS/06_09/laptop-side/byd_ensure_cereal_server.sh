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
DEVICE_IP="${1:-172.20.10.2}"
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

# ---------------------------------------------------------------------------
# STALENESS: "running" is NOT the same as "running the right version".
# Two ways a stale server survives, both observed on 2026-09-05:
#   1. $LIVE exists but is an OLDER build than $MASTER. Checking only for
#      existence reported OK while serving a version missing yaw_sensor_rate.
#   2. $LIVE is current but the PROCESS started before the file was updated.
#      Python caches modules at import, so the running server keeps serving the
#      old code even though the file on disk is right.
# Both are detected below and fixed by restarting. A restart costs ~2s of
# stream, which is far cheaper than silently capturing a dataset with a
# missing field.
# ---------------------------------------------------------------------------
NEED_RESTART=0

if ssh "$DEVICE" "test -f $LIVE"; then
    LIVE_MD5="$(ssh "$DEVICE" "md5sum $LIVE | cut -d' ' -f1")"
    MASTER_MD5="$(ssh "$DEVICE" "md5sum $MASTER | cut -d' ' -f1")"
    if [[ "$LIVE_MD5" == "$MASTER_MD5" ]]; then
        echo "[ensure-cereal] $LIVE present and matches master (${LIVE_MD5:0:8})"
    else
        echo "[ensure-cereal] STALE: $LIVE (${LIVE_MD5:0:8}) != master (${MASTER_MD5:0:8})"
        echo "[ensure-cereal] refreshing from master and restarting"
        ssh "$DEVICE" "cp $MASTER $LIVE"
        NEED_RESTART=1
    fi
else
    echo "[ensure-cereal] $LIVE missing (likely wiped by updater swap) — restoring from kommu_tools"
    ssh "$DEVICE" "cp $MASTER $LIVE"
    NEED_RESTART=1
fi

if ssh "$DEVICE" "pgrep -f '$PGREP_PAT' > /dev/null"; then
    if [[ "$NEED_RESTART" == "0" ]]; then
        # File is current — but is the RUNNING process older than it?
        # Compare process start time against file mtime, both device-side.
        AGE_CHECK="$(ssh "$DEVICE" "
            pid=\$(pgrep -f '$PGREP_PAT' | head -1)
            [ -z \"\$pid\" ] && echo unknown && exit 0
            pstart=\$(stat -c %Y /proc/\$pid 2>/dev/null || echo 0)
            fmtime=\$(stat -c %Y $LIVE 2>/dev/null || echo 0)
            if [ \"\$fmtime\" -gt \"\$pstart\" ]; then echo stale; else echo fresh; fi
        " 2>/dev/null || echo unknown)"
        if [[ "$AGE_CHECK" == "stale" ]]; then
            echo "[ensure-cereal] STALE: running process predates $LIVE — Python cached the"
            echo "[ensure-cereal] old module at import. Restarting."
            NEED_RESTART=1
        fi
    fi

    if [[ "$NEED_RESTART" == "1" ]]; then
        echo "[ensure-cereal] stopping the stale server (~2s stream gap)"
        ssh "$DEVICE" "pkill -f '$PGREP_PAT'" || true
        sleep 1
        ssh "$DEVICE" "screen -wipe >/dev/null 2>&1" || true
    else
        echo "[ensure-cereal] server already running and current — leaving it alone"
    fi
fi

if ! ssh "$DEVICE" "pgrep -f '$PGREP_PAT' > /dev/null"; then
    echo "[ensure-cereal] starting server"
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
