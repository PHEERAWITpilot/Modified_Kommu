#!/bin/bash
# Pre-deploy check for the BYD longitudinal ACC_CMD capture harness.
# Run from the repo root:  bash claude/tests/run_all.sh
# All four suites must pass before anything is scp'd to the device.
set -u
cd "$(dirname "$0")/../.." || exit 1
fail=0
run() { printf '%-26s' "$1"; shift; if "$@" >/tmp/kommu_test_out 2>&1; then echo "PASS"; else echo "FAIL"; tail -25 /tmp/kommu_test_out; fail=1; fi; }

run "gate self-test"        python3 claude/byd_longitudinal_kill_gate.py
run "decoder round-trip"    bash -c 'cd bumpbump_clone && python3 ../claude/byd_acc_cmd_decode.py --selftest'
run "carcontroller gating"  bash -c 'cd bumpbump_clone && python3 ../claude/tests/test_longitudinal_gate.py'
run "capture session"       python3 claude/tests/test_capture_session.py
echo
[ $fail -eq 0 ] && echo "ALL PASS — safe to deploy" || echo "FAILURES — do NOT deploy"
exit $fail
