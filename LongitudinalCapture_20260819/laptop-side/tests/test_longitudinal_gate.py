#!/usr/bin/env python3
"""
Stage-0/1 pre-deploy test: drives the REAL patched cam_lka CarController.update()
and asserts exactly what lands in can_sends under every gate state.

Run this BEFORE every deploy of the longitudinal capture harness:

    cd bumpbump_clone && python3 ../claude/tests/test_longitudinal_gate.py

The two assertions that matter most, and why:
  * gate closed => NO ACC_CMD *and* NO keepalive buttons. Folding the gate into
    the config `if` in carcontroller.py would fall through to the else branch
    and emit resume-buttons whenever the gate closed — the safety mechanism
    inventing a TX path of its own. That must stay impossible.
  * shadow mode => payloads logged, still zero TX. This is what makes the
    primary capture deliverable reachable without touching the bus.
"""
import inspect
import json
import os
import sys
import tempfile
import time

sys.path.insert(0, '.')
sys.path.insert(0, 'opendbc_repo')

from cereal import car  # noqa: E402
from opendbc.car.byd.values import CAR  # noqa: E402
import opendbc.car.byd.byd_longitudinal_kill_gate as gate  # noqa: E402
import opendbc.car.byd.cam_lka.carcontroller as ccmod  # noqa: E402

ACC_CMD = 0x32E
PCM_BUTTONS = 0x3B0

D = tempfile.mkdtemp(prefix="gateint_")
ENABLE = os.path.join(D, "ENABLE")
KILL = os.path.join(D, "KILL")
HEALTH = os.path.join(D, "health.json")
SHADOW = os.path.join(D, "SHADOW")
SHADOWLOG = os.path.join(D, "shadow.jsonl")

# The gate resolves these at call time (not as default args), so redirecting
# the module constants is enough to point it at a temp dir.
ENGAGED_ONLY = os.path.join(D, "ENGAGED_ONLY")
gate.ENABLE_FILE, gate.KILL_FILE = ENABLE, KILL
gate.HEALTH_FILE, gate.SHADOW_FILE, gate.SHADOW_LOG = HEALTH, SHADOW, SHADOWLOG
gate.ENGAGED_ONLY_FILE = ENGAGED_ONLY

PASS = FAIL = 0


def check(desc, got, want):
    global PASS, FAIL
    if got == want:
        PASS += 1
        print(f"  PASS  {desc}")
    else:
        FAIL += 1
        print(f"  FAIL  {desc}\n          got  {got}\n          want {want}")


def write_health(**over):
    data = {"transmit_error_cnt": 0, "receive_error_cnt": 0, "bus_off": False,
            "error_passive": False, "error_warning": False,
            "safety_tx_blocked": 0, "ts": time.time()}
    data.update(over)
    with open(HEALTH, "w") as f:
        json.dump(data, f)


def rm(*paths):
    for p in paths:
        try:
            os.remove(p)
        except OSError:
            pass


def make_cp(op_long):
    CP = car.CarParams.new_message()
    CP.carFingerprint = CAR.BYD_SEAL
    CP.openpilotLongitudinalControl = op_long
    CP.steerControlType = car.CarParams.SteerControlType.angle
    return CP


def make_cc_cs(standstill, enabled=True):
    CC = car.CarControl.new_message()
    CC.enabled = enabled
    CC.latActive = enabled
    CC.actuators.accel = 1.5
    CC.actuators.steeringAngleDeg = 3.0

    o = car.CarState.new_message()
    o.vEgo = 0.0 if standstill else 12.0
    o.standstill = standstill
    o.gasPressed = False
    o.brakePressed = False
    o.steeringAngleDeg = 3.0
    o.steeringTorque = 0.0

    CC = CC.as_reader()
    o = o.as_reader()

    class CS:
        out = o
        lkas_rdy_btn = False
        res_btn = False
        lka_on = True
        lkas_healthy = True
        lss_state = lss_alert = tsr = HMA = 0
        pt2 = pt3 = pt4 = pt5 = 0
        lkas_hud_status_passthrough = 0
    return CC, CS


_needs_dbc = 'dbc_names' in inspect.signature(ccmod.CarController.__init__).parameters


def run(CP, frames=200, standstill=False, enabled=True):
    """Run N frames; return {addr: count} across all can_sends."""
    cc = ccmod.CarController(dbc_names=None, CP=CP) if _needs_dbc else ccmod.CarController(CP)
    counts = {}
    for _ in range(frames):
        CC, CS = make_cc_cs(standstill, enabled)
        _, sends = cc.update(CC, CS, 0)
        for msg in sends:
            counts[msg[0]] = counts.get(msg[0], 0) + 1
    return counts


print("Stage-0/1: real carcontroller.update() under every gate state")
print(f"  tmp: {D}\n")

print("A. stock config (op_long=False) — the gate must be irrelevant")
cp_off = make_cp(False)
check("no ACC_CMD, moving", ACC_CMD in run(cp_off, standstill=False), False)
a_still = run(cp_off, standstill=True)
check("no ACC_CMD, standstill", ACC_CMD in a_still, False)
check("keepalive buttons DO fire at standstill (stock behaviour intact)",
      a_still.get(PCM_BUTTONS, 0) > 0, True)

print("\nB. op_long=True, but SEAL not in BYD_OP_LONG_PLATFORMS")
cp_on = make_cp(True)
check("still no ACC_CMD (platform tuple also gates it)",
      ACC_CMD in run(cp_on, standstill=True), False)

print("\nC. config flipped (SEAL added to the platform tuple) — gate now decides")
orig = ccmod.BYD_OP_LONG_PLATFORMS
ccmod.BYD_OP_LONG_PLATFORMS = tuple(orig) + (CAR.BYD_SEAL,)
try:
    rm(ENABLE, KILL, HEALTH, SHADOW)
    c = run(cp_on, standstill=True)
    check("gate closed -> NO ACC_CMD", ACC_CMD in c, False)
    check("gate closed -> NO spurious keepalive buttons (the else-branch fix)",
          PCM_BUTTONS in c, False)

    open(ENABLE, "w").close()
    check("enable only, no health -> NO ACC_CMD",
          ACC_CMD in run(cp_on, standstill=True), False)

    write_health()
    c3 = run(cp_on, frames=200, standstill=False)
    check("gate OPEN -> ACC_CMD transmitted", c3.get(ACC_CMD, 0) > 0, True)
    check("gate OPEN -> ACC_CMD at 50Hz (100 of 200 frames)", c3.get(ACC_CMD), 100)

    open(KILL, "w").close()
    c4 = run(cp_on, standstill=False)
    check("kill file -> NO ACC_CMD", ACC_CMD in c4, False)
    check("kill file -> no keepalive buttons either", PCM_BUTTONS in c4, False)
    rm(KILL)

    write_health(transmit_error_cnt=1)
    check("TEC=1 -> NO ACC_CMD", ACC_CMD in run(cp_on, standstill=False), False)

    for flag in ("bus_off", "error_passive", "error_warning"):
        write_health(**{flag: True})
        check(f"{flag} -> NO ACC_CMD", ACC_CMD in run(cp_on, standstill=False), False)

    write_health()
    os.utime(HEALTH, (time.time() - 5, time.time() - 5))
    check("health 5s stale -> NO ACC_CMD", ACC_CMD in run(cp_on, standstill=False), False)

    print("\nD. engaged-only mode — inject at the engage moment, not from boot")
    rm(KILL, SHADOW, SHADOWLOG, ENGAGED_ONLY)
    open(ENABLE, "w").close()
    write_health()
    # baseline: without the file, disengaged frames still transmit (the old
    # continuous behaviour, i.e. what collided within seconds on 07-27)
    e0 = run(cp_on, frames=200, standstill=False, enabled=False)
    check("continuous mode: TX even while DISENGAGED", e0.get(ACC_CMD, 0), 100)

    open(ENGAGED_ONLY, "w").close()
    e1 = run(cp_on, frames=200, standstill=False, enabled=False)
    check("engaged-only: disengaged => NO TX at all", ACC_CMD in e1, False)
    check("engaged-only: disengaged => no keepalive buttons either",
          PCM_BUTTONS in e1, False)
    e2 = run(cp_on, frames=200, standstill=False, enabled=True)
    check("engaged-only: engaged => TX flows at 50Hz", e2.get(ACC_CMD, 0), 100)

    # the other gates still dominate even when engaged
    open(KILL, "w").close()
    check("engaged-only: kill file still wins over engaged=True",
          ACC_CMD in run(cp_on, standstill=False, enabled=True), False)
    rm(KILL)
    write_health(transmit_error_cnt=1)
    check("engaged-only: TEC=1 still wins over engaged=True",
          ACC_CMD in run(cp_on, standstill=False, enabled=True), False)
    write_health()
    rm(ENGAGED_ONLY)

    print("\nE. shadow mode — payload captured, zero TX")
    rm(ENABLE, HEALTH, SHADOWLOG)
    open(SHADOW, "w").close()
    d1 = run(cp_on, frames=300, standstill=False)
    check("shadow -> still NO ACC_CMD on the wire", ACC_CMD in d1, False)
    check("shadow -> still no keepalive buttons", PCM_BUTTONS in d1, False)
    recs = [json.loads(x) for x in open(SHADOWLOG)] if os.path.exists(SHADOWLOG) else []
    check("shadow -> payloads were logged", len(recs) > 0, True)
    if recs:
        r = recs[0]
        check("shadow record addr is 0x32E", r["addr"], ACC_CMD)
        check("shadow record has 8 bytes", len(r["hex"]), 16)
        check("shadow record marks engaged", r["long_active"], True)
        check("shadow logged at 2Hz (6 per 300 frames)", len(recs), 6)
        print(f"        sample payload: {r['hex']}  accel={r['accel']}")
finally:
    ccmod.BYD_OP_LONG_PLATFORMS = orig

print(f"\n  {PASS} passed, {FAIL} failed")
sys.exit(0 if FAIL == 0 else 1)
