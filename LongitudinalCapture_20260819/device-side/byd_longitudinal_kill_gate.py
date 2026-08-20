#!/usr/bin/env python3
"""
byd_longitudinal_kill_gate.py

TX gate for the BYD ACC_CMD (0x32E / 814) longitudinal capture harness.

DEPLOY: scp to /data/openpilot/opendbc_repo/opendbc/car/byd/
        (same directory manual_steer_hook.py occupies)
IMPORT: opendbc.car.byd.byd_longitudinal_kill_gate

Imported by cam_lka/carcontroller.py at the point ACC_CMD is about to be
appended to can_sends. Same mental model as manual_desired_angle() for
steering: nothing arms persistently, every single frame re-asks "is it still
OK to send this." No state is carried between frames.

WHY THIS EXISTS
  openpilotLongitudinalControl and BYD_OP_LONG_PLATFORMS are static config,
  read once at process start. Without this gate the only way to stop 0x32E TX
  is to edit two files and reboot. The 07-27 experiment (CONTEXT.md) showed
  0x32E injection collides with the car's own ~50Hz ACC_CMD transmitter on
  bus 0 and drives the controller toward bus-off, so a runtime kill path that
  works in milliseconds -- not a reboot -- is mandatory before that config is
  ever enabled on a real car.

GATE STACK (ALL must pass to allow TX)
  1. /tmp/LONGITUDINAL_KILL must NOT exist. Existence alone is the signal --
     nothing to parse, nothing to go stale. Checked FIRST and independently of
     everything else, so it still works if the monitor has hung or died
     without reaching its cleanup.
  2. /tmp/LONGITUDINAL_CAPTURE_ENABLE must exist. Owned solely by
     byd_long_conflict_capture.py --capture, which creates it only after the
     operator types the confirmation phrase and removes it on every exit path.
  3. /tmp/longitudinal_health.json must be fresher than HEALTH_MAX_AGE_S and
     report an all-clear bus-0 CAN controller: TEC == 0 and none of
     errorWarning / errorPassive / busOff set.

  4. OPTIONAL, only if /tmp/LONGITUDINAL_ENGAGED_ONLY exists: the car must be
     engaged this frame. This is the "inject only at the engage moment"
     experiment -- instead of transmitting continuously from process start
     (which is what caused the 07-27 collision within seconds of boot), TX is
     confined to the engaged window. Strictly narrows exposure; it can only
     ever suppress frames, never add any. Bonus property: because braking
     drops engagement, in this mode the brake pedal is a DIRECT TX kill.

  Gate 3 is deliberately maximally conservative. The classic 128 error-passive
  threshold was NEVER measured on this platform -- 07-27 logged TEC as a raw
  gauge and recorded onset as "first time TEC leaves 0". So TEC leaving 0 IS
  the abort, not some fraction of a threshold nobody observed here.

FAIL-CLOSED, ALWAYS
  Every error path -- missing file, stale file, bad JSON, missing key, OSError,
  anything unexpected -- returns False. There is no code path in this module
  that returns True as a fallback. Note this INVERTS the steering hook's
  convention: manual_steer_hook falls back to stock behaviour because stock
  lateral is the safe state. Here the safe state is "do not transmit 0x32E at
  all", so falling back to stock (transmitting) would be exactly backwards.

SHADOW MODE
  longitudinal_shadow_enabled() + shadow_log() capture the fully-engaged
  payload WITHOUT transmitting anything. create_accel_command() packs
  identical bytes whether or not the result is appended to can_sends, so the
  primary deliverable of this harness -- the engaged ACC_CMD bit pattern --
  needs zero bus traffic. Shadow mode is the DEFAULT posture; opening the TX
  gate is a separate, explicitly flagged run.

/tmp is wiped on every device reboot, so all three gate files fail safe across
reboots by construction -- there is no way to leave this armed accidentally.
"""

import json
import os
import time

ENABLE_FILE = "/tmp/LONGITUDINAL_CAPTURE_ENABLE"
KILL_FILE = "/tmp/LONGITUDINAL_KILL"
HEALTH_FILE = "/tmp/longitudinal_health.json"
SHADOW_FILE = "/tmp/LONGITUDINAL_SHADOW"
ENGAGED_ONLY_FILE = "/tmp/LONGITUDINAL_ENGAGED_ONLY"
SHADOW_LOG = "/tmp/longitudinal_shadow.jsonl"

# The health file is produced from pandaStates, which publishes at 10Hz.
# 400ms == four missed samples: loose enough not to flap on ordinary jitter,
# far tighter than the 2-21s error onset window observed on 07-27.
HEALTH_MAX_AGE_S = 0.4

# Bus-0 CAN controller flags that must all be clear. Names match
# cereal PandaCanState (log.capnp) as re-published by the capture script.
_FLAGS = ("bus_off", "error_passive", "error_warning")


def longitudinal_tx_allowed(engaged=None, health_file=None, enable_file=None,
                            kill_file=None, engaged_only_file=None,
                            max_age_s=None, now=None):
  """True only if every gate passes. Any doubt whatsoever -> False.

  `engaged` is the car's current engagement state (carcontroller passes
  CC.enabled). When /tmp/LONGITUDINAL_ENGAGED_ONLY exists, TX is additionally
  restricted to frames where engaged is True -- the "inject only at the engage
  moment" experiment, instead of injecting continuously from process start.
  Passing engaged=None with that file present is treated as "not proven
  engaged" and blocks TX, same fail-closed rule as everywhere else.

  Paths default to the module constants, resolved at CALL time rather than as
  default-argument values (which Python would freeze at def time) so tests can
  redirect them at the module level.
  """
  health_file = HEALTH_FILE if health_file is None else health_file
  enable_file = ENABLE_FILE if enable_file is None else enable_file
  kill_file = KILL_FILE if kill_file is None else kill_file
  engaged_only_file = ENGAGED_ONLY_FILE if engaged_only_file is None else engaged_only_file
  max_age_s = HEALTH_MAX_AGE_S if max_age_s is None else max_age_s
  try:
    # 1. manual kill -- checked first, independent of all other state
    if os.path.exists(kill_file):
      return False

    # 1b. engaged-only mode: suppress every frame outside the engaged window.
    # Side effect worth knowing: the brake pedal drops engagement, so in this
    # mode the brake becomes a DIRECT TX kill, not just a cruise kill.
    if os.path.exists(engaged_only_file) and not engaged:
      return False

    # 2. a capture session must be live
    if not os.path.exists(enable_file):
      return False

    # 3. bus-0 health must be fresh AND pristine
    st = os.stat(health_file)
    now = time.time() if now is None else now
    if (now - st.st_mtime) > max_age_s:
      return False  # monitor isn't watching -> nobody is guarding this TX

    with open(health_file) as f:
      health = json.load(f)

    tec = health.get("transmit_error_cnt")
    if tec is None or tec != 0:
      return False

    for flag in _FLAGS:
      # missing key is treated as "not proven clear" -> closed
      if health.get(flag, True):
        return False

  except Exception:
    return False  # fail closed on literally anything

  return True


def longitudinal_shadow_enabled(shadow_file=None):
  """True if the frame should be computed and logged but NEVER transmitted."""
  try:
    return os.path.exists(SHADOW_FILE if shadow_file is None else shadow_file)
  except Exception:
    return False


def shadow_log(acc_msg, accel, long_active, brake_hold, cc_enabled, v_ego,
               path=None):
  """Append one would-have-been-sent ACC_CMD frame to the shadow log.

  acc_msg is the (addr, dat, bus) tuple returned by packer.make_can_msg.
  Called from the control loop, so it must NEVER raise -- a logging failure
  must not take down steering.
  """
  try:
    path = SHADOW_LOG if path is None else path
    addr, dat, bus = acc_msg
    rec = {
      "t": time.time(),
      "addr": addr,
      "hex": bytes(dat).hex(),
      "bus": bus,
      "accel": round(float(accel), 4),
      "long_active": bool(long_active),
      "brake_hold": bool(brake_hold),
      "cc_enabled": bool(cc_enabled),
      "v_ego": round(float(v_ego), 3),
    }
    with open(path, "a") as f:
      f.write(json.dumps(rec) + "\n")
  except Exception:
    pass  # never propagate into the control loop


# ---------------------------------------------------------------------------
# Self-test (Stage 0): runs on the laptop, no car, no cereal, no device.
#   python3 claude/byd_longitudinal_kill_gate.py
# ---------------------------------------------------------------------------
def _selftest():
  import tempfile
  import traceback

  passed = failed = 0

  def check(desc, got, want):
    nonlocal passed, failed
    if got == want:
      passed += 1
      print(f"  PASS  {desc}  -> {got}")
    else:
      failed += 1
      print(f"  FAIL  {desc}  -> got {got}, want {want}")

  d = tempfile.mkdtemp(prefix="longgate_")
  enable = os.path.join(d, "ENABLE")
  kill = os.path.join(d, "KILL")
  health = os.path.join(d, "health.json")

  def write_health(**over):
    data = {"transmit_error_cnt": 0, "receive_error_cnt": 0,
            "bus_off": False, "error_passive": False, "error_warning": False,
            "safety_tx_blocked": 0, "ts": time.time()}
    data.update(over)
    with open(health, "w") as f:
      json.dump(data, f)

  def allowed(**kw):
    return longitudinal_tx_allowed(health_file=health, enable_file=enable,
                                   kill_file=kill, **kw)

  print("byd_longitudinal_kill_gate self-test")
  print(f"  tmp dir: {d}\n")

  # --- everything absent -------------------------------------------------
  check("no files at all", allowed(), False)

  # --- enable file alone is not enough -----------------------------------
  open(enable, "w").close()
  check("enable only, no health file", allowed(), False)

  # --- the one and only True case ----------------------------------------
  write_health()
  check("enable + fresh clean health (THE allow case)", allowed(), True)

  # --- kill file overrides everything ------------------------------------
  open(kill, "w").close()
  check("kill file present, all else fine", allowed(), False)
  os.remove(kill)
  check("kill file removed again", allowed(), True)

  # --- enable file removed ------------------------------------------------
  os.remove(enable)
  check("enable removed, health still clean", allowed(), False)
  open(enable, "w").close()

  # --- staleness ----------------------------------------------------------
  write_health()
  check("health 0.39s old (fresh)", allowed(now=time.time() + 0.39), True)
  check("health 0.41s old (stale)", allowed(now=time.time() + 0.41), False)
  check("health 10s old (very stale)", allowed(now=time.time() + 10), False)

  # --- each error condition trips independently ---------------------------
  write_health(transmit_error_cnt=1)
  check("TEC == 1 (first nonzero)", allowed(), False)
  write_health(transmit_error_cnt=255)
  check("TEC == 255", allowed(), False)
  for flag in _FLAGS:
    write_health(**{flag: True})
    check(f"{flag} set", allowed(), False)

  # --- malformed / missing data --------------------------------------------
  write_health()
  check("sanity: clean again", allowed(), True)

  with open(health, "w") as f:
    f.write("{not json at all")
  check("corrupt JSON", allowed(), False)

  with open(health, "w") as f:
    json.dump({"ts": time.time()}, f)
  check("JSON missing every key", allowed(), False)

  with open(health, "w") as f:
    json.dump({"transmit_error_cnt": 0, "ts": time.time()}, f)
  check("TEC present but flags missing", allowed(), False)

  with open(health, "w") as f:
    f.write("")
  check("empty health file", allowed(), False)

  # --- engaged-only mode ---------------------------------------------------
  eo = os.path.join(d, "ENGAGED_ONLY")

  def allowed_eo(**kw):
    return longitudinal_tx_allowed(health_file=health, enable_file=enable,
                                   kill_file=kill, engaged_only_file=eo, **kw)

  write_health()
  check("no engaged-only file: engaged=False still allowed", allowed_eo(engaged=False), True)
  check("no engaged-only file: engaged=None still allowed", allowed_eo(), True)
  open(eo, "w").close()
  check("engaged-only + engaged=True  -> allowed", allowed_eo(engaged=True), True)
  check("engaged-only + engaged=False -> blocked", allowed_eo(engaged=False), False)
  check("engaged-only + engaged=None  -> blocked (fail closed)", allowed_eo(), False)
  check("engaged-only + engaged=True but TEC bad -> blocked",
        (write_health(transmit_error_cnt=3), allowed_eo(engaged=True))[1], False)
  write_health()
  open(kill, "w").close()
  check("engaged-only + engaged=True but kill file -> blocked", allowed_eo(engaged=True), False)
  os.remove(kill)
  os.remove(eo)
  check("engaged-only file removed -> back to normal", allowed_eo(engaged=False), True)

  # --- shadow mode ---------------------------------------------------------
  shadow_marker = os.path.join(d, "SHADOW")
  check("shadow disabled", longitudinal_shadow_enabled(shadow_marker), False)
  open(shadow_marker, "w").close()
  check("shadow enabled", longitudinal_shadow_enabled(shadow_marker), True)

  # --- shadow_log must never raise -----------------------------------------
  log_path = os.path.join(d, "shadow.jsonl")
  shadow_log((0x32E, b"\x64\x19\x19\x00\x00\x10\x00\x00", 0),
             0.0, True, False, True, 0.0, path=log_path)
  with open(log_path) as f:
    rec = json.loads(f.readline())
  check("shadow_log wrote hex", rec["hex"], "6419190000100000")
  check("shadow_log wrote addr", rec["addr"], 0x32E)

  bad = 0
  for args in (
    (None, 0, 0, 0, 0, 0),                      # unpack failure
    ((1, 2), 0, 0, 0, 0, 0),                    # wrong tuple length
    ((0x32E, b"\x00", 0), "nope", 0, 0, 0, 0),  # unfloatable accel
  ):
    try:
      shadow_log(*args, path=os.path.join(d, "junk.jsonl"))
    except Exception:
      bad += 1
      traceback.print_exc()
  check("shadow_log swallowed all bad input", bad, 0)
  try:
    shadow_log((0x32E, b"\x00", 0), 0, 0, 0, 0, 0,
               path="/nonexistent-dir-xyz/shadow.jsonl")
    unwritable_raised = False
  except Exception:
    unwritable_raised = True
  check("shadow_log swallowed unwritable path", unwritable_raised, False)

  # --- the carcontroller fallback stub must also be closed -----------------
  def _fallback():
    return False
  check("carcontroller import-failure fallback", _fallback(), False)

  print(f"\n  {passed} passed, {failed} failed")
  return 0 if failed == 0 else 1


if __name__ == "__main__":
  raise SystemExit(_selftest())
