#!/usr/bin/env python3
# Kommu.AI — Manual Steering Hook (device-side logic, Stage 0)
#
# WHAT THIS IS
#   The device-side counterpart to manual_steer_sender.py. The sender is a dumb
#   input device that writes a timestamped target angle to /tmp/manual_steer.json.
#   This module decides, on each carcontroller frame, whether to SUBSTITUTE that
#   manual target for the planner's desired steering angle — and returns the angle
#   that should be fed into the EXISTING create_can_steer_command path.
#
# WHAT THIS IS NOT
#   * It does NOT encode CAN, compute checksums, cycle counters, or open the Panda.
#     All of that stays in bydcan.create_can_steer_command + the CANPacker, unchanged.
#   * It does NOT manufacture engagement. It only acts when the car is ALREADY
#     legitimately lat_active (stock ACC on, LKA latched). If the car is not
#     engaged, this module is a no-op and the stock angle flows through untouched.
#   * It does NOT bypass any limit. The manual angle is returned as a *desired*
#     angle, to be fed through the SAME _compute_apply_angle limiting chain
#     (lowpass -> apply_std_steer_angle_limits -> +/-10 deg meas clamp -> 120 cap)
#     that the planner angle goes through. See integration note at bottom.
#
# GATING (all must hold for the manual angle to be used):
#   1. Enable file /tmp/MANUAL_STEER_ENABLE present   (rm = instant kill -> stock)
#      NOTE: as of the writer's connection-lifecycle automation, this file
#      means ONLY "a session is live" (a sender is connected). It does NOT by
#      itself mean the driver wants to override — see gate 2.5.
#   2. /tmp/manual_steer.json exists, parses, and is FRESH (ts within STALE_S)
#   2.5. The target's "active" flag is True. This is the driver's explicit
#      override toggle (key M on the sender), separate from mere connection.
#      Missing/False/unparseable -> treated exactly like no session at all:
#      the planner angle passes through untouched. This exists specifically so
#      a connected-but-idle (or connected-but-not-toggled-on) sender at some
#      stale/idle target (e.g. 0.0 deg) can never fight bukapilot's own
#      lane-centering just by virtue of being connected.
#   3. The car is genuinely lat_active this frame (passed in by caller)
#   4. Vehicle speed >= SPEED_FLOOR_MS (EPS rejects standstill angle cmds; do not
#      fight that gate — refuse to command below the floor)
#
# If any gate fails -> return (planner_angle, lat_active) UNCHANGED (no-op).
#
# The returned desired-angle is still subject to ALL downstream limits, so even a
# bad/hostile file can only walk the wheel at the rate limit within +/-10 deg of
# the current measured angle. This module cannot snap the wheel.

import json
import os
import time

ENABLE_FILE = "/tmp/MANUAL_STEER_ENABLE"
TARGET_FILE = "/tmp/manual_steer.json"

# Freshness: sender writes at 50 Hz (20 ms). Anything older than this = stale.
STALE_S = 0.20          # 200 ms -> ~10 missed writes; fall back to planner
# Refuse to command manual steering below this speed (m/s). EPS rejects
# standstill ADAS angle commands by design; do not attempt to defeat it.
# NOTE: 0.30 is a CONSERVATIVE PLACEHOLDER, not a measured EPS threshold. The
# real minimum speed at which the Dolphin EPS accepts ADAS angle commands is
# unknown and can only be found at Stage 3 (moving, developer present). Until
# then this floor exists only to guarantee the tool never commands at a dead
# stop; it is intentionally low and belt-and-suspenders with the carcontroller's
# own `not CS.out.standstill` gate in lat_active. Do not treat 0.30 as validated.
SPEED_FLOOR_MS = 0.30
# Two-tier caps — third (innermost) enforcement gate.
# NORMAL: 45 deg — 25 deg margin to Panda max_angle 70 deg (byd.h max_angle=700 raw).
# HIGH:   60 deg — 10 deg margin to Panda max_angle 70 deg. Deliberate intermediate
#         step: validate 45 before stepping HIGH toward ceiling, preserving
#         >=10 deg margin before the 70 deg ceiling.
# HIGH applies only when JSON has active=True AND high_range=True.
# Any parse failure or missing high_range field defaults to NORMAL (fail-safe).
MANUAL_ABS_MAX_DEG_NORMAL = 45.0
MANUAL_ABS_MAX_DEG_HIGH   = 60.0
MANUAL_ABS_MAX_DEG = MANUAL_ABS_MAX_DEG_NORMAL  # default arg for function signature


def _read_target(now, target_file=TARGET_FILE, stale_s=STALE_S):
  """Return a fresh (angle, active, high_range) 3-tuple, or None if missing/stale/invalid.

  `active` is the driver's explicit override toggle (gate 2.5). `high_range`
  selects the cap tier: False -> NORMAL (45 deg), True -> HIGH (60 deg).
  high_range missing from JSON defaults to False (fail-safe NORMAL mode).
  high_range can never be True when active is False; the active guard below
  enforces this even if sender/writer upstream failed to.
  Any parse failure returns None — same fail-safe shape as before.
  """
  try:
    with open(target_file, "r") as f:
      payload = json.load(f)
    ts = float(payload["ts"])
    angle = float(payload["angle_deg"])
    active = bool(payload["active"])
    # Missing key -> False (older writer / partial file). Also enforce active guard.
    high_range = bool(payload.get("high_range", False)) and active
  except (OSError, ValueError, KeyError, TypeError):
    return None
  if now - ts > stale_s:
    return None            # stale -> fall back to planner
  if now - ts < -1.0:
    return None            # clock skew / future timestamp -> distrust
  return angle, active, high_range


def _clamp(v, lo, hi):
  return lo if v < lo else hi if v > hi else v


def manual_desired_angle(planner_angle, lat_active, v_ego, *,
                         now=None,
                         enable_file=ENABLE_FILE,
                         target_file=TARGET_FILE,
                         stale_s=STALE_S,
                         speed_floor_ms=SPEED_FLOOR_MS,
                         abs_max_deg=MANUAL_ABS_MAX_DEG):
  """Decide the desired steering angle for this frame.

  Returns (desired_angle, use_manual):
    desired_angle : the angle to feed into the EXISTING limit chain / encoder.
    use_manual    : True if the manual target was substituted, else False.

  On any gate failure this returns (planner_angle, False) — a pure no-op that
  preserves stock behaviour. The caller feeds desired_angle through the same
  _compute_apply_angle limiting the planner angle would have received.
  """
  if now is None:
    now = time.time()

  # Gate 1: must be legitimately engaged. Never manufacture engagement.
  if not lat_active:
    return planner_angle, False

  # Gate 2: enable file present (dead-man / instant kill via rm).
  if not os.path.exists(enable_file):
    return planner_angle, False

  # Gate 3: speed floor — respect EPS standstill rejection.
  if v_ego < speed_floor_ms:
    return planner_angle, False

  # Gate 4: fresh, valid manual target.
  result = _read_target(now, target_file=target_file, stale_s=stale_s)
  if result is None:
    return planner_angle, False
  angle, active, high_range = result

  # Gate 2.5: driver's explicit override toggle. A connected-but-not-toggled
  # (or stale-field) session behaves identically to no session at all — the
  # enable file alone is no longer sufficient to substitute.
  if not active:
    return planner_angle, False

  # Select cap tier. HIGH only when both active and high_range are True —
  # high_range=True with active=False is already filtered by _read_target,
  # but the check above provides an additional barrier. Falls back to
  # abs_max_deg (NORMAL, 20 deg) for any missing/False/fail-safe case.
  cap = MANUAL_ABS_MAX_DEG_HIGH if high_range else abs_max_deg
  angle = _clamp(float(angle), -cap, cap)
  return angle, True


# ── Integration note (how the carcontroller actually uses this) ────────────────
#
# The manual angle goes THROUGH the same limiter the planner angle uses
# (lowpass -> apply_std_steer_angle_limits -> +/-10deg meas clamp -> 120 cap),
# so it can never snap the wheel. This is done WITHOUT mutating the capnp
# actuators struct (which would be fragile: as_builder()/as_reader() mid-update
# can silently no-op, leaving you on stock steering while believing manual is
# active) via ONE explicit optional parameter on _compute_apply_angle.
#
# This exact design is implemented in
# claude/carcontroller_desired_override_cam_lka.patch, applied directly to
# _compute_apply_angle and update() in
# opendbc/car/byd/carcontroller.py:
#
#   def _compute_apply_angle(self, CS, actuators, lat_active, desired_override=None):
#       ...
#       desired_in = actuators.steeringAngleDeg if desired_override is None else desired_override
#       after_lowpass = lowpass_1pole(desired_in, self.last_apply_angle)
#       ... rest unchanged ...
#
#   # in update(), just before the create_can_steer_command call:
#   desired_override = None
#   manual_ang, use_manual = manual_desired_angle(actuators.steeringAngleDeg, lat_active, CS.out.vEgo)
#   if use_manual:
#       desired_override = manual_ang          # explicit, no struct mutation
#   apply_angle, steer_angle_limited = self._compute_apply_angle(CS, actuators, lat_active, desired_override)
#   ... create_can_steer_command(self.packer, apply_angle, lat_active, ...)  # unchanged
#
# Why this is safe:
#   * desired_override=None  -> byte-for-byte identical to stock behaviour.
#   * No capnp mutation, so no silent-no-op failure mode.
#   * The manual angle is a DESIRED input; every downstream limit still applies.
#   * lat_active is passed through unchanged — the hook never manufactures it.
#   * Everything below _compute_apply_angle (6-arg encoder, packer checksum,
#     counter, bus 0, Panda relay gate) is exactly as Kommu wrote it.
#
# Status: the patch above has been reviewed, dry-run/applied against a scratch
# copy of the real on-device staging_v2 carcontroller.py (py_compile clean,
# zero fuzz/rejects), and approved for deployment. This hook itself still only
# activates when /tmp/MANUAL_STEER_ENABLE is explicitly present — see GATING
# above — so landing both files on-device does not, by itself, change any
# driving behaviour.
