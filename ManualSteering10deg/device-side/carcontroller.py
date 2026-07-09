import math
import numpy as np

# --- Manual-steer hook (Stage 0/1 validated). Reads /tmp/manual_steer.json,
# gated by /tmp/MANUAL_STEER_ENABLE. Returns (desired_angle, use_manual).
# It NEVER encodes CAN and only substitutes the DESIRED angle input; the frame
# is still built by the existing create_can_steer_command below, and the manual
# angle still passes through the full _compute_apply_angle limiter (lowpass +
# rate limit + +/-10deg-of-measured clamp). If the hook is unavailable for any
# reason, fall back to a no-op so stock behaviour is unchanged. ---
try:
  from opendbc.car.byd.manual_steer_hook import manual_desired_angle
except Exception:
  def manual_desired_angle(planner_angle, lat_active, v_ego, **kw):
    return planner_angle, False

from opendbc.can.packer import CANPacker

from opendbc.car.byd.bydcan import (
  create_accel_command,
  create_can_steer_command,
  create_lkas_hud,
  create_steering_torque_spoof_camera,
  send_buttons,
)
from opendbc.car.byd.values import DBC, CAR, ACCEL_MULT, CANBUS
from opendbc.car.byd.values import CarControllerParams
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_std_steer_angle_limits

STEER_LOWPASS_HZ = 2
STEER_DT = 0.02
MAX_STEER_ANGLE_OFFSET_DEG = 10
# Degrees: warn on HUD when command hits angle safety envelope (meas offset or global max).
STEER_ANGLE_LIMIT_WARN_EPS_DEG = 0.08
LKA_COOLDOWN_MIN_FRAMES = 30
BUTTON_KEEPALIVE_FRAMES = 100
SPOOF_DURATION_FRAMES = 50
SPOOF_CYCLE_FRAMES = 150


def lowpass_1pole(x, y_prev):
  if y_prev is None:
    return x
  alpha = math.exp(-2.0 * math.pi * STEER_LOWPASS_HZ * STEER_DT)
  return alpha * y_prev + (1.0 - alpha) * x


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(DBC[CP.carFingerprint]["pt"])

    self.lka_active = False
    self.last_apply_angle = 0
    self.accel_mult = ACCEL_MULT[CP.carFingerprint]
    self.lka_cooldown = 0
    self.prev_press = False
    self.prev_res_press = False
    self.lka_latched = False
    self.button_send_bus = CANBUS.cam_bus if CP.carFingerprint in (CAR.BYD_ATTO3, CAR.BYD_M6) else CANBUS.main_bus

  def _update_lka_latch_state(self, CS):
    if self.CP.carFingerprint in (CAR.BYD_M6, CAR.BYD_SEAL, CAR.BYD_SHARK, CAR.BYD_SEALION7):
      lkas_rising = CS.lkas_rdy_btn and not self.prev_press
      if lkas_rising:
        if not self.lka_latched:
          self.lka_latched = True
        else:
          self.lka_latched = False
          self.lka_cooldown = 0
          self.lka_active = False
      elif (
        self.CP.carFingerprint == CAR.BYD_SHARK
        and CS.res_btn
        and not self.prev_res_press
        and not self.lka_latched
      ):
        # Shark: stock RES (resume) also arms LKA latch; disengage still via LKAS btn or brake.
        self.lka_latched = True
      self.prev_press = CS.lkas_rdy_btn
      self.prev_res_press = CS.res_btn

      if CS.out.brakePressed:
        self.lka_latched = False
        self.lka_cooldown = 0
        self.lka_active = False
      elif self.lka_latched:
        self.lka_active = True
        self.lka_cooldown += 1
    else:
      if CS.lka_on:
        self.lka_cooldown += 1
        self.lka_active = True
      if not CS.lka_on and CS.lkas_rdy_btn:
        self.lka_active = False
        self.lka_cooldown = 0

  def _compute_apply_angle(self, CS, actuators, lat_active, desired_override=None):
    meas_deg = CS.out.steeringAngleDeg
    if not lat_active:
      return meas_deg, False

    limits = CarControllerParams.ANGLE_LIMITS
    # desired_override, when provided by the manual-steer hook, replaces the
    # planner's desired angle as the INPUT to the identical limiter chain.
    # desired_override=None => byte-for-byte identical to stock behaviour.
    desired_in = actuators.steeringAngleDeg if desired_override is None else desired_override
    after_lowpass = lowpass_1pole(desired_in, self.last_apply_angle)
    after_std = apply_std_steer_angle_limits(
      after_lowpass,
      self.last_apply_angle,
      CS.out.vEgo,
      meas_deg,
      lat_active,
      limits,
    )
    lo = meas_deg - MAX_STEER_ANGLE_OFFSET_DEG
    hi = meas_deg + MAX_STEER_ANGLE_OFFSET_DEG
    apply_angle = float(np.clip(after_std, lo, hi))
    self.last_apply_angle = apply_angle

    eps = STEER_ANGLE_LIMIT_WARN_EPS_DEG
    meas_clip_limited = abs(after_std - apply_angle) > eps
    angle_max_limited = abs(apply_angle) >= limits.STEER_ANGLE_MAX - eps
    steer_angle_limited = meas_clip_limited or angle_max_limited
    return apply_angle, steer_angle_limited

  def update(self, CC, CS, now_nanos):
    del now_nanos
    can_sends = []

    enabled = CC.latActive
    actuators = CC.actuators
    pcm_cancel_cmd = CC.cruiseControl.cancel

    self._update_lka_latch_state(CS)

    lat_active = (self.lka_cooldown > LKA_COOLDOWN_MIN_FRAMES) and enabled and self.lka_active and not CS.out.standstill

    apply_angle = CS.out.steeringAngleDeg
    hand_on_wheel_warning = False
    if (self.frame % 2) == 0:
      # Manual-steer substitution: only ever acts when the car is ALREADY
      # legitimately lat_active (the hook itself also requires that + the
      # enable file + a fresh target + speed floor). Never manufactures
      # engagement. When it does not act, use_manual is False and
      # desired_override stays None => stock path exactly.
      desired_override = None
      manual_ang, use_manual = manual_desired_angle(actuators.steeringAngleDeg, lat_active, CS.out.vEgo)
      if use_manual:
        desired_override = manual_ang
      apply_angle, steer_angle_limited = self._compute_apply_angle(CS, actuators, lat_active, desired_override)
      hand_on_wheel_warning = bool(lat_active and steer_angle_limited)
      can_sends.append(
        create_can_steer_command(
          self.packer,
          apply_angle,
          lat_active,
          CS.out.standstill,
          CS.lkas_healthy,
          CS.lkas_rdy_btn or CS.out.brakePressed,
        )
      )
      can_sends.append(
        create_lkas_hud(
          self.packer,
          lat_active,
          CS.lss_state,
          CS.lss_alert,
          CS.tsr,
          CS.HMA,
          CS.pt2,
          CS.pt3,
          CS.pt4,
          CS.pt5,
          CS.lkas_hud_status_passthrough,
          CS.lka_on,
          hand_on_wheel_warning,
        )
      )

      if self.CP.openpilotLongitudinalControl:
        long_active = CC.enabled and not CS.out.gasPressed
        brake_hold = CS.out.standstill and actuators.accel < 0
        can_sends.append(create_accel_command(self.packer, actuators.accel, long_active, self.accel_mult, brake_hold))
      else:
        if CS.out.standstill and CC.enabled and (self.frame % BUTTON_KEEPALIVE_FRAMES == 0):
          can_sends.append(send_buttons(self.packer, 1, 0, self.button_send_bus))

    if self.CP.carFingerprint in (CAR.BYD_M6, CAR.BYD_SEAL, CAR.BYD_SEALION7, CAR.BYD_SHARK):
      cycle_position = self.frame % SPOOF_CYCLE_FRAMES
      spoof_active = cycle_position < SPOOF_DURATION_FRAMES

      if (self.frame % 5) == 0:
        can_sends.append(
          create_steering_torque_spoof_camera(self.packer, lat_active, CS.out.steeringTorque, spoof_active)
        )

    if pcm_cancel_cmd:
      can_sends.append(send_buttons(self.packer, 0, 1, self.button_send_bus))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = apply_angle

    self.frame += 1
    return new_actuators, can_sends
