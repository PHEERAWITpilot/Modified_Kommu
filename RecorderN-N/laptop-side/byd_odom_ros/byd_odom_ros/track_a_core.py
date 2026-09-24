"""Track A dead-reckoning core, shared by odom_node.py and offline consumers.

Track A = heading from the car's own yaw-rate sensor (CAN 546 YAW_SENSOR) with
the ECU's per-sample YAW_OFFSET subtracted, position from wheel speed.

Stdlib `math` only: no ROS, no numpy, no node-instance state. Every function
takes its inputs explicitly so the same code runs inside the live node, on the
device, and in offline tools, and cannot drift between them.

Input validation lives in ONE place, sanitize_can_sample(), at the boundary.
The integration math below it is deliberately unguarded so it stays
bit-identical to every recording made before this module existed.
"""

import math

# Below this speed the car is treated as stationary: speed is zeroed and the
# yaw rate is zeroed, so standstill sensor noise never integrates into drift.
MIN_SPEED_MS = 0.05
# A sample whose interval to the previous one exceeds this is not integrated.
DT_MAX_S = 1.0
# A sample older than this (receive time vs now) is skipped entirely.
STALE_S = 0.5


class Integrator:
    """One independent x/y/yaw dead-reckoning track."""

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

    def step(self, v_ms: float, yaw_rate: float, dt: float):
        self.yaw += yaw_rate * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        self.x += v_ms * math.cos(self.yaw) * dt
        self.y += v_ms * math.sin(self.yaw) * dt

    def step_absolute_heading(self, v_ms: float, heading_rad: float, dt: float):
        """Position integration with heading SET, not accumulated.

        Used only by the `steerproxy` track. Everything else calls step(), which
        integrates a yaw RATE. Here the heading is overwritten each tick with an
        absolute value, so no history is carried. x/y integration is otherwise
        identical, so the two tracks differ in exactly one respect.
        """
        self.yaw = math.atan2(math.sin(heading_rad), math.cos(heading_rad))
        self.x += v_ms * math.cos(self.yaw) * dt
        self.y += v_ms * math.sin(self.yaw) * dt


def is_stale(now: float, sample_t: float, stale_s: float = STALE_S) -> bool:
    """True when the sample was received more than stale_s before `now`.
    Both times must come from the same clock."""
    return (now - sample_t) > stale_s


def advance_clock(last_t, t: float, dt_max: float = DT_MAX_S):
    """Return (new_last_t, dt). dt is None when this sample must not be
    integrated: the first sample, a non-increasing time, or a gap > dt_max.
    new_last_t is always t, so the interval after a rejected gap is measured
    from the rejected sample, not from the last integrated one."""
    if last_t is None:
        return t, None
    dt = t - last_t
    if dt <= 0.0 or dt > dt_max:
        return t, None
    return t, dt


def speed_gate(v_ms: float) -> float:
    """Zero a speed below MIN_SPEED_MS; sign is preserved otherwise."""
    return 0.0 if abs(v_ms) < MIN_SPEED_MS else v_ms


def _finite_or_none(value):
    if value is None:
        return None
    try:
        f = float(value)
    except (TypeError, ValueError):
        return None
    if math.isnan(f) or math.isinf(f):
        return None
    return f


def sanitize_can_sample(yaw_rate, yaw_offset, yaw_ok):
    """The single boundary check for a YAW_SENSOR sample.

    Returns (rate, offset, ok): rate and offset as floats (or None), and ok True
    only when the sensor reports valid AND both values are finite numbers.
    Callers must treat ok=False as "no yaw measurement this sample".
    """
    rate = _finite_or_none(yaw_rate)
    offset = _finite_or_none(yaw_offset)
    ok = bool(yaw_ok) and rate is not None and offset is not None
    return rate, offset, ok


def yaw_rate_offset_corrected(rate: float, offset: float) -> float:
    """Per-sample ECU offset subtraction (Track A). Inputs already sanitized."""
    return rate - offset


def standstill_gate(yaw_rate: float, v_ms: float) -> float:
    """Zero a yaw rate while the car is stationary. v_ms is the UNSIGNED speed."""
    return 0.0 if abs(v_ms) < MIN_SPEED_MS else yaw_rate


def track_a_yaw_rate(yaw_rate, yaw_offset, yaw_ok, v_ms: float) -> float:
    """Complete Track A yaw rate for one sample, exactly as odom_node computes it:
    0.0 without a valid sample, otherwise (rate - offset), zeroed at standstill."""
    rate, offset, ok = sanitize_can_sample(yaw_rate, yaw_offset, yaw_ok)
    if not ok:
        return 0.0
    return standstill_gate(yaw_rate_offset_corrected(rate, offset), v_ms)
