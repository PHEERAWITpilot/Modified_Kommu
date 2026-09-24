#!/usr/bin/env python3
"""
byd_odom_node — ROS2 node publishing TWO parallel live odometry estimates for
the BYD Dolphin, for direct visual comparison in RViz.

    DEVICE (Kommu, no ROS)                    LAPTOP (ROS2 Jazzy)
    ----------------------                    -------------------
    bukapilot / cereal
        |
        v
    byd_cereal_server.py  --- TCP/JSON --->   THIS NODE
    (existing project tool,                       |
     port 5556)                                   +--> /byd/odom_measured   (nav_msgs/Odometry)
                                                   +--> /byd/path_measured   (nav_msgs/Path)
                                                   +--> TF: odom -> base_link_measured
                                                   |
                                                   +--> /byd/odom_corrected  (nav_msgs/Odometry)
                                                   +--> /byd/path_corrected  (nav_msgs/Path)
                                                   +--> TF: odom -> base_link_corrected
                                                            |
                                                            v
                                                          RViz2 (green = measured, orange = corrected)

  The raw uncorrected kinematic track (/byd/*_kinematic, steer_ratio=13.11,
  no offset removal) is DEPRECATED and commented out — see the block in
  __init__. It is no longer constructed, stepped, or published.

INTEGRATORS (same input stream, different heading source):

  MEASURED  — heading rate comes directly from the car's own yaw_rate signal
              (cs.yawRate via cereal). This is the car's actual sensed rotation,
              not derived from steering geometry at all.

  KINEMATIC — heading rate is derived purely from steer_deg + v_kmh through the
              bicycle model (tan(delta)/wheelbase). No sensor fusion, no
              correction — this is exactly what the earlier byd_odometry.py
              work characterized as having an ~8% yaw over-prediction at
              steer_ratio=13.11 (effective kinematic ratio nearer 14.1-14.3).

Both integrators consume the SAME v_kmh sample at the SAME timestep, so any
divergence you see between the two paths in RViz is attributable to the
heading-rate source alone, not to different speed data.

⚠️ REQUIRES byd_cereal_server.py to emit a "yaw_rate" field. If it's absent
from a given sample, the MEASURED integrator holds its heading constant for
that tick (does not fall back to the kinematic estimate silently — that would
defeat the point of having two independent traces to compare) and a one-time
warning is logged.

Constants override the CAR.BYD_SEAL CarSpecs placeholders (Seal values, not
Dolphin): wheelbase = 2.70 m (placeholder 2.92), steer_ratio = 13.11 (placeholder
16.0, and itself known to carry ~8% kinematic bias per prior validation).
"""

import argparse
import csv
import datetime
import fcntl
import json
import os
from collections import deque
import json
import math
import os
import socket
import sys
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Quaternion, TransformStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

try:
    from byd_odom_ros.track_a_core import (
        Integrator, MIN_SPEED_MS, advance_clock, is_stale, sanitize_can_sample,
        speed_gate, standstill_gate, yaw_rate_offset_corrected)
except ImportError:
    # Loaded as a bare file (claude/tests/ import it by path), not as a package.
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from track_a_core import (
        Integrator, MIN_SPEED_MS, advance_clock, is_stale, sanitize_can_sample,
        speed_gate, standstill_gate, yaw_rate_offset_corrected)

WHEELBASE_M = 2.70
STEER_RATIO = 13.11
DEFAULT_RECORD_DIR = os.path.expanduser("~/Desktop/Kommu.AI/Odom_record")
# --- live stream health ---------------------------------------------------
# A gap is measured on rx_mono between DISTINCT samples: the same 0.3 s
# threshold the stall investigation used, so numbers here and in the offline
# analysis mean the same thing.
HEALTH_GAP_S = 0.3
HEALTH_WINDOW_S = 5.0       # rolling window for Hz and latency percentiles
HEALTH_PRINT_S = 2.5        # one summary line at most this often
# --- incremental CSV ------------------------------------------------------
# Rows are appended to disk as the drive runs, so a SIGKILL or a power cut
# costs at most FLUSH_S of data instead of the entire session.
CSV_FLUSH_S = 5.0
CSV_FLUSH_ROWS = 250
RECORD_README = """This directory holds REAL DRIVE RECORDINGS written by odom_node.py.

Each run is Odom_record/YYYY/MM/DD/HHMM/ containing odom.csv and meta.json,
timestamped from when the run STARTED. The CSV files are made read-only after
writing.

INDEX.csv is an append-only log of every run ever saved. If a run directory is
missing but its line is still in INDEX.csv, that recording was deleted.

Do NOT write test or scratch runs here. Point them somewhere else:
    ros2 run byd_odom_ros odom_node --record-dir /tmp/byd_test_record ...
"""
# Every track this node integrates, in the order their columns appear in the
# recorded CSV. Attribute name -> column prefix.
# DISABLED 2026-09-11 — the node now integrates only three tracks: `corrected`
# (steer-derived, ratio 14.2, angle-offset removed) and Methods A and B (both
# yaw-sensor derived, differing only in how the sensor bias is estimated).
#
# `measured`, `deltaref`, `yawsensor`, `startref` and `steerproxy` are commented
# out throughout: constructor, publishers, per-tick computation, integration and
# publication. Their code is left in place, disabled, so any of them can be
# revived by uncommenting — nothing was deleted.
#
# CONSEQUENCE FOR RECORDINGS: odom.csv is now NARROWER. Runs written from here
# on are NOT column-compatible with the 19 runs recorded up to 2026-09-10.
# Read the column names from the header row rather than by position.
#
# SECOND BREAKPOINT, 2026-09-11: `ekf_fused` added, plus 17 hardcoded EKF
# scalar columns appended at the very end of the row. odom.csv is WIDER again,
# so runs from here on are not column-compatible with the three-track runs
# either. The by-name rule above now applies twice over.
RECORD_TRACKS = (
    ("corrected", "corrected"),
    ("yawoffs", "trackA_persample"), ("yawavg", "trackB_windowed"),
    ("ekf_ba_slow", "ekf_ba_slow"), ("ekf_ba_tight", "ekf_ba_tight"),
)
# THIRD BREAKPOINT, 2026-09-15: the single `ekf` pose (ekf_x/_y/_yaw_deg) is
# replaced by two EKF tracks, ekf_ba_slow_* and ekf_ba_tight_*, identical except
# for the phase-3 b_a random-walk rate. The 17 ekf_* scalar columns and every
# ekf_* meta.json key describe the PRIMARY track, ekf_ba_slow.

# v_kmh from the device is an unsigned wheel-speed magnitude, so a reverse
# manoeuvre integrates as forward motion and folds the path back on itself.
# Only D and R describe travel; N/P and any unknown or transitional value map
# to 0 so an ambiguous gear never contributes assumed-forward motion.
GEAR_SIGN = {"drive": 1.0, "reverse": -1.0, "neutral": 0.0, "park": 0.0}

# --- ekf_fused: 7-state EKF -------------------------------------------------
# State vector layout. Everything internal is radians / rad per second; degrees
# appear only in the CSV/display columns, matching the other tracks.
#
#   X = [x, y, psi, v, r, b_r, theta_g, b_a]
#
#   b_r     residual gyro bias BEYOND what per-sample offset subtraction removes
#   theta_g curvature gain, == steer_ratio * wheelbase; r = v * g / theta_g,
#           g = sR0 * tan(steer_rad / sR0)  (see _effective_steer)
IX_X, IX_Y, IX_PSI, IX_V, IX_R, IX_B_R, IX_THETA_G, IX_B_A = range(8)
EKF_N = 8
# b_a (8th state): residual longitudinal accel bias in m/s^2 AFTER the
# session-zero subtraction. Positive = accelerometer reads too high.
# theta_g is a divisor. Clamp it at USE, not just at state level, and clamp low
# enough that a physically absurd estimate cannot invert the steering sign.
EKF_THETA_G_MIN = 10.0
EKF_THETA_G_MAX = 100.0
EKF_P_DIAG_FLOOR = 1e-12
EKF_DET_MIN = 1e-15


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def tire_angle_rad(wheel_angle_deg: float, steer_ratio: float) -> float:
    return math.radians(wheel_angle_deg) / steer_ratio


def heading_rate_rad_s(v_ms: float, tire_rad: float, wheelbase: float) -> float:
    return v_ms * math.tan(tire_rad) / wheelbase


# ---------------------------------------------------------------------------
# EKF support: tiny pure-Python linear algebra.
#
# Deliberately NOT numpy. This file imports only stdlib + rclpy, and the
# standalone test suite in claude/tests/ imports these functions with the ROS
# modules stubbed out -- adding numpy would put a third-party dependency in
# both places to save ~10 microseconds per update. A 7x7 matmul is 343
# multiplies and S is never larger than 2x2, so a closed-form inverse is all
# the linear algebra required: no general solver, no pivoting.
# ---------------------------------------------------------------------------
def _matmul(A, B):
    n, k, m = len(A), len(B), len(B[0])
    return [[sum(A[i][x] * B[x][j] for x in range(k)) for j in range(m)]
            for i in range(n)]


def _matadd(A, B):
    return [[A[i][j] + B[i][j] for j in range(len(A[0]))] for i in range(len(A))]


def _transpose(A):
    return [[A[j][i] for j in range(len(A))] for i in range(len(A[0]))]


def _eye(n):
    return [[1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]


def _symmetrize(P):
    """P = (P + P^T)/2. Nearly free, and it removes the asymmetry drift that
    otherwise makes a Cholesky-based PSD check fail spuriously."""
    n = len(P)
    return [[0.5 * (P[i][j] + P[j][i]) for j in range(n)] for i in range(n)]


def _inv_small(S):
    """Closed-form inverse for 1x1 and 2x2 only, returning None when the matrix
    is too near-singular to trust. Those are the only sizes this filter ever
    produces: a 2x7 update when both measurements are fresh, 1x7 when only one
    is, and 1x7 again for the ZUPT pseudo-measurement."""
    m = len(S)
    if m == 1:
        if abs(S[0][0]) < EKF_DET_MIN:
            return None
        return [[1.0 / S[0][0]]]
    if m == 2:
        det = S[0][0] * S[1][1] - S[0][1] * S[1][0]
        if abs(det) < EKF_DET_MIN:
            return None
        return [[S[1][1] / det, -S[0][1] / det],
                [-S[1][0] / det, S[0][0] / det]]
    raise ValueError("_inv_small handles m=1 and m=2 only, got %d" % m)


def _fnum(value):
    """Coerce a stream field to a finite float, or None.

    None, non-numeric, NaN and Inf are all treated identically: absent. The
    device stream legitimately produces None (d.get on a field the server has
    not populated yet -- accel_* stay None until CAN 547 first decodes), and
    float(None) raises, so every EKF input goes through here."""
    if value is None:
        return None
    try:
        f = float(value)
    except (TypeError, ValueError):
        return None
    if math.isnan(f) or math.isinf(f):
        return None
    return f


def _all_finite(X, P):
    for v in X:
        if math.isnan(v) or math.isinf(v):
            return False
    for row in P:
        for v in row:
            if math.isnan(v) or math.isinf(v):
                return False
    return True


def _effective_steer(steer_rad, cfg):
    """Steering-WHEEL angle (rad) -> the exact bicycle-model steer term g.

    steer_rad is the steering-WHEEL angle, not the front-wheel angle, and
    theta_g is steer_ratio * wheelbase. So tan() must NOT be applied to
    steer_rad directly: for a 460 deg wheel that is tan(8.03 rad) = -5.67, a
    sign flip. The exact model is r = v * tan(delta_f) / L with
    delta_f = steer_rad / sR0 and L = theta_g / sR0, i.e.

        r = v * g / theta_g,   g = sR0 * tan(steer_rad / sR0)

    g reduces to steer_rad at small angles, so theta_g keeps its meaning and
    its 38.34 seed. At full lock the old linear form under-predicted yaw by
    ~11% -- what dragged theta_g to 25-27 during parking manoeuvres. sR0 is the
    fixed nominal ratio; an error in it enters only the tan() curvature
    correction, a second-order effect.

    Returns (g, clamped). The front-wheel angle is clamped to
    +/-tire_angle_max_rad so neither tan() nor the Jacobian can approach the
    +/-90 deg singularity. The largest lock seen across every recording is
    491.3 deg at the wheel, 34.6 deg at the tire, so the 45 deg default is
    never reached in normal driving.
    """
    sr0 = cfg["steer_ratio_nominal"]
    lim = cfg["tire_angle_max_rad"]
    delta_f = steer_rad / sr0
    clamped = abs(delta_f) > lim
    if clamped:
        delta_f = lim if delta_f > 0 else -lim
    return sr0 * math.tan(delta_f), clamped


def predict_ekf(X, P, dt, steer_rad, accel_long_corrected, cfg):
    """One EKF prediction step. Pure: no self, no globals, no mutation of the
    arguments -- new X and P are returned. cfg is a plain dict so the test
    suite can sweep tuning without touching module state.

    accel_long_corrected may be None, meaning CAN 547 gave us nothing this
    tick. Zero is the correct PRIOR for an unknown acceleration, but believing
    it would be wrong, so Q_v is inflated for that step instead.
    """
    x, y, psi, v, _r, b_r, theta_g, b_a = X

    # theta_g is a divisor AND appears in the Jacobian. Clamp ONCE and use the
    # same clamped value in both places: if the prediction used one value and
    # F used another, F would not be the derivative of the function actually
    # applied and P would be silently wrong.
    theta_g_safe = min(max(theta_g, cfg["theta_g_min"]), cfg["theta_g_max"])
    # Exact-model steer term, computed ONCE and used by the r prediction and
    # both r-row Jacobian entries, so the three cannot drift apart.
    g, _steer_clamped = _effective_steer(steer_rad, cfg)

    accel_missing = accel_long_corrected is None
    a = 0.0 if accel_missing else accel_long_corrected

    # --- b_a: residual accel bias (8th state) -------------------------------
    # accel fed to the velocity prediction is, end to end:
    #     (accel_long - accel_long_offset) - session_zero_bias - b_a
    # The session-zero bias is computed in tick() from the SAME offset-
    # subtracted expression, so the ECU offset is never subtracted twice.
    #
    # ba_active is False until the car first moves (phase 1). While inactive,
    # b_a is EXCLUDED from the dynamics: not subtracted here, and d v/d b_a is 0
    # in F below. That is what actually freezes it -- sigma_ba = 0 alone would
    # not: with d v/d b_a = -dt the ZUPT updates at standstill would move b_a
    # through the v-b_a cross-covariance and it would absorb the parking slope
    # BEFORE session-zero subtracts that same slope, i.e. a double correction.
    # With the coupling off, P[v][b_a] stays 0 and b_a's gain is exactly 0,
    # without zeroing any row of K.
    #
    # Known limitations (do not treat b_a as a calibration measurement):
    #  1. Scale-error contamination. Accel vs wheel-speed slope is 1.07-1.15;
    #     b_a is a constant offset and cannot represent a gain error, so hard
    #     acceleration/braking leaks some scale error into it. The real fix is a
    #     wheel-speed scale-factor state (v3, deferred).
    #  2. Road gradient. Replay 2026-09-15: at sigma_ba_slow 0.008 phase-3 b_a
    #     TRACKED grade (range 0.43-0.59 m/s^2, 10 s swings 0.27-0.33,
    #     corr +0.6 with a grade proxy). ekf_ba_tight runs the same filter at
    #     0.001 so the live drive shows whether a tighter rate holds b_a steady.
    ba_active = cfg["ba_active"]
    a_eff = (a - b_a) if ba_active else a

    # --- state propagation ---
    x_n = x + v * math.cos(psi) * dt
    y_n = y + v * math.sin(psi) * dt
    psi_n = psi + _r * dt if cfg["integrate_psi"] else psi
    # wrap the STATE only, never inside the covariance update
    psi_n = math.atan2(math.sin(psi_n), math.cos(psi_n))
    v_n = v + a_eff * dt
    # r is REPLACED, not integrated: the bicycle model gives yaw rate directly
    # as a function of current speed and steering angle. Intentional.
    r_n = v_n * g / theta_g_safe
    X_n = [x_n, y_n, psi_n, v_n, r_n, b_r, theta_g, b_a]

    # --- Jacobian F = df/dX, evaluated at the CURRENT estimate ---
    F = [[0.0] * EKF_N for _ in range(EKF_N)]
    F[IX_X][IX_X] = 1.0
    F[IX_X][IX_PSI] = -v * math.sin(psi) * dt
    F[IX_X][IX_V] = math.cos(psi) * dt
    F[IX_Y][IX_Y] = 1.0
    F[IX_Y][IX_PSI] = v * math.cos(psi) * dt
    F[IX_Y][IX_V] = math.sin(psi) * dt
    F[IX_PSI][IX_PSI] = 1.0
    F[IX_PSI][IX_R] = dt if cfg["integrate_psi"] else 0.0
    F[IX_V][IX_V] = 1.0
    # r depends on v and theta_g only -- NOT on its own previous value, because
    # the prediction replaces it. Hence F[r][r] == 0.
    F[IX_R][IX_V] = g / theta_g_safe
    F[IX_R][IX_THETA_G] = -v_n * g / (theta_g_safe * theta_g_safe)
    F[IX_B_R][IX_B_R] = 1.0
    F[IX_THETA_G][IX_THETA_G] = 1.0
    # b_a enters v_n = v + (a - b_a)*dt, so d v_n/d b_a = -dt. r_n = v_n*g/theta_g
    # is computed FROM v_n, so it inherits d r_n/d b_a = (g/theta_g)*(-dt) by the
    # chain rule -- omitting that entry would make F stop being the derivative of
    # the prediction actually applied. Both use the same dt, g and theta_g_safe
    # as the prediction above, and both are 0 while b_a is inactive.
    dv_dba = -dt if ba_active else 0.0
    F[IX_V][IX_B_A] = dv_dba
    F[IX_R][IX_B_A] = dv_dba * g / theta_g_safe
    F[IX_B_A][IX_B_A] = 1.0

    # --- Q_v: the ONE entry that is NOT sigma^2 * dt -------------------------
    # v is not a random walk. It has a deterministic drive term
    # (v += accel_corrected * dt) and its process noise is a MEASURED
    # per-sample accelerometer error in m/s^2. sigma_a = 0.13 IS that measured
    # sd -- taken WHILE DRIVING (0.117-0.141 m/s^2 across five runs), not at
    # standstill, where Park reads ~0.018 and a just-stopped car in Drive
    # ~0.12 -- so the quadratic form is precisely what that number means.
    # Do NOT "fix" this to sigma_a**2 * dt: that changes injected velocity
    # noise by 1/dt (50x at the 50 Hz predict rate) and strips sigma_a of its
    # empirical basis. The linear branch exists only to A/B on one recording.
    if cfg["qv_mode"] == "linear":
        q_v = cfg["sigma_a"] ** 2 * dt
    else:                                    # "quadratic" -- the default path
        q_v = (cfg["sigma_a"] * dt) ** 2
    if accel_missing:
        q_v *= cfg["qv_missing_mult"]

    # every OTHER entry is a true random walk, so sigma^2 * dt
    Q = [0.0] * EKF_N
    Q[IX_X] = cfg["sigma_xy"] ** 2 * dt
    Q[IX_Y] = cfg["sigma_xy"] ** 2 * dt
    Q[IX_PSI] = cfg["sigma_psi"] ** 2 * dt
    Q[IX_V] = q_v
    Q[IX_R] = cfg["sigma_r"] ** 2 * dt
    Q[IX_B_R] = cfg["sigma_br"] ** 2 * dt
    Q[IX_THETA_G] = cfg["sigma_thetag"] ** 2 * dt
    # b_a: random walk whose rate depends on the drive phase (0 / fast / slow),
    # chosen by EkfTrack.step and handed in through cfg.
    Q[IX_B_A] = cfg["sigma_ba"] ** 2 * dt

    # P = F P F^T + Q
    P_n = _matmul(_matmul(F, P), _transpose(F))
    for i in range(EKF_N):
        P_n[i][i] += Q[i]
    P_n = _symmetrize(P_n)
    return X_n, P_n


def update_ekf(X, P, z, H, R, lock_theta_g=False, joseph=True):
    """One EKF measurement update. Generic in row count, so yaw-only (1x7),
    speed-only (1x7), both (2x7) and the ZUPT pseudo-measurement all take this
    same path. The returned K is the one ACTUALLY APPLIED (post-gate)."""
    m = len(H)
    n = len(X)

    HX = _matmul(H, [[xi] for xi in X])                  # m x 1
    innov = [[z[i] - HX[i][0]] for i in range(m)]        # m x 1

    Ht = _transpose(H)                                   # n x m
    PHt = _matmul(P, Ht)                                 # n x m
    S = _matadd(_matmul(H, PHt), R)                      # m x m

    Sinv = _inv_small(S)
    if Sinv is None:                                     # |det| < EKF_DET_MIN
        return X, P, innov, S, None                      # caller counts + warns once

    K = _matmul(PHt, Sinv)                               # n x m

    # ---- theta_g excitation lock -------------------------------------------
    # K is states x measurements, so theta_g is a ROW, not a column. Zero it
    # IN PLACE, HERE, before K is read by anything below. Both consumers use
    # this same object, so they cannot disagree about how much information
    # theta_g received. Zeroing only at the state update would let P shrink
    # theta_g's variance for information it never got -- overconfidence that
    # compounds during exactly the low-excitation stretches where the filter
    # should be most humble, and never self-corrects.
    #
    # Provable consequence, asserted by the test suite: with K[theta_g][*] = 0,
    # row theta_g of KH is zero, so that row of (I - KH) is e_theta_g, giving
    # [(I-KH) P (I-KH)^T][tg][tg] == P[tg][tg] and [K R K^T][tg][tg] == 0.
    # A gated update therefore leaves P[tg][tg] BITWISE unchanged.
    if lock_theta_g:
        for j in range(m):
            K[IX_THETA_G][j] = 0.0
    # ------------------------------------------------------------------------

    # consumer 1 of K -- state update
    dX = _matmul(K, innov)                               # n x 1
    X = [X[i] + dX[i][0] for i in range(n)]
    # Re-wrap psi. predict_ekf wraps it too, but the correction above is applied
    # AFTER that wrap and can push it back outside [-pi, pi] -- psi has no
    # measurement row, yet K's psi row is generally nonzero through the
    # cross-covariance, so the update does move it. Wrapping the STATE only;
    # the covariance is untouched, and no residual involves psi so there is no
    # residual-wrapping problem to solve.
    X[IX_PSI] = math.atan2(math.sin(X[IX_PSI]), math.cos(X[IX_PSI]))

    # consumer 2 of K -- covariance update, SAME gated K
    KH = _matmul(K, H)                                   # n x n
    if joseph:
        #   P = (I - K H) P (I - K H)^T + K R K^T
        # Joseph rather than the naive (I-KH)P: R_r is ~9e-6 against order-1 P
        # entries, so this is a very high-gain update and (I-KH)P is exactly
        # where catastrophic cancellation shows up. b_r and theta_g are also
        # near-collinear by construction, so P's smallest eigenvalue lives in
        # the direction where that cancellation is worst -- and that direction
        # is the one rho measures.
        IKH = _matadd(_eye(n), [[-val for val in row] for row in KH])
        P = _matadd(_matmul(_matmul(IKH, P), _transpose(IKH)),
                    _matmul(_matmul(K, R), _transpose(K)))
    else:
        IKH = _matadd(_eye(n), [[-val for val in row] for row in KH])
        P = _matmul(IKH, P)
    P = _symmetrize(P)

    return X, P, innov, S, K


# ---------------------------------------------------------------------------
# theta_g seed persistence (tuning round 2, Task 6)
#
# No state file. The node runs on the LAPTOP, so the device's auto-updater never
# touches it, and every run already writes its converged theta_g into its own
# meta.json. The seed is taken from the newest run in record_dir/INDEX.csv that
# passes every check below. Scratch runs pointed at another --record-dir have
# their own INDEX.csv and can never contaminate the real seed.
# ---------------------------------------------------------------------------
THETA_G_PERSIST_MIN = 37.0          # brackets every good-condition estimate so far
THETA_G_PERSIST_MAX = 40.0          # (37.0-39.3) and excludes the 25-29 parking dips
THETA_G_PERSIST_SD_MAX = 1.0        # must sit below the persisted P0 sd, or a run
                                    # that learned nothing could re-persist its seed
THETA_G_PERSIST_MIN_DURATION_S = 120.0
THETA_G_PERSIST_MIN_UPDATES = 1000
# Starting theta_g uncertainty. Replay of runs 1653/1700/1721: the early-drive
# dip bottoms at 28-30 with sd 5, 34-35 with sd 2. A qualified persisted seed
# has earned the narrower prior; a default or override seed has not.
EKF_P0_THETA_G_SD_DEFAULT = 5.0
EKF_P0_THETA_G_SD_PERSISTED = 2.0


def _theta_g_run_qualifies(meta, wheelbase, steer_ratio):
    """None if this run's final theta_g may seed the next run, else the reason.

    Strict: any missing or unparseable field rejects the run. Runs recorded
    before ekf_theta_g_sd_final existed therefore never qualify."""
    try:
        fin = float(meta["ekf_theta_g_final"])
        sd = float(meta["ekf_theta_g_sd_final"])
        if not (math.isfinite(fin) and math.isfinite(sd)):
            return "non-finite theta_g or sd"
        if not (THETA_G_PERSIST_MIN <= fin <= THETA_G_PERSIST_MAX):
            return "final theta_g %.3f outside [%.1f, %.1f]" % (
                fin, THETA_G_PERSIST_MIN, THETA_G_PERSIST_MAX)
        if sd >= THETA_G_PERSIST_SD_MAX:
            return "final theta_g sd %.3f >= %.1f" % (sd, THETA_G_PERSIST_SD_MAX)
        for key in ("ekf_resets", "ekf_theta_g_clamps", "ekf_singular",
                    "ekf_steer_clamps"):
            if int(meta[key]) != 0:
                return "%s = %s" % (key, meta[key])
        if float(meta["duration_s"]) < THETA_G_PERSIST_MIN_DURATION_S:
            return "duration %.0f s < %.0f s" % (
                float(meta["duration_s"]), THETA_G_PERSIST_MIN_DURATION_S)
        if int(meta["ekf_updates"]) < THETA_G_PERSIST_MIN_UPDATES:
            return "updates %s < %d" % (meta["ekf_updates"],
                                        THETA_G_PERSIST_MIN_UPDATES)
        # theta_g means steer_ratio * wheelbase, and the tan() model uses the
        # ratio directly, so a run under different geometry is not comparable.
        if abs(float(meta["wheelbase_m"]) - wheelbase) > 1e-9:
            return "wheelbase %s != %s" % (meta["wheelbase_m"], wheelbase)
        if abs(float(meta["corrected_steer_ratio"]) - steer_ratio) > 1e-9:
            return "steer ratio %s != %s" % (meta["corrected_steer_ratio"],
                                             steer_ratio)
    except (KeyError, TypeError, ValueError) as e:
        return "missing or invalid field: %r" % (e,)
    return None


def _select_theta_g_seed(override, record_dir, wheelbase, steer_ratio):
    """Choose theta_g's seed and starting sd. Never raises, never blocks startup.

    Returns (seed, source, run_dir, p0_sd, note) with source one of
    "override", "persisted", "default"."""
    default = steer_ratio * wheelbase
    if override is not None:
        return (float(override), "override", None, EKF_P0_THETA_G_SD_DEFAULT,
                "--ekf-theta-g given")
    try:
        with open(os.path.join(record_dir, "INDEX.csv"), newline="") as fh:
            rows = list(csv.DictReader(fh))
    except (OSError, csv.Error, UnicodeDecodeError):
        return (default, "default", None, EKF_P0_THETA_G_SD_DEFAULT,
                "no readable INDEX.csv in %s" % record_dir)
    checked = 0
    last_reason = "no runs recorded"
    for row in reversed(rows):                      # newest first
        run_dir = (row.get("run_dir") or "").strip()
        if not run_dir:
            continue
        checked += 1
        try:
            with open(os.path.join(record_dir, run_dir, "meta.json")) as fh:
                meta = json.load(fh)
        except (OSError, ValueError) as e:
            last_reason = "%s: unreadable meta.json (%s)" % (run_dir, e)
            continue
        why = _theta_g_run_qualifies(meta, wheelbase, steer_ratio)
        if why is None:
            return (float(meta["ekf_theta_g_final"]), "persisted", run_dir,
                    EKF_P0_THETA_G_SD_PERSISTED,
                    "from %s" % run_dir)
        last_reason = "%s: %s" % (run_dir, why)
    return (default, "default", None, EKF_P0_THETA_G_SD_DEFAULT,
            "no qualifying run among %d (newest rejection: %s)"
            % (checked, last_reason))


class DeviceStreamClient(threading.Thread):
    """Reads newline-delimited JSON from the device's cereal TCP server.
    Own thread so a network stall never blocks the ROS executor. Reconnects
    indefinitely."""

    def __init__(self, host, port, logger):
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.logger = logger
        self.lock = threading.Lock()
        self.latest = None
        self.connected = False
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def get_latest(self):
        with self.lock:
            return self.latest

    def run(self):
        while not self._stop.is_set():
            sock = None
            try:
                self.logger.info(f"connecting to device {self.host}:{self.port} ...")
                sock = socket.create_connection((self.host, self.port), timeout=5.0)
                sock.settimeout(2.0)
                self.connected = True
                self.logger.info("connected to device cereal stream")
                buf = b""
                while not self._stop.is_set():
                    try:
                        chunk = sock.recv(4096)
                    except socket.timeout:
                        continue
                    if not chunk:
                        raise ConnectionError("device closed the stream")
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            d = json.loads(line)
                        except json.JSONDecodeError:
                            continue
                        with self.lock:
                            self.latest = (d, time.monotonic())
            except Exception as e:
                self.connected = False
                self.logger.warn(f"device stream lost ({e}); retrying in 2s")
                time.sleep(2.0)
            finally:
                if sock is not None:
                    try:
                        sock.close()
                    except Exception:
                        pass


class EkfTrack:
    """The `ekf_fused` track: a 7-state EKF, not a dead-reckoner.

    Every other track picks ONE heading source and integrates it. This one
    predicts with wheel speed + steering + longitudinal accel, corrects with
    wheel speed + yaw rate, and additionally estimates two calibration
    parameters online that no other track has: the residual gyro bias b_r and
    the curvature gain theta_g.

    It exposes .x/.y/.yaw as PLAIN ATTRIBUTES, synced at the end of step(), so
    it is structurally an Integrator as far as _publish_track and the
    RECORD_TRACKS row loop are concerned -- both touch only those three. They
    are deliberately not properties: a property would break silently the first
    time anything assigned to .yaw.
    """

    def __init__(self, cfg, theta_g_seed):
        self.cfg = cfg
        self.theta_g_seed = theta_g_seed
        # b_a starts at 0: session-zero has already removed what it measured.
        # Never persisted across drives -- it is dominated by the parking slope.
        self.X0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, theta_g_seed, 0.0]
        self.P0 = [[0.0] * EKF_N for _ in range(EKF_N)]
        for i, sd in enumerate(cfg["p0_sd"]):
            self.P0[i][i] = sd * sd
        self.X = list(self.X0)
        self.P = [row[:] for row in self.P0]
        # published pose, kept in sync at the end of every step()
        self.x = self.y = self.yaw = 0.0
        # diagnostics
        self.updates = 0
        self.skipped = 0
        self.theta_g_clamps = 0
        self.singular = 0
        self.steer_clamps = 0
        self.resets = 0
        self.nis_sum = 0.0
        self.nis_n = 0
        self.last_innov_v = None
        self.last_innov_r = None
        self.last_nis = None
        self.last_updated = 0
        self.ok = True
        # b_a three-phase process noise. One-way: 1 -> 2 -> 3, never back.
        #   1 stationary, car has not moved yet: b_a out of the dynamics
        #   2 first movement, up to ba_fast_window s: sigma_ba_fast
        #   3 normal driving: sigma_ba_slow
        self.ba_phase = 1
        self.ba_phase2_t = 0.0
        self.ba_phase2_exit = "never"     # "time" | "never"
        self.ba_phase2_exit_t = None

    def _sync_pose(self):
        self.x, self.y, self.yaw = self.X[IX_X], self.X[IX_Y], self.X[IX_PSI]

    def _reset_numeric(self):
        """Recover from a non-finite value WITHOUT throwing the pose away.

        The pose is this track's product; discarding it turns a numerical
        hiccup into a visible path jump in RViz. So x/y/psi survive and only
        the estimator states and the covariance are reinitialised."""
        keep = self.X[:3]
        if not all(math.isfinite(val) for val in keep):
            keep = [0.0, 0.0, 0.0]
        self.X = keep + [0.0, 0.0, 0.0, self.theta_g_seed, 0.0]
        self.P = [row[:] for row in self.P0]
        self.resets += 1
        self.ok = False

    def step(self, dt, steer_rad, accel_corrected, z_v, z_r, do_update,
             lock_theta_g, integrate_psi, zupt, moving):
        """Predict, then optionally correct. Returns nothing; read .X/.P or the
        synced .x/.y/.yaw afterwards."""
        self.ok = True
        self.last_innov_v = None
        self.last_innov_r = None
        self.last_nis = None
        self.last_updated = 0

        cfg = dict(self.cfg)
        cfg["integrate_psi"] = integrate_psi
        # b_a phase: phase 1 ends on the FIRST tick the wheels move, not when
        # the session-zero window freezes (that can happen after 100 samples
        # while still parked).
        if self.ba_phase == 1 and moving:
            self.ba_phase = 2
            self.ba_phase2_t = 0.0
        if self.ba_phase == 1:
            cfg["ba_active"], cfg["sigma_ba"] = False, 0.0
        elif self.ba_phase == 2:
            cfg["ba_active"], cfg["sigma_ba"] = True, self.cfg["sigma_ba_fast"]
        else:
            cfg["ba_active"], cfg["sigma_ba"] = True, self.cfg["sigma_ba_slow"]
        # same helper predict_ekf uses, so the count cannot disagree with it
        if _effective_steer(steer_rad, cfg)[1]:
            self.steer_clamps += 1

        self.X, self.P = predict_ekf(self.X, self.P, dt, steer_rad,
                                     accel_corrected, cfg)
        if not _all_finite(self.X, self.P):
            self._reset_numeric()
            self._sync_pose()
            return

        if do_update:
            # Missing measurements are SKIPPED, never zeroed. Feeding 0.0 for an
            # absent reading asserts the car is stationary / not turning, which
            # is a fabricated observation and worse than no observation.
            rows, z, R_diag = [], [], []
            if zupt:
                # Zero-velocity pseudo-measurement. Not a gate: it goes through
                # the same update machinery, so its effect is visible in P.
                # Also sharpens the b_r observation by pinning v hard at 0.
                rows.append([0.0] * EKF_N)
                rows[-1][IX_V] = 1.0
                z.append(0.0)
                R_diag.append(self.cfg["r_zupt"] ** 2)
            elif z_v is not None:
                rows.append([0.0] * EKF_N)
                rows[-1][IX_V] = 1.0
                z.append(z_v)
                R_diag.append(self.cfg["r_v"] ** 2)
            if z_r is not None:
                row = [0.0] * EKF_N
                row[IX_R] = 1.0
                row[IX_B_R] = 1.0      # z_r measures true r PLUS residual bias
                rows.append(row)
                z.append(z_r)
                R_diag.append(self.cfg["r_r"] ** 2)
            # v2: add accel_lat measurement, see spec section 6.
            #     h(X) = v * r (centripetal), validated for sign: slope 1.02,
            #     r = 0.80 against v*yawRate on a real drive. Deliberately not
            #     wired until the two-measurement filter has a converged
            #     baseline to judge a third channel against.

            if rows:
                m = len(rows)
                R = [[R_diag[i] if i == j else 0.0 for j in range(m)]
                     for i in range(m)]
                Xn, Pn, innov, S, K = update_ekf(
                    self.X, self.P, z, rows, R,
                    lock_theta_g=lock_theta_g, joseph=self.cfg["joseph"])
                if K is None:
                    self.singular += 1
                else:
                    self.X, self.P = Xn, Pn
                    self.updates += 1
                    self.last_updated = 1
                    # NIS = innov^T S^-1 innov. Expectation is m (the dof), so
                    # a running mean far from that means R/Q are mistuned.
                    Sinv = _inv_small(S)
                    if Sinv is not None:
                        nis = 0.0
                        for i in range(m):
                            for j in range(m):
                                nis += innov[i][0] * Sinv[i][j] * innov[j][0]
                        self.last_nis = nis
                        self.nis_sum += nis
                        self.nis_n += 1
                    # label the innovations by which row they came from
                    idx = 0
                    if zupt or z_v is not None:
                        self.last_innov_v = innov[idx][0]
                        idx += 1
                    if z_r is not None:
                        self.last_innov_r = innov[idx][0]
        else:
            self.skipped += 1

        # clamp theta_g at STATE level too, and count it: a nonzero count means
        # the filter fought the model and the run's calibration is suspect
        tg = self.X[IX_THETA_G]
        lo, hi = self.cfg["theta_g_min"], self.cfg["theta_g_max"]
        if not math.isfinite(tg) or tg < lo or tg > hi:
            self.X[IX_THETA_G] = min(max(tg if math.isfinite(tg) else lo, lo), hi)
            self.theta_g_clamps += 1

        if not _all_finite(self.X, self.P):
            self._reset_numeric()
            self._sync_pose()
            return

        # b_a phase 2 exit: TIME ONLY. A P-covariance criterion was tried and
        # dropped: in replay, sd(b_a) collapsed 0.15 -> 0.05 within ~7 updates of
        # the car starting to creep (0.6-0.7 s), so phase 2 ended before it did
        # anything, while b_a itself was still swinging with the launch transient.
        if self.ba_phase == 2:
            self.ba_phase2_t += dt
            if self.ba_phase2_t >= self.cfg["ba_fast_window"]:
                self.ba_phase, self.ba_phase2_exit = 3, "time"
                self.ba_phase2_exit_t = self.ba_phase2_t

        # Floor the diagonal, but only AFTER it has been recorded: a negative
        # diagonal is the clearest evidence the covariance update went non-PSD
        # and quietly rounding it up to a tiny positive would hide that.
        self.P_diag_raw = [self.P[i][i] for i in range(EKF_N)]
        for i in range(EKF_N):
            if self.P[i][i] < EKF_P_DIAG_FLOOR:
                self.P[i][i] = EKF_P_DIAG_FLOOR

        self._sync_pose()

    def rho_br_thetag(self):
        """Correlation between b_r and theta_g. |rho| -> 1 is the filter saying
        the two states have become indistinguishable and are moving as a pair.
        The single most direct trade-off detector, and it costs one divide."""
        pbb = self.P[IX_B_R][IX_B_R]
        ptt = self.P[IX_THETA_G][IX_THETA_G]
        if pbb <= 0.0 or ptt <= 0.0:
            return 0.0
        return self.P[IX_B_R][IX_THETA_G] / math.sqrt(pbb * ptt)


class BydOdomNode(Node):
    def __init__(self, args):
        super().__init__("byd_odom_node")

        self.wheelbase = args.wheelbase
        self.steer_ratio = args.steer_ratio
        self.corrected_steer_ratio = args.corrected_steer_ratio
        self.gear_mode = args.gear_mode
        self.stale_s = args.stale_timeout
        self.frame_odom = args.odom_frame

        # ── DISABLED 2026-09-11 (see block above RECORD_TRACKS) ──
        # self.meas = Integrator()
        # ─────────────────────────────────────────────────────────────────────
        # DEPRECATED — raw/uncalibrated kinematic track. Kept commented out for
        # reference only. DO NOT re-enable in ./byd_drive.sh or any launch config.
        #
        # This is the ORIGINAL bicycle-model integrator: steer_ratio=13.11 with
        # NO steering-angle offset correction. 13.11 is the port's STATIC
        # control-tuned value (opendbc car/byd/values.py, also this node's
        # --steer-ratio default) — it is NOT a liveParameters online-calibrated
        # figure. It over-predicts yaw rate by ~8% per prior validation;
        # independently corroborated here on 2026-08-25, where switching to
        # 14.2 alone moved the yaw estimate -7.7%. It also ignores the steering
        # centre-offset bias, measured live on 2026-08-25 at 1.537 deg via
        # liveParameters.angleOffsetAverageDeg — a further -5.1% at 30 deg
        # steer, and strongly steer-dependent (-31% at 6 deg, -9% at 90 deg),
        # which is the signature of a constant bias and shows up as consistent
        # curvature error on gentle steering rather than a uniform scale error.
        #
        # Superseded by the `corrected` track (steer_ratio=14.2, offset-removed),
        # which is now the only steer-derived track this package publishes.
        # Left here, disabled, purely so the old behaviour can be reproduced for
        # comparison if ever needed again. `delta` and `yaw_rate_kin` are used
        # by nothing else, so this commenting-out is self-contained.
        # ─────────────────────────────────────────────────────────────────────
        # self.kin = Integrator()
        self.corrected = Integrator()
        # self.deltaref = Integrator()
        # self.yawsensor = Integrator()
        # self.startref = Integrator()
        # self.steerproxy = Integrator()
        # Two yaw-offset treatments, run side by side for a live A/B test.
        # Both read the SAME raw YAW_SENSOR rate; they differ only in which
        # estimate of the bias channel they subtract.
        self.yawoffs = Integrator()   # Track A: per-sample offset
        self.yawavg = Integrator()    # Track B: time-windowed mean offset
        # ekf_fused: the only track that FUSES rather than picking one heading
        # source. theta_g is seeded from THIS NODE'S geometry, not a literal,
        # so the EKF and `corrected` start from identical curvature -- a
        # hardcoded seed would put a gratuitous offset between them on tick one
        # and pollute exactly the A/B comparison these tracks exist to make.
        self.ekf_mode = args.ekf_mode
        # Seed: an explicit --ekf-theta-g wins; otherwise the newest qualifying
        # recorded run; otherwise corrected_steer_ratio * wheelbase. The starting
        # sd is 2.0 only for a qualified persisted seed, 5.0 otherwise.
        (theta_g_seed, self.ekf_theta_g_seed_source, self.ekf_theta_g_seed_run,
         p0_tg_sd, self._ekf_seed_note) = _select_theta_g_seed(
            args.ekf_theta_g, args.record_dir, args.wheelbase,
            args.corrected_steer_ratio)
        self.ekf_theta_g_seed = theta_g_seed
        self.ekf_theta_g_p0_sd = p0_tg_sd
        self.ekf_cfg = {
            "sigma_xy": 1e-3, "sigma_psi": 0.001, "sigma_r": 0.05,
            "sigma_a": args.ekf_sigma_a,
            "sigma_br": args.ekf_sigma_br,
            "sigma_thetag": args.ekf_sigma_thetag,
            "qv_mode": args.ekf_qv_mode,
            "qv_missing_mult": 100.0,
            "r_v": args.ekf_r_v, "r_r": args.ekf_r_r, "r_zupt": 1e-2,
            "theta_g_min": args.ekf_theta_g_min,
            "theta_g_max": args.ekf_theta_g_max,
            "joseph": args.ekf_joseph == "on",
            # exact bicycle model: tan() of the FRONT-WHEEL angle, which needs
            # the nominal ratio to convert from the steering-wheel angle
            "steer_ratio_nominal": args.corrected_steer_ratio,
            "tire_angle_max_rad": math.radians(45.0),
            "sigma_ba_fast": args.ekf_sigma_ba_fast,
            "sigma_ba_slow": args.ekf_sigma_ba_slow,
            "ba_fast_window": args.ekf_ba_fast_window,
            "p0_sd": (0.01, 0.01, 0.01, 1.0, 1.0, 0.01, p0_tg_sd,
                      args.ekf_ba_p0_sd),
        }
        # Two EKF tracks, IDENTICAL except the phase-3 b_a random-walk rate, fed
        # the same inputs every tick. ekf_ba_slow is the PRIMARY track: meta.json,
        # the persisted theta_g seed and the [ekf] line all describe it.
        self.ekf_ba_slow = EkfTrack(self.ekf_cfg, theta_g_seed)
        self.ekf_cfg_tight = dict(self.ekf_cfg)
        self.ekf_cfg_tight["sigma_ba_slow"] = args.ekf_sigma_ba_tight
        self.ekf_ba_tight = EkfTrack(self.ekf_cfg_tight, theta_g_seed)
        self.ekf_fused = self.ekf_ba_slow
        self.last_t = None
        # self._warned_no_yaw_rate = False   # measured track disabled
        # --- deltaref track state (independent of `corrected`) ---
        # Reconstructs steering angle from the CHANGE between consecutive raw
        # samples, so any CONSTANT sensor bias cancels: summing deltas
        # telescopes to steer(t) - steer(t0), and the bias appears in both
        # terms. The tradeoff is that it re-zeros ONCE at startup and holds
        # that reference for the session -- it does NOT track in-session
        # drift, which `corrected` does via liveParameters.angleOffsetAverageDeg.
        # self._deltaref_prev_steer_deg = None
        # self._deltaref_theta = 0.0
        # self._warned_deltaref_zero_ref = False
        self._warned_no_yaw_sensor = False
        # --- startref state -------------------------------------------------
        # Same delta accumulation as deltaref, but based at the REAL starting
        # reading instead of 0.0. That sum telescopes to steer(t0) + steer(t)
        # - steer(t0) = steer(t), so this track IS the raw uncorrected steering
        # angle. Proven sample-by-sample against a 55522-sample real log:
        # max difference 1.78e-15 deg (float noise, 5.6e13x below the sensor's
        # own 0.1 deg resolution). See tests/test_startref_equivalence.py.
        # self._startref_prev_steer_deg = None
        # self._startref_theta = 0.0
        self._warned_startref = False
        # self._warned_steerproxy = False
        self._warned_gear = False
        # --- Track A / Track B yaw-offset comparison state ------------------
        # YAW_OFFSET is the ECU's own live bias estimate. It is piecewise
        # constant and quantised to 1 LSB (0.002133 rad/s = 0.122 deg/s), and
        # that quantisation is exactly what Track A's residual drift is made
        # of: measured over an 11 min drive, per-sample subtraction left 10.2
        # deg RMS heading error while a best-fit constant left 2.9 deg. A mean
        # over the offset channel resolves below one LSB, which is the point
        # of Track B. Only the OFFSET is averaged -- averaging the RATE would
        # wash out real turning motion.
        self.yawavg_window_s = args.yawavg_window
        self.yawavg_warmup_s = args.yawavg_warmup
        self._yawavg_win = deque()     # (t, offset) inside the window
        self._yawavg_t0 = None         # first sample time, for warm-up
        self._warned_yawavg_warm = False

        # --- ekf_fused per-session state ---------------------------------
        # Mirrors the Track B block above: nothing here persists across runs.
        self._ekf_last_rx = None
        self._ekf_t0 = None
        self._ekf_accel_win = []
        self._ekf_accel_bias = None
        self._ekf_accel_bias_n = 0
        self._ekf_accel_bias_frozen = False      # set once, never cleared
        self.ekf_accel_bias_samples = args.ekf_accel_bias_samples
        self.ekf_accel_bias_min = args.ekf_accel_bias_min
        self.ekf_thetag_excite_min = args.ekf_thetag_excite_min
        self.ekf_update_mode = args.ekf_update_mode
        self.ekf_zupt = args.ekf_zupt == "on"
        self._logged_ba_phase2_exit = False
        self.ekf_diag_s = args.ekf_diag_s
        self._ekf_diag_last = time.monotonic()
        self._ekf_excite_win = deque()
        self._warned_ekf_accel_bias = False
        self._warned_ekf_singular = False
        self._warned_ekf_nonfinite = False
        self._warned_ekf_steer_clamp = False
        if args.ekf_accel_bias is not None:      # explicit override, e.g. replay
            self._ekf_accel_bias = float(args.ekf_accel_bias)
            self._ekf_accel_bias_frozen = True   # skip calibration entirely
        # --- run recording -------------------------------------------------
        # Rows are buffered in memory and written once, on shutdown or on
        # request. At 50 Hz an hour of driving is ~180k rows, which is a few
        # tens of MB as CSV -- cheap enough to keep, and far simpler than
        # streaming to disk while also meeting the tick deadline.
        self.record_dir = args.record_dir
        self._rec = []              # rows not yet flushed to disk
        self._rec_saved = False
        self._rec_total = 0         # rows written across the whole run
        self._rec_t_first = None
        self._rec_t_last = None
        self._csv_fh = None
        self._csv_w = None
        self._csv_path = None
        self._out_dir = None
        self._last_flush = time.monotonic()
        # --- live health state ---
        self._h_win = deque()       # (rx_mono, latency) inside HEALTH_WINDOW_S
        self._h_last_rx = None      # rx_mono of the previous DISTINCT sample
        self._h_gaps = 0
        self._h_max_gap = 0.0
        self._h_last_print = time.monotonic()
        self._h_t0 = time.monotonic()
        self._rec_start_wall = datetime.datetime.now()
        self._save_srv = self.create_service(
            Trigger, "/byd/save_record", self._srv_save_record)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # self.pub_odom_meas = self.create_publisher(Odometry, "/byd/odom_measured", qos)
        # self.pub_path_meas = self.create_publisher(Path, "/byd/path_measured", qos)
        # DEPRECATED (see block above)
        # self.pub_odom_kin = self.create_publisher(Odometry, "/byd/odom_kinematic", qos)
        # self.pub_path_kin = self.create_publisher(Path, "/byd/path_kinematic", qos)
        self.pub_odom_corr = self.create_publisher(Odometry, "/byd/odom_corrected", qos)
        self.pub_path_corr = self.create_publisher(Path, "/byd/path_corrected", qos)
        # self.pub_odom_dref = self.create_publisher(Odometry, "/byd/odom_deltaref", qos)
        # self.pub_path_dref = self.create_publisher(Path, "/byd/path_deltaref", qos)
        # self.pub_odom_yaws = self.create_publisher(Odometry, "/byd/odom_yawsensor", qos)
        # self.pub_path_yaws = self.create_publisher(Path, "/byd/path_yawsensor", qos)
        # self.pub_odom_sref = self.create_publisher(Odometry, "/byd/odom_startref", qos)
        # self.pub_path_sref = self.create_publisher(Path, "/byd/path_startref", qos)
        # self.pub_odom_sprx = self.create_publisher(Odometry, "/byd/odom_steerproxy", qos)
        # self.pub_path_sprx = self.create_publisher(Path, "/byd/path_steerproxy", qos)
        self.pub_odom_yoff = self.create_publisher(Odometry, "/byd/odom_yawoffset", qos)
        self.pub_path_yoff = self.create_publisher(Path, "/byd/path_yawoffset", qos)
        self.pub_odom_yavg = self.create_publisher(Odometry, "/byd/odom_yawavg", qos)
        self.pub_path_yavg = self.create_publisher(Path, "/byd/path_yawavg", qos)
        self.pub_odom_ekf = self.create_publisher(Odometry, "/byd/odom_ekf_ba_slow", qos)
        self.pub_path_ekf = self.create_publisher(Path, "/byd/path_ekf_ba_slow", qos)
        self.pub_odom_ekf_tight = self.create_publisher(Odometry, "/byd/odom_ekf_ba_tight", qos)
        self.pub_path_ekf_tight = self.create_publisher(Path, "/byd/path_ekf_ba_tight", qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        # self.path_meas = Path()
        # self.path_meas.header.frame_id = self.frame_odom
        # DEPRECATED (see block above)
        # self.path_kin = Path()
        # self.path_kin.header.frame_id = self.frame_odom
        self.path_corr = Path()
        self.path_corr.header.frame_id = self.frame_odom
        # self.path_dref = Path()
        # self.path_dref.header.frame_id = self.frame_odom
        # self.path_yaws = Path()
        # self.path_yaws.header.frame_id = self.frame_odom
        # self.path_sref = Path()
        # self.path_sref.header.frame_id = self.frame_odom
        # self.path_sprx = Path()
        # self.path_sprx.header.frame_id = self.frame_odom
        self.path_yoff = Path()
        self.path_yoff.header.frame_id = self.frame_odom
        self.path_yavg = Path()
        self.path_yavg.header.frame_id = self.frame_odom
        self.path_ekf = Path()
        self.path_ekf.header.frame_id = self.frame_odom
        self.path_ekf_tight = Path()
        self.path_ekf_tight.header.frame_id = self.frame_odom
        self.max_path_poses = args.max_path_poses
        # Path republish is decoupled from the odom/TF tick rate: a Path message
        # carries its whole history, so publishing it costs O(len(poses)) and, at
        # full tick rate, eventually overruns the timer period as the path grows.
        # Poses are still appended every tick — only the publish is throttled.
        self.path_publish_every_n = max(1, round(args.rate / args.path_publish_hz))
        self._tick_count = 0

        self.client = DeviceStreamClient(args.host, args.port, self.get_logger())
        self.client.start()

        self.timer = self.create_timer(1.0 / args.rate, self.tick)
        self.get_logger().info(
            f"byd_odom_node up: wheelbase={self.wheelbase} m. "
            f"Publishing /byd/*_measured (needs cereal yaw_rate) and /byd/*_corrected "
            f"(steer-derived, steer_ratio={self.corrected_steer_ratio}, angle-offset removed) "
            f"and /byd/*_deltaref (same ratio, delta-reconstructed angle) "
            f"and /byd/*_yawsensor (car's own CAN yaw sensor — no ratio, no offset) "
            f"and /byd/*_startref (raw uncorrected steer, ratio {self.corrected_steer_ratio} — expected to drift like the deprecated kinematic track). "
            f"The raw /byd/*_kinematic track (steer_ratio={self.steer_ratio}) is DEPRECATED "
            f"and not published. "
            f"odom+TF at {args.rate:.0f} Hz; Path republished every {self.path_publish_every_n} "
            f"ticks (~{args.rate / self.path_publish_every_n:.1f} Hz)."
        )
        # Separate line: the block above is already stale about three disabled
        # tracks, and correcting it is a different change from adding this one.
        if self.ekf_mode == "fused":
            _seed_src = "%s, P0 sd %.1f, %s" % (
                self.ekf_theta_g_seed_source, self.ekf_theta_g_p0_sd,
                self._ekf_seed_note)
            self.get_logger().info(
                f"[ekf] EKF ON, two tracks identical except phase-3 sigma_ba: "
                f"ekf_ba_slow ({args.ekf_sigma_ba_slow}) -> /byd/*_ekf_ba_slow, "
                f"ekf_ba_tight ({args.ekf_sigma_ba_tight}) -> /byd/*_ekf_ba_tight; "
                f"b_a phase 2 lasts {args.ekf_ba_fast_window:.0f} s from first motion. "
                f"theta_g seed {theta_g_seed:.3f} ({_seed_src}), "
                f"Q_v {args.ekf_qv_mode}, update on {args.ekf_update_mode} samples, "
                f"ZUPT {args.ekf_zupt}, Joseph {args.ekf_joseph}. "
                f"R_v={args.ekf_r_v} m/s matches the 0.037-0.048 m/s distinct-sample "
                f"wheel-speed noise. The z_v innovations are ~0.99 autocorrelated "
                f"(a systematic accel/wheel-speed scale mismatch, not white noise), "
                f"so NIS running above 2 is expected, not a fault."
            )
        else:
            self.get_logger().info("[ekf] ekf_fused OFF (--ekf-mode off)")

    def _publish_track(self, integ: Integrator, path_msg: Path, pub_odom, pub_path,
                        child_frame: str, stamp, v_ms: float, yaw_rate: float,
                        publish_path: bool = True):
        quat = yaw_to_quaternion(integ.yaw)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.frame_odom
        tf.child_frame_id = child_frame
        tf.transform.translation.x = integ.x
        tf.transform.translation.y = integ.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation = quat
        self.tf_broadcaster.sendTransform(tf)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_odom
        odom.child_frame_id = child_frame
        odom.pose.pose.position.x = integ.x
        odom.pose.pose.position.y = integ.y
        odom.pose.pose.orientation = quat
        odom.twist.twist.linear.x = v_ms
        odom.twist.twist.angular.z = yaw_rate
        pub_odom.publish(odom)

        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = self.frame_odom
        ps.pose = odom.pose.pose
        path_msg.poses.append(ps)
        if len(path_msg.poses) > self.max_path_poses:
            path_msg.poses = path_msg.poses[-self.max_path_poses:]
        path_msg.header.stamp = stamp
        # Throttled: see path_publish_every_n. Gated on the per-tick counter (not a
        # per-track one) so both tracks' Paths go out on the same ticks, in sync.
        if publish_path and (self._tick_count % self.path_publish_every_n) == 0:
            pub_path.publish(path_msg)

    def tick(self):
        item = self.client.get_latest()
        if item is None:
            return
        d, rx_mono = item

        if is_stale(time.monotonic(), rx_mono, self.stale_s):
            return

        # EKF sample freshness. The node ticks at 50 Hz but the device streams
        # at ~10, so get_latest() hands back the same payload several times.
        # _ekf_last_rx is NOT assigned here: it is assigned inside the EKF block
        # after the predict has run, so a tick that returns at the dt guard
        # below does not consume the sample.
        ekf_fresh = (rx_mono != self._ekf_last_rx)

        v_ms = float(d.get("v_kmh", 0.0)) / 3.6
        steer_deg = float(d.get("steer_deg", 0.0))

        gear_raw = str(d.get("gear", "")).strip().lower()
        if self.gear_mode == "reverse-only":
            # Reverse is the only gear that changes anything: it flips the sign
            # so the path retraces instead of folding forward. Everything else,
            # INCLUDING park, neutral and an unrecognised or missing value,
            # integrates as forward motion. That is deliberate — this mode
            # trusts wheel speed to be ~0 when stopped rather than trusting
            # gear to say so, and therefore never freezes the path on a gear
            # value the server failed to forward. No warning fires here: an
            # unknown gear is not an error in this mode, it is the default case.
            gear_mult = -1.0 if gear_raw == "reverse" else 1.0
        elif self.gear_mode == "off":
            # Pre-GEAR_SIGN behaviour. Speed is an unsigned forward magnitude
            # and gear is ignored entirely, so a reverse manoeuvre integrates
            # as forward motion and folds the path back on itself.
            gear_mult = 1.0
        else:  # "signed" — the default
            gear_mult = GEAR_SIGN.get(gear_raw, 0.0)
            if gear_raw not in GEAR_SIGN and not self._warned_gear:
                self.get_logger().warn(
                    f"gear {gear_raw!r} is not one of {sorted(GEAR_SIGN)} — treating it as "
                    "no translation. The path will hold position while this persists; if the "
                    "car is actually moving, byd_cereal_server.py is not forwarding gear. "
                    "--gear-mode reverse-only ignores gear except for reverse."
                )
                self._warned_gear = True
        v_signed = v_ms * gear_mult

        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9
        self.last_t, dt = advance_clock(self.last_t, t)
        if dt is None:
            return

        v_for_integration = speed_gate(v_signed)

        # --- DEPRECATED uncorrected kinematic yaw rate (see block in __init__) ---
        # delta = tire_angle_rad(steer_deg, self.steer_ratio)
        # yaw_rate_kin = heading_rate_rad_s(v_for_integration, delta, self.wheelbase)

        # --- Corrected kinematic yaw rate: same bicycle model as `kin`, but with
        # (1) a separately-configurable steer ratio, and (2) the steering-angle
        # sensor's centre bias removed. Both are independent error sources; `kin`
        # is deliberately left uncorrected so the three tracks stay comparable.
        angle_offset_deg = float(d.get("angle_offset_deg", 0.0) or 0.0)
        effective_steer_deg = steer_deg - angle_offset_deg
        delta_corr = tire_angle_rad(effective_steer_deg, self.corrected_steer_ratio)
        yaw_rate_corr = heading_rate_rad_s(v_for_integration, delta_corr, self.wheelbase)

        # # --- deltaref yaw rate: SAME ratio as `corrected`, different bias handling ---
        # # Independent code path; reads nothing from and writes nothing to the
        # # corrected track. See the state comments in __init__ for the tradeoff.
        # if self._deltaref_prev_steer_deg is None:
            # # First valid sample: capture the zero reference. No delta exists yet,
            # # so theta stays 0.0 and this tick contributes no rotation.
            # self._deltaref_prev_steer_deg = steer_deg
            # if not self._warned_deltaref_zero_ref:
                # self.get_logger().warn(
                    # "deltaref track: steering-angle zero-reference captured at startup "
                    # f"(raw steer_deg={steer_deg:.2f}) — ASSUMES THE WHEELS ARE STRAIGHT "
                    # "RIGHT NOW. If they were not, that error carries as a residual bias "
                    # "for this whole session. deltaref is immune to a CONSTANT sensor "
                    # "bias but does NOT track in-session drift; the `corrected` track "
                    # "does, via liveParameters.angleOffsetAverageDeg."
                # )
                # self._warned_deltaref_zero_ref = True
        # else:
            # self._deltaref_theta += steer_deg - self._deltaref_prev_steer_deg
            # self._deltaref_prev_steer_deg = steer_deg
        # delta_dref = tire_angle_rad(self._deltaref_theta, self.corrected_steer_ratio)
        # yaw_rate_dref = heading_rate_rad_s(v_for_integration, delta_dref, self.wheelbase)

        # # --- startref: delta accumulation based at the REAL startup reading ----
        # # Identical in form to deltaref except for the base. Because the sum
        # # telescopes, theta == steer_deg every tick: this track is the RAW,
        # # UNCORRECTED steering angle, i.e. exactly what the deprecated kinematic
        # # track integrated (only at ratio 14.2 instead of 13.11). It is expected
        # # to reproduce that track's drift, and is published for visual comparison
        # # rather than as a candidate correction.
        # if self._startref_prev_steer_deg is None:
            # self._startref_prev_steer_deg = steer_deg
            # self._startref_theta = steer_deg          # base = real reading, NOT 0.0
            # if not self._warned_startref:
                # self.get_logger().warn(
                    # f"startref track: based at the RAW startup reading "
                    # f"(steer_deg={steer_deg:.2f}), then accumulating deltas. That sum "
                    # "telescopes to steer(t), so this track IS the raw uncorrected "
                    # "steering angle -- it applies NO offset correction and is expected, "
                    # "by construction, to drift the same way the deprecated kinematic "
                    # "track did. Published for comparison, not as a fix."
                # )
                # self._warned_startref = True
        # else:
            # self._startref_theta += steer_deg - self._startref_prev_steer_deg
            # self._startref_prev_steer_deg = steer_deg
        # delta_sref = tire_angle_rad(self._startref_theta, self.corrected_steer_ratio)
        # yaw_rate_sref = heading_rate_rad_s(v_for_integration, delta_sref, self.wheelbase)

        # # --- steerproxy: tire angle used DIRECTLY as heading -------------------
        # # Structurally unlike every other track: there is no rate integration at
        # # all. heading := tire_angle(steer) each tick, overwriting whatever came
        # # before, so no turn history is retained.
        # heading_sprx = tire_angle_rad(steer_deg, self.corrected_steer_ratio)
        # if not self._warned_steerproxy:
            # self.get_logger().warn(
                # "steerproxy track: heading is NOT accumulated — it is set to the "
                # "current tire angle every tick. HYPOTHESIS UNDER TEST: it will snap "
                # "back toward 'facing straight' whenever the wheel returns to centre, "
                # "regardless of how far the car actually turned earlier, and can never "
                # "represent more than max_steer/ratio of heading. Checked against the "
                # "2026-09-05 3-loop log: over 30 straight-after-turn segments this "
                # "method averaged +0.06 deg while the accumulated heading averaged "
                # "+530.3 deg. Published to see that failure, not as a candidate."
            # )
            # self._warned_steerproxy = True

        # --- yawsensor: the car's OWN physical yaw-rate sensor (CAN 546) --------
        # No steer ratio, no centre offset, no bicycle model. This is a direct
        # angular-rate measurement; the other tracks all INFER rotation from
        # steering geometry. Validated on a real drive 2026-09-03: sign matched
        # steering 45/45 samples, ~0 on straights, 22 deg/s peak at a tight
        # car-park turn. Distinct from `measured`, which reads cs.yawRate and is
        # structurally always 0 because the BYD port never assigns it.
        raw_yaw_sensor, off_raw, yaw_sensor_ok = sanitize_can_sample(
            d.get("yaw_sensor_rate"), d.get("yaw_sensor_offset", 0.0),
            d.get("yaw_sensor_ok", False))
        # if raw_yaw_sensor is None or not yaw_sensor_ok:
            # if not self._warned_no_yaw_sensor:
                # self.get_logger().warn(
                    # "yawsensor track: cereal stream has no valid 'yaw_sensor_rate' — "
                    # "the track will hold heading. Update byd_cereal_server.py "
                    # "(needs the YAW_SENSOR CANParser block) or check can_valid."
                # )
                # self._warned_no_yaw_sensor = True
            # yaw_rate_yaws = 0.0
        # else:
            # yaw_rate_yaws = float(raw_yaw_sensor)
            # # Same standstill gate as the other tracks: below the speed floor we
            # # are not travelling, so integrating sensor noise only adds drift.
            # if abs(v_ms) < MIN_SPEED_MS:
                # yaw_rate_yaws = 0.0

        # --- Track A / Track B: two ways of removing the yaw-sensor bias ----
        # Distinct from the `yawsensor` track above, which subtracts NOTHING
        # and therefore carries the full ~0.24 deg/s bias.
        if raw_yaw_sensor is None or not yaw_sensor_ok:
            # Relocated here when the `yawsensor` track was disabled: this is now
            # the only place the missing-sensor condition is reported, and it
            # zeroes BOTH remaining yaw-sensor tracks, so it must stay loud.
            if not self._warned_no_yaw_sensor:
                self.get_logger().warn(
                    "cereal stream has no valid 'yaw_sensor_rate' — Methods A and B "
                    "will BOTH hold heading, leaving `corrected` with no reference "
                    "to compare against. Update byd_cereal_server.py (needs the "
                    "YAW_SENSOR CANParser block) or check can_valid."
                )
                self._warned_no_yaw_sensor = True
            yaw_rate_yoff = 0.0
            yaw_rate_yavg = 0.0
            ekf_z_r = None
        else:
            # Track A: per-sample subtraction -- the same arithmetic carstate.py
            # now does for CS.yawRate, recomputed here from the raw fields so
            # the two methods are compared on equal footing inside one node.
            yaw_rate_yoff = yaw_rate_offset_corrected(raw_yaw_sensor, off_raw)
            # Snapshot for the EKF BEFORE the MIN_SPEED_MS gate further down
            # zeroes yaw_rate_yoff. Tracks A/B want the gate: they integrate the
            # rate straight into heading, so standstill noise is pure drift. The
            # EKF must NOT have it. At v=0 the prediction sets r = v*steer/theta_g
            # = 0 exactly and theta_g's partial is exactly 0 too, so the residual
            # collapses to (z_r - b_r): a clean, single-state observation of the
            # residual gyro bias, and the only regime in the whole drive where
            # b_r is decoupled from theta_g. Gating would replace that with a
            # fabricated zero that actively drives b_r toward 0 at every stop.
            ekf_z_r = yaw_rate_yoff
            # Track B: simple (unweighted) mean of the offset channel over a
            # trailing time window. Time-based rather than sample-count based
            # so the averaging length is unaffected by tick-rate jitter. The
            # window always holds the current sample, so the mean is seeded
            # from real data and is never an empty or zero average.
            self._yawavg_win.append((t, off_raw))
            cutoff = t - self.yawavg_window_s
            while len(self._yawavg_win) > 1 and self._yawavg_win[0][0] < cutoff:
                self._yawavg_win.popleft()
            off_avg = sum(o for _, o in self._yawavg_win) / len(self._yawavg_win)
            yaw_rate_yavg = raw_yaw_sensor - off_avg
            if self._yawavg_t0 is None:
                self._yawavg_t0 = t
            yaw_rate_yoff = standstill_gate(yaw_rate_yoff, v_ms)
            yaw_rate_yavg = standstill_gate(yaw_rate_yavg, v_ms)
        # Warm-up gates only Track B's PATH publication, not its integration:
        # a Path carries its whole history, so the line simply appears complete
        # once the window has filled. Both tracks therefore integrate from the
        # same first sample and stay time-aligned.
        # CAVEAT: if the car is already moving during warm-up, Track B's offset
        # estimate is averaged over fewer samples and is correspondingly less
        # settled; any heading error from that persists for the session.
        yavg_warm = (self._yawavg_t0 is not None
                     and (t - self._yawavg_t0) >= self.yawavg_warmup_s)
        if yavg_warm and not self._warned_yawavg_warm:
            self.get_logger().info(
                f"yawavg (Track B) warm: {self.yawavg_warmup_s:.0f}s seeded, "
                f"{len(self._yawavg_win)} samples in a {self.yawavg_window_s:.0f}s "
                f"window; publishing /byd/path_yawavg.")
            self._warned_yawavg_warm = True

        # --- ekf_fused ------------------------------------------------------
        if self.ekf_mode == "fused":
            accel_raw = _fnum(d.get("accel_long"))
            accel_off = _fnum(d.get("accel_long_offset"))
            accel_ok = bool(d.get("accel_ok", False))
            # accel_long and accel_long_offset are stored un-subtracted on
            # purpose (the ECU offset channel steps by 1 LSB over hours), so the
            # subtraction happens here -- and the session bias below is computed
            # from this SAME expression. Calibrating on raw accel_long while
            # also subtracting the offset at runtime would double-correct.
            if accel_raw is not None and accel_off is not None and accel_ok:
                accel_raw = accel_raw - accel_off
            else:
                accel_raw = None

            # ONE window at session start, then frozen forever. The rest offset
            # is dominated by PARKING SLOPE, not sensor bias: flipping the car
            # 180 deg in the same spot moved it +3.44 -> -1.78 LSB, i.e. a
            # 0.41 deg gravity term against only ~0.83 LSB of real bias.
            # Re-arming at a later standstill would measure whatever hill the
            # car stopped on and then subtract that hill for the rest of the
            # drive. Hence the guard.
            if not self._ekf_accel_bias_frozen:          # <-- GUARD, read first
                if (accel_raw is not None and accel_ok
                        and abs(v_ms) < MIN_SPEED_MS and ekf_fresh):
                    self._ekf_accel_win.append(accel_raw)

                # window closes on EITHER the target count OR first real motion
                if (len(self._ekf_accel_win) >= self.ekf_accel_bias_samples
                        or abs(v_ms) >= MIN_SPEED_MS):
                    if len(self._ekf_accel_win) >= self.ekf_accel_bias_min:
                        self._ekf_accel_bias = (sum(self._ekf_accel_win)
                                                / len(self._ekf_accel_win))
                    else:
                        self._ekf_accel_bias = 0.0
                        if not self._warned_ekf_accel_bias:
                            self.get_logger().warn(
                                "[ekf] accel zero-bias: only %d stationary samples "
                                "(need %d) -- falling back to 0.0 for this session. "
                                "Longitudinal prediction will carry the parking "
                                "slope as an apparent acceleration."
                                % (len(self._ekf_accel_win), self.ekf_accel_bias_min))
                            self._warned_ekf_accel_bias = True
                    self._ekf_accel_bias_n = len(self._ekf_accel_win)
                    self._ekf_accel_bias_frozen = True    # set once
                    self._ekf_accel_win = []              # drop the buffer

            # used every tick; 0.0 until the window closes
            _bias = self._ekf_accel_bias if self._ekf_accel_bias is not None else 0.0
            ekf_accel_corr = (accel_raw - _bias) if accel_raw is not None else None

            # steer_rad MUST be the offset-corrected angle. angle_offset_deg is
            # an ADDITIVE bias; theta_g is MULTIPLICATIVE and structurally
            # cannot absorb it. Feeding raw steer would hand the filter an error
            # no state can represent, and b_r/theta_g would thrash fitting it.
            ekf_steer_rad = math.radians(effective_steer_deg)

            # z_v: v_signed (gear-signed), NOT v_for_integration. The latter is
            # a hard-thresholded dead-reckoning input; as a MEASUREMENT its step
            # to zero at 0.05 m/s is a discontinuity the filter could only
            # reconcile by inventing a large deceleration.
            gear_valid = gear_raw in GEAR_SIGN
            # An unrecognised gear yields v_signed = 0 for a moving car --
            # deliberate path-freezing for the dead-reckoners, but a fabricated
            # stop measurement for the EKF. Skip the row entirely. A genuine
            # park/neutral is different: there 0 is TRUE and should be fed.
            ekf_z_v = v_signed if gear_valid else None

            ekf_do_update = (ekf_fresh or self.ekf_update_mode == "every-tick")
            ekf_zupt_now = (self.ekf_zupt and gear_valid
                            and abs(v_ms) < MIN_SPEED_MS)

            # theta_g and b_r both reach the measurement through one scalar
            # residual, with sensitivities -v*delta/theta_g^2 and 1. They are
            # separable ONLY when the excitation v*delta varies. Below the
            # threshold, hold theta_g rather than let a small yaw-bias error be
            # converted into a large curvature-gain step that nothing pulls back.
            ekf_excite = abs(v_signed * ekf_steer_rad)
            ekf_lock_tg = ekf_excite < self.ekf_thetag_excite_min
            self._ekf_excite_win.append((t, ekf_excite))
            while self._ekf_excite_win and self._ekf_excite_win[0][0] < t - 10.0:
                self._ekf_excite_win.popleft()

            # Heading integration is suppressed at standstill so psi cannot
            # creep while parked, while z_r stays ungated so b_r still learns.
            ekf_integrate_psi = abs(v_ms) >= MIN_SPEED_MS

            _moving = abs(v_ms) >= MIN_SPEED_MS
            for _trk in (self.ekf_ba_slow, self.ekf_ba_tight):
                _trk.step(
                    dt, ekf_steer_rad, ekf_accel_corr, ekf_z_v, ekf_z_r,
                    ekf_do_update, ekf_lock_tg, ekf_integrate_psi, ekf_zupt_now,
                    _moving)
            if (self.ekf_ba_slow.ba_phase2_exit != "never"
                    and not self._logged_ba_phase2_exit):
                self.get_logger().info(
                    "[ekf] b_a phase 2 -> 3 after %.1f s (time window): "
                    "b_a slow %+.4f, tight %+.4f m/s^2"
                    % (self.ekf_ba_slow.ba_phase2_exit_t,
                       self.ekf_ba_slow.X[IX_B_A], self.ekf_ba_tight.X[IX_B_A]))
                self._logged_ba_phase2_exit = True

            if ekf_fresh:
                self._ekf_last_rx = rx_mono
            if self._ekf_t0 is None:
                self._ekf_t0 = t
            if self.ekf_fused.singular and not self._warned_ekf_singular:
                self.get_logger().warn(
                    "[ekf] innovation covariance near-singular -- update skipped. "
                    "Check R_v/R_r are not both ~0.")
                self._warned_ekf_singular = True
            if self.ekf_fused.resets and not self._warned_ekf_nonfinite:
                self.get_logger().warn(
                    "[ekf] non-finite state caught; estimator states and P reset, "
                    "pose PRESERVED. Check the input stream for NaN/Inf.")
                self._warned_ekf_nonfinite = True
            if self.ekf_fused.steer_clamps and not self._warned_ekf_steer_clamp:
                self.get_logger().warn(
                    "[ekf] front-wheel angle exceeded +/-%.0f deg and was clamped "
                    "for the tan() model. Max seen in any recording is 34.6 deg, "
                    "so check steer_deg / angle_offset_deg / the steer ratio."
                    % math.degrees(self.ekf_cfg["tire_angle_max_rad"]))
                self._warned_ekf_steer_clamp = True

        # # --- Measured yaw rate: from the car's own sensor, via cereal ---
        # raw_yaw_rate = d.get("yaw_rate")
        # if raw_yaw_rate is None:
            # if not self._warned_no_yaw_rate:
                # self.get_logger().warn(
                    # "cereal stream has no 'yaw_rate' field — the MEASURED track "
                    # "will hold its heading constant (NOT falling back to the "
                    # "kinematic estimate) until the server is updated to emit it."
                # )
                # self._warned_no_yaw_rate = True
            # yaw_rate_meas = 0.0
        # else:
            # yaw_rate_meas = float(raw_yaw_rate)
            # if abs(v_ms) < MIN_SPEED_MS:
                # yaw_rate_meas = 0.0

        # self.meas.step(v_for_integration, yaw_rate_meas, dt)
        # self.kin.step(v_for_integration, yaw_rate_kin, dt)   # DEPRECATED
        self.corrected.step(v_for_integration, yaw_rate_corr, dt)
        # self.deltaref.step(v_for_integration, yaw_rate_dref, dt)
        # self.yawsensor.step(v_for_integration, yaw_rate_yaws, dt)
        # self.startref.step(v_for_integration, yaw_rate_sref, dt)
        # self.steerproxy.step_absolute_heading(v_for_integration, heading_sprx, dt)
        self.yawoffs.step(v_for_integration, yaw_rate_yoff, dt)
        self.yawavg.step(v_for_integration, yaw_rate_yavg, dt)

        stamp = now.to_msg()
        self._tick_count += 1
        # self._publish_track(self.meas, self.path_meas, self.pub_odom_meas, self.pub_path_meas,
        #                      "base_link_measured", stamp, v_for_integration, yaw_rate_meas)
        # DEPRECATED — no longer published:
        # self._publish_track(self.kin, self.path_kin, self.pub_odom_kin, self.pub_path_kin,
        #                      "base_link_kinematic", stamp, v_for_integration, yaw_rate_kin)
        self._publish_track(self.corrected, self.path_corr, self.pub_odom_corr, self.pub_path_corr,
                             "base_link_corrected", stamp, v_for_integration, yaw_rate_corr)
        # self._publish_track(self.deltaref, self.path_dref, self.pub_odom_dref, self.pub_path_dref,
        #                      "base_link_deltaref", stamp, v_for_integration, yaw_rate_dref)
        # self._publish_track(self.yawsensor, self.path_yaws, self.pub_odom_yaws, self.pub_path_yaws,
        #                      "base_link_yawsensor", stamp, v_for_integration, yaw_rate_yaws)
        # self._publish_track(self.startref, self.path_sref, self.pub_odom_sref, self.pub_path_sref,
        #                      "base_link_startref", stamp, v_for_integration, yaw_rate_sref)
        # self._publish_track(self.steerproxy, self.path_sprx, self.pub_odom_sprx, self.pub_path_sprx,
        #                      "base_link_steerproxy", stamp, v_for_integration, 0.0)
        self._publish_track(self.yawoffs, self.path_yoff, self.pub_odom_yoff, self.pub_path_yoff,
                             "base_link_yawoffset", stamp, v_for_integration, yaw_rate_yoff)
        self._publish_track(self.yawavg, self.path_yavg, self.pub_odom_yavg, self.pub_path_yavg,
                             "base_link_yawavg", stamp, v_for_integration, yaw_rate_yavg,
                             publish_path=yavg_warm)
        if self.ekf_mode == "fused":
            # v_ms/yaw_rate here are the FUSED ESTIMATES, not the measurements:
            # _publish_track puts them into odom.twist, so RViz and any
            # downstream consumer see the estimator's own velocity for free.
            self._publish_track(self.ekf_ba_slow, self.path_ekf,
                                self.pub_odom_ekf, self.pub_path_ekf,
                                "base_link_ekf_ba_slow", stamp,
                                self.ekf_ba_slow.X[IX_V], self.ekf_ba_slow.X[IX_R])
            self._publish_track(self.ekf_ba_tight, self.path_ekf_tight,
                                self.pub_odom_ekf_tight, self.pub_path_ekf_tight,
                                "base_link_ekf_ba_tight", stamp,
                                self.ekf_ba_tight.X[IX_V], self.ekf_ba_tight.X[IX_R])

        row = [t, time.time(), float(d.get("v_kmh", 0.0)), v_signed, v_for_integration,
               steer_deg, gear_raw, angle_offset_deg,
               d.get("yaw_sensor_rate"), d.get("yaw_sensor_offset"),
               int(bool(d.get("yaw_sensor_ok", False))),
               yaw_rate_yoff, yaw_rate_yavg]
        for attr, _ in RECORD_TRACKS:
            tr = getattr(self, attr)
            row += [tr.x, tr.y, math.degrees(tr.yaw)]
        row += [rx_mono, d.get("ts"), d.get("cereal_ts")]
        # ACCEL_SENSOR (CAN 547), appended AFTER the timing trio so that every
        # column already in use keeps its index. Raw and OFFSET are recorded
        # separately and un-subtracted, matching how the cereal server exposes
        # them: the offset channel steps by 1 LSB over hours/power cycles, so a
        # pre-subtracted value would hide that. accel_ok is 0 until 547 decodes.
        row += [d.get("accel_long"), d.get("accel_long_offset"),
                d.get("accel_lat"), d.get("accel_lat_offset"),
                int(bool(d.get("accel_ok", False)))]
        # ekf_fused scalars. Pose (x/y/yaw_deg) comes from RECORD_TRACKS; these
        # are the remaining states, the P diagonal and the per-tick diagnostics.
        # Appended last, same rule as the accel block: every column already in
        # use keeps its index.
        if self.ekf_mode == "fused":
            _e = self.ekf_fused
            # P diagonal as RAW VARIANCES, pre-floor. A negative entry is the
            # clearest evidence the covariance update went non-PSD, and taking
            # the sqrt for readability would mask it as NaN.
            _pd = getattr(_e, "P_diag_raw", [_e.P[i][i] for i in range(EKF_N)])
            row += [_e.X[IX_V], _e.X[IX_R], _e.X[IX_B_R], _e.X[IX_THETA_G],
                    _pd[IX_X], _pd[IX_Y], _pd[IX_PSI], _pd[IX_V], _pd[IX_R],
                    _pd[IX_B_R], _pd[IX_THETA_G],
                    _e.last_innov_v, _e.last_innov_r, _e.last_nis,
                    _e.last_updated, ekf_accel_corr, int(bool(_e.ok))]
            _t = self.ekf_ba_tight
            _pdt = getattr(_t, "P_diag_raw", [_t.P[i][i] for i in range(EKF_N)])
            row += [_e.X[IX_B_A], _pd[IX_B_A], _t.X[IX_B_A], _pdt[IX_B_A],
                    _e.ba_phase]
        else:
            row += [None] * 22
        self._rec.append(row)
        self._health_update(rx_mono, d.get("ts"))
        if self.ekf_mode == "fused":
            self._ekf_diag()
        if (len(self._rec) >= CSV_FLUSH_ROWS
                or (time.monotonic() - self._last_flush) >= CSV_FLUSH_S):
            self._flush_rows()

    def _ekf_diag(self):
        """Throttled [ekf] line, mirroring [health]'s format discipline.

        Everything printed here is also in the CSV (P diagonal, ekf_nis), so
        this is a summary of something reconstructable offline, never the only
        copy. rho is the direct b_r/theta_g trade-off detector: |rho| -> 1 means
        the two states have become indistinguishable and are moving as a pair.
        """
        now = time.monotonic()
        if (now - self._ekf_diag_last) < self.ekf_diag_s:
            return
        self._ekf_diag_last = now
        e = self.ekf_fused
        tg = e.X[IX_THETA_G]
        br = e.X[IX_B_R]
        tg_sd = math.sqrt(max(e.P[IX_THETA_G][IX_THETA_G], 0.0))
        br_sd = math.sqrt(max(e.P[IX_B_R][IX_B_R], 0.0))
        rho = e.rho_br_thetag()
        nis = (e.nis_sum / e.nis_n) if e.nis_n else float("nan")
        # Low VARIANCE of the excitation, not low magnitude, is what makes the
        # pair unobservable -- a big but perfectly constant v*delta is just as
        # degenerate as a small one.
        exc = [val for _, val in self._ekf_excite_win]
        exc_sd = 0.0
        if len(exc) > 2:
            mean = sum(exc) / len(exc)
            exc_sd = math.sqrt(sum((val - mean) ** 2 for val in exc) / len(exc))
        flags = []
        if abs(rho) > 0.9:
            flags.append("RHO-HIGH")
        # Fixed at the 0.25 this flag effectively used before excite_min moved
        # from 0.5 to 4.4, so the log line keeps its meaning.
        if exc_sd < 0.25:
            flags.append("EXCITE-LOW")
        if e.theta_g_clamps:
            flags.append("CLAMPED")
        self.get_logger().info(
            "[ekf] theta_g %.2f (seed %.2f, sd %.3f) | b_r %+.6f (sd %.2e) "
            "| b_a slow %+.4f tight %+.4f (ph %d) "
            "| rho %+.2f | NIS %.2f (n=%d) | upd %d/%d | clamp %d %s"
            % (tg, self.ekf_theta_g_seed, tg_sd, br, br_sd,
               e.X[IX_B_A], self.ekf_ba_tight.X[IX_B_A], e.ba_phase,
               rho, nis, e.nis_n,
               e.updates, e.updates + e.skipped, e.theta_g_clamps,
               " ".join(flags)))

    def _health_update(self, rx_mono, srv_ts):
        """Track stream health from data the node already has. No new I/O.

        Only DISTINCT samples count: the node ticks at 50 Hz while the server
        sends at 10 Hz, so get_latest() hands back the same payload several
        times and counting every tick would report a rate that is not real.
        """
        if rx_mono == self._h_last_rx:
            return
        now = time.monotonic()
        if self._h_last_rx is not None:
            gap = rx_mono - self._h_last_rx
            if gap > HEALTH_GAP_S:
                self._h_gaps += 1
                self._h_max_gap = max(self._h_max_gap, gap)
                # Fired at the moment it happens, not folded into the next
                # summary line, so it stands out in a live scroll.
                self.get_logger().warn(
                    "!! GAP %.2fs !! at %s — stream delivered nothing "
                    "(gap #%d this session)"
                    % (gap, time.strftime("%H:%M:%S"), self._h_gaps))
        self._h_last_rx = rx_mono
        lat = (rx_mono - float(srv_ts)) if srv_ts is not None else None
        self._h_win.append((rx_mono, lat))
        cutoff = rx_mono - HEALTH_WINDOW_S
        while len(self._h_win) > 1 and self._h_win[0][0] < cutoff:
            self._h_win.popleft()

        if (now - self._h_last_print) < HEALTH_PRINT_S:
            return
        self._h_last_print = now
        span = self._h_win[-1][0] - self._h_win[0][0]
        hz = (len(self._h_win) - 1) / span if span > 0 else 0.0
        # The two clocks have an arbitrary constant offset, so absolute
        # latency is meaningless; the EXCESS over the window median is not.
        lats = sorted(x for _, x in self._h_win if x is not None)
        if len(lats) >= 4:
            med = lats[len(lats) // 2]
            p95 = lats[int(0.95 * (len(lats) - 1))]
            lat_s = "%+.3fs" % (p95 - med)
        else:
            lat_s = "n/a"
        self.get_logger().info(
            "[health] %5.1f Hz | gaps %d (max %.2fs) | lat p95 %s | %.0fs"
            % (hz, self._h_gaps, self._h_max_gap, lat_s, now - self._h_t0))

    def _open_csv(self):
        """Create the run directory and open odom.csv on the first flush."""
        ts = self._rec_start_wall
        base = os.path.join(self.record_dir, ts.strftime("%Y"), ts.strftime("%m"),
                            ts.strftime("%d"), ts.strftime("%H%M"))
        # Two runs can start inside the same minute; never overwrite a drive.
        out = base
        n = 2
        while os.path.exists(out):
            out = "%s_%d" % (base, n)
            n += 1
        os.makedirs(out, exist_ok=True)
        self._out_dir = out
        self._csv_path = os.path.join(out, "odom.csv")
        self._csv_fh = open(self._csv_path, "w", newline="")
        self._csv_w = csv.writer(self._csv_fh)
        self._csv_w.writerow(self._record_columns())
        self._csv_fh.flush()
        self.get_logger().info("recording to %s (written as the drive runs)"
                               % self._csv_path)

    def _flush_rows(self):
        """Append buffered rows to disk. Never fatal: a failed write must not
        take the node down mid-drive."""
        self._last_flush = time.monotonic()
        if not self._rec:
            return
        try:
            if self._csv_fh is None:
                self._open_csv()
            if self._rec_t_first is None:
                self._rec_t_first = self._rec[0][0]
            self._rec_t_last = self._rec[-1][0]
            self._csv_w.writerows(self._rec)
            self._csv_fh.flush()
            self._rec_total += len(self._rec)
            self._rec = []
        except OSError as e:
            self.get_logger().error("CSV flush failed: %s" % e)

    def _record_columns(self):
        cols = ["t_mono", "t_wall_unix", "v_kmh", "v_signed_ms", "v_integrated_ms",
                "steer_deg", "gear", "angle_offset_deg",
                "yaw_sensor_rate", "yaw_sensor_offset", "yaw_sensor_ok",
                "trackA_yaw_rate", "trackB_yaw_rate"]
        for _, name in RECORD_TRACKS:
            cols += [name + "_x", name + "_y", name + "_yaw_deg"]
        # Timing instrumentation, appended last so every existing column keeps
        # its position. rx_mono is this laptop's clock when the payload came off
        # the socket; srv_ts and srv_cereal_ts are the DEVICE's clock at send and
        # at last fresh carState. The two clocks are unrelated in absolute terms,
        # so compare DELTAS, never the raw values.
        cols += ["rx_mono", "srv_ts", "srv_cereal_ts"]
        cols += ["accel_long", "accel_long_offset",
                 "accel_lat", "accel_lat_offset", "accel_ok"]
        # ekf_fused scalars -- see the row builder. innov/nis are EMPTY (not
        # 0.0) on predict-only rows, matching how d.get() already writes empty
        # cells, so a predict-only row is distinguishable from a zero residual.
        cols += ["ekf_v", "ekf_r", "ekf_b_r", "ekf_theta_g",
                 "ekf_P_x", "ekf_P_y", "ekf_P_psi", "ekf_P_v", "ekf_P_r",
                 "ekf_P_br", "ekf_P_thetag",
                 "ekf_innov_v", "ekf_innov_r", "ekf_nis", "ekf_updated",
                 "ekf_accel_corr", "ekf_ok"]
        # b_a (8th state), appended last. ekf_P_ba is the raw pre-floor
        # variance so a negative value stays visible.
        cols += ["ekf_ba_slow_b_a", "ekf_ba_slow_P_ba",
                 "ekf_ba_tight_b_a", "ekf_ba_tight_P_ba", "ekf_ba_phase"]
        return cols

    def save_record(self, reason="shutdown"):
        """Write the buffered run to <record_dir>/YYYY/MM/DD/HHMM/.

        Safe to call more than once; only the first call for a given run
        writes, so a service request followed by shutdown does not produce
        two half-copies of the same drive.
        """
        if self._rec_saved:
            return None
        self._flush_rows()
        if self._rec_total == 0:
            self.get_logger().warn(
                "save_record: nothing recorded (no samples arrived) — writing nothing.")
            self._rec_saved = True
            return None
        ts = self._rec_start_wall
        try:
            # Rows are already on disk; this finalises the run.
            if self._csv_fh is not None:
                self._csv_fh.flush()
                self._csv_fh.close()
                self._csv_fh = None
            out = self._out_dir
            csv_path = self._csv_path
            dur = (self._rec_t_last - self._rec_t_first) if self._rec_t_first is not None else 0.0
            meta = {
                "started": ts.isoformat(timespec="seconds"),
                "saved": datetime.datetime.now().isoformat(timespec="seconds"),
                "reason": reason,
                "samples": self._rec_total,
                "duration_s": round(dur, 2),
                "wheelbase_m": self.wheelbase,
                "steer_ratio": self.steer_ratio,
                "corrected_steer_ratio": self.corrected_steer_ratio,
                "gear_mode": self.gear_mode,
                "yawavg_window_s": self.yawavg_window_s,
                "yawavg_warmup_s": self.yawavg_warmup_s,
                # ekf_fused: the converged theta_g and b_r are what make a
                # recording self-describing -- you can read the calibration the
                # filter settled on without parsing the CSV. None of it is ever
                # read back automatically; it persists nowhere.
                "ekf_mode": self.ekf_mode,
                "ekf_theta_g_seed": self.ekf_theta_g_seed,
                "theta_g_seed_source": self.ekf_theta_g_seed_source,
                "theta_g_seed_run": self.ekf_theta_g_seed_run,
                "theta_g_seed_value": self.ekf_theta_g_seed,
                "ekf_theta_g_p0_sd": self.ekf_theta_g_p0_sd,
                "ekf_theta_g_sd_final": math.sqrt(max(
                    self.ekf_fused.P[IX_THETA_G][IX_THETA_G], 0.0)),
                "ekf_theta_g_final": self.ekf_fused.X[IX_THETA_G],
                "ekf_b_r_final": self.ekf_fused.X[IX_B_R],
                "ekf_primary_track": "ekf_ba_slow",
                "ekf_b_a_final": self.ekf_ba_slow.X[IX_B_A],
                "ekf_b_a_phase2_exit": self.ekf_ba_slow.ba_phase2_exit,
                "ekf_ba_fast_window": self.ekf_cfg["ba_fast_window"],
                "ekf_sigma_ba_fast": self.ekf_cfg["sigma_ba_fast"],
                "ekf_sigma_ba_slow": self.ekf_cfg["sigma_ba_slow"],
                "ekf_ba_p0_sd": self.ekf_cfg["p0_sd"][IX_B_A],
                "ekf_ba_tight_sigma_ba_slow": self.ekf_cfg_tight["sigma_ba_slow"],
                "ekf_ba_tight_b_a_final": self.ekf_ba_tight.X[IX_B_A],
                "ekf_ba_tight_theta_g_final": self.ekf_ba_tight.X[IX_THETA_G],
                "ekf_ba_tight_updates": self.ekf_ba_tight.updates,
                "ekf_ba_tight_resets": self.ekf_ba_tight.resets,
                "ekf_ba_tight_nis_mean": ((self.ekf_ba_tight.nis_sum / self.ekf_ba_tight.nis_n)
                                          if self.ekf_ba_tight.nis_n else None),
                "ekf_accel_bias": self._ekf_accel_bias,
                "ekf_accel_bias_n": self._ekf_accel_bias_n,
                "ekf_qv_mode": self.ekf_cfg["qv_mode"],
                "ekf_sigma_a": self.ekf_cfg["sigma_a"],
                "ekf_sigma_br": self.ekf_cfg["sigma_br"],
                "ekf_sigma_thetag": self.ekf_cfg["sigma_thetag"],
                "ekf_r_v": self.ekf_cfg["r_v"],
                "ekf_r_r": self.ekf_cfg["r_r"],
                "ekf_thetag_excite_min": self.ekf_thetag_excite_min,
                "ekf_update_mode": self.ekf_update_mode,
                "ekf_zupt": self.ekf_zupt,
                "ekf_joseph": self.ekf_cfg["joseph"],
                "ekf_updates": self.ekf_fused.updates,
                "ekf_skipped": self.ekf_fused.skipped,
                "ekf_theta_g_clamps": self.ekf_fused.theta_g_clamps,
                "ekf_singular": self.ekf_fused.singular,
                "ekf_resets": self.ekf_fused.resets,
                "ekf_steer_clamps": self.ekf_fused.steer_clamps,
                "ekf_steer_model": "tan",
                "ekf_nis_mean": ((self.ekf_fused.nis_sum / self.ekf_fused.nis_n)
                                 if self.ekf_fused.nis_n else None),
                "tracks": [name for _, name in RECORD_TRACKS],
            }
            meta_path = os.path.join(out, "meta.json")
            with open(meta_path, "w") as fh:
                json.dump(meta, fh, indent=2)

            # --- guards against a recording being lost silently --------------
            # 1. An append-only index at the top of the record dir. A run that
            #    vanishes still leaves its line here, so loss is DETECTABLE and
            #    you can see exactly what a missing directory contained. This
            #    file is only ever appended to, never rewritten.
            index = os.path.join(self.record_dir, "INDEX.csv")
            try:
                new_index = not os.path.exists(index)
                with open(index, "a", newline="") as fh:
                    w = csv.writer(fh)
                    if new_index:
                        w.writerow(["saved", "run_dir", "samples", "duration_s", "reason"])
                    w.writerow([meta["saved"], os.path.relpath(out, self.record_dir),
                                meta["samples"], meta["duration_s"], reason])
            except OSError as e:
                self.get_logger().warn(f"could not update INDEX.csv: {e}")

            # 2. A note explaining what this directory is, so nobody (human or
            #    otherwise) mistakes real drives for scratch output.
            readme = os.path.join(self.record_dir, "README.txt")
            if not os.path.exists(readme):
                try:
                    with open(readme, "w") as fh:
                        fh.write(RECORD_README)
                except OSError:
                    pass

            # 3. Make the recording itself read-only. This does not stop a
            #    determined `rm -rf`, but it blocks overwriting and makes a
            #    plain `rm -r` stop and ask instead of deleting silently.
            for f in (csv_path, meta_path):
                try:
                    os.chmod(f, 0o444)
                except OSError:
                    pass

            self._rec_saved = True
            self.get_logger().info(
                f"saved {self._rec_total} samples ({dur:.0f}s) -> {csv_path}")
            return csv_path
        except OSError as e:
            # A failed write must not take the node down mid-drive.
            self.get_logger().error(f"save_record failed: {e}")
            return None

    def _srv_save_record(self, request, response):
        path = self.save_record(reason="service request")
        response.success = path is not None
        response.message = path or "nothing written (already saved, or no samples)"
        return response

    def destroy_node(self):
        self.save_record(reason="shutdown")
        self.client.stop()
        return super().destroy_node()


SINGLETON_LOCK = "/tmp/byd_odom_node.lock"
_lock_fh = None          # module-global: the lock lives as long as the process


def _acquire_singleton(allow_multiple: bool) -> None:
    """Refuse to start if another odom_node already holds the lock.

    Two instances publish to the SAME topics with INDEPENDENT integrator state,
    so RViz interleaves two diverging trajectories and the path appears to jump.
    It is invisible to `ros2 topic hz` and it silently corrupted three separate
    measurements on 2026-09-05 before being spotted.

    The guard lives HERE rather than only in byd_drive.sh because every other
    entry path bypasses that script -- `ros2 launch ...` (which the README even
    documents as the manual fallback), `ros2 run ...`, or an IDE. An flock is
    released automatically when the holder dies, so a crash cannot leave a
    stale lock behind.
    """
    global _lock_fh
    if allow_multiple:
        print("[odom_node] --allow-multiple: skipping the single-instance guard. "
              "Two nodes will publish to the same topics with separate integrator "
              "state; expect the path to jump.")
        return
    _lock_fh = open(SINGLETON_LOCK, "w")
    try:
        fcntl.flock(_lock_fh, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        print(f"[odom_node] REFUSING to start: another odom_node already holds "
              f"{SINGLETON_LOCK}.", file=sys.stderr)
        print("[odom_node] Two instances publish to the same topics with separate "
              "integrator state, which makes the RViz path jump.", file=sys.stderr)
        print("[odom_node] Use  ./byd_drive.sh <ip> --kill-existing  to replace it, "
              "or pass --allow-multiple if you genuinely want two.", file=sys.stderr)
        sys.exit(1)
    _lock_fh.write(str(os.getpid()))
    _lock_fh.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="172.20.10.3", help="device IP (dynamic — check per network)")
    ap.add_argument("--port", type=int, default=5556)
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--path-publish-hz", type=float, default=10.0,
                    help="how often the accumulated Path messages are republished. "
                         "Poses are still APPENDED every tick; only the publish is "
                         "throttled, because republishing the whole Path costs O(n) "
                         "in its length.")
    ap.add_argument("--wheelbase", type=float, default=WHEELBASE_M)
    ap.add_argument("--steer-ratio", type=float, default=STEER_RATIO)
    ap.add_argument("--allow-multiple", action="store_true",
                    help="bypass the single-instance guard and start a second "
                         "node anyway. Expect a jumping path; see _acquire_singleton.")
    ap.add_argument("--corrected-steer-ratio", type=float, default=14.2,
                    help="steer ratio for the CORRECTED track. 13.11 is the "
                         "control-tuned value and over-predicts yaw by ~8%%; prior "
                         "validation put the odometry-fitted value near 14.1-14.3.")
    ap.add_argument("--gear-mode", choices=("signed", "reverse-only", "off"),
                    default="reverse-only",
                    help="how gear maps to the sign of integrated speed. "
                         "'signed': drive +1, reverse -1, park/neutral/"
                         "unknown 0, so an ambiguous gear freezes the path. "
                         "'reverse-only' (DEFAULT since 2026-09-11): reverse -1, "
                         "EVERYTHING else +1 including "
                         "park/neutral/unknown — relies on wheel speed being ~0 when "
                         "stopped, so a gear the server fails to forward never "
                         "freezes the path. 'off': gear ignored entirely, "
                         "pre-GEAR_SIGN behaviour, reverse folds the path back on "
                         "itself. Applies to ALL tracks, not just corrected.")
    ap.add_argument("--stale-timeout", type=float, default=0.5)
    ap.add_argument("--max-path-poses", type=int, default=20000)
    ap.add_argument("--yawavg-window", type=float, default=15.0,
                    help="Track B: trailing window, seconds, over which the "
                         "YAW_OFFSET channel is averaged (default 15).")
    ap.add_argument("--record-dir", default=DEFAULT_RECORD_DIR,
                    help="where run recordings are written, as "
                         "<dir>/YYYY/MM/DD/HHMM/odom.csv (default: %s)" % DEFAULT_RECORD_DIR)
    ap.add_argument("--yawavg-warmup", type=float, default=5.0,
                    help="Track B: seconds of samples to seed the window "
                         "before its Path is published (default 5).")
    # --- ekf_fused ---------------------------------------------------------
    ap.add_argument("--ekf-mode", choices=("fused", "off"), default="fused",
                    help="enable the ekf_fused track (default fused). 'off' "
                         "skips all EKF work and writes empty EKF CSV columns.")
    ap.add_argument("--ekf-theta-g", type=float, default=None,
                    help="curvature-gain seed, == steer_ratio * wheelbase. "
                         "Default None means corrected_steer_ratio * wheelbase "
                         "(14.2 * 2.70 = 38.34), so the EKF and the `corrected` "
                         "track start from IDENTICAL geometry -- a hardcoded "
                         "seed would bias the A/B comparison from tick one.")
    ap.add_argument("--ekf-theta-g-min", type=float, default=EKF_THETA_G_MIN)
    ap.add_argument("--ekf-theta-g-max", type=float, default=EKF_THETA_G_MAX)
    ap.add_argument("--ekf-thetag-excite-min", type=float, default=4.4,
                    help="|v * steering-WHEEL angle| (rad m/s) below which "
                         "theta_g is HELD for that update. Derived: open only "
                         "when a 1.0 theta_g error (2.6%%) would move predicted "
                         "yaw by at least sigma_r = 0.003 rad/s, i.e. "
                         "v*g >= sigma_r*theta_g^2/1.0 = 4.41 (25 deg of wheel "
                         "at 10 m/s). This only decides whether there is enough "
                         "signal to learn from; it cannot judge whether the model "
                         "is trustworthy at that angle. 0 disables the gate.")
    ap.add_argument("--ekf-sigma-a", type=float, default=0.13,
                    help="per-sample accelerometer noise sd, m/s^2, measured "
                         "WHILE DRIVING: 0.117-0.141 across five runs. Standstill "
                         "numbers do not apply (Park ~0.018, just-stopped in "
                         "Drive ~0.12). See --ekf-qv-mode: this number's meaning "
                         "depends on that choice, so do not change one alone.")
    ap.add_argument("--ekf-qv-mode", choices=("quadratic", "linear"),
                    default="quadratic",
                    help="how Q_v scales with dt. 'quadratic' (default) is "
                         "(sigma_a*dt)^2, correct for white per-sample sensor "
                         "noise and what sigma_a=0.13 already means. 'linear' is "
                         "sigma_a^2*dt, the random-walk form used by every OTHER "
                         "state; it changes injected velocity noise by 1/dt (50x "
                         "at 50 Hz), so sigma_a would need re-deriving. Provided "
                         "to A/B on one recording, not as a drop-in.")
    ap.add_argument("--ekf-sigma-br", type=float, default=7.1e-5,
                    help="b_r random-walk rate, rad/s per sqrt(s). 1e-5 * sqrt(50) "
                         "restores the original per-tick intent at the node's "
                         "50 Hz tick after Q was made dt-scaled, and is within "
                         "20%% of one 0.00213 rad/s offset LSB step per 10 min.")
    ap.add_argument("--ekf-sigma-thetag", type=float, default=0.02,
                    help="theta_g random-walk rate per sqrt(s). Kept at 0.02: "
                         "replay showed raising it does not speed recovery once "
                         "R_v is 0.04, and adds late-drive jitter. NOTE the "
                         "variance ratio against --ekf-sigma-br is ~8e4, so the "
                         "filter will "
                         "push essentially the whole yaw residual into theta_g "
                         "and leave b_r near its initial value. That is why the "
                         "excitation gate exists.")
    ap.add_argument("--ekf-r-v", type=float, default=0.04,
                    help="sd on the wheel-speed measurement, m/s. Anchored to the "
                         "0.037-0.048 m/s tick noise measured on DISTINCT samples "
                         "(the earlier 0.016 counted repeated samples). The real "
                         "residual is a systematic accel/wheel-speed scale "
                         "mismatch (slope 1.07-1.15, innovations ~0.99 "
                         "autocorrelated) that this treats as noise -- a known "
                         "v1 limitation, pending a scale-factor state.")
    ap.add_argument("--ekf-r-r", type=float, default=0.003,
                    help="sd on the yaw-rate measurement, rad/s. Measured "
                         "standstill sd is 0.000209, so this is also inflated; "
                         "it sits just above the 1-LSB quantisation step.")
    ap.add_argument("--ekf-update-mode", choices=("fresh", "every-tick"),
                    default="fresh",
                    help="'fresh' (default) runs the measurement update only on "
                         "a NEW sample. The node ticks at 50 Hz on a ~10 Hz "
                         "stream, so 'every-tick' applies one measurement five "
                         "times and shrinks the covariance as if it were five "
                         "independent observations. Prediction runs every tick "
                         "either way.")
    ap.add_argument("--ekf-zupt", choices=("on", "off"), default="on",
                    help="zero-velocity pseudo-measurement at standstill. A "
                         "pseudo-measurement, not a gate, so its effect is "
                         "visible in P. Stops the millimetre-per-minute position "
                         "creep the dead-reckoners avoid by freezing outright.")
    ap.add_argument("--ekf-accel-bias", type=float, default=None,
                    help="force the session accel zero-bias and SKIP calibration "
                         "entirely. Intended for replaying a recording with the "
                         "bias from its meta.json.")
    ap.add_argument("--ekf-accel-bias-samples", type=int, default=100,
                    help="target stationary samples for the SINGLE calibration "
                         "window at session start. This sizes that one window; "
                         "it does NOT re-trigger it. The estimate freezes when "
                         "the window closes and never re-arms, because the rest "
                         "offset is mostly parking slope and a mid-drive re-arm "
                         "would bake a hill into the correction.")
    ap.add_argument("--ekf-accel-bias-min", type=int, default=20,
                    help="below this many stationary samples the bias falls back "
                         "to 0.0 with a warning. Also sizes the one window only.")
    ap.add_argument("--ekf-sigma-ba-fast", type=float, default=0.03,
                    help="b_a random-walk rate, m/s^2 per sqrt(s), in phase 2: "
                         "the first --ekf-ba-fast-window seconds after the car "
                         "first moves, while the parking-slope residual is exposed.")
    ap.add_argument("--ekf-sigma-ba-slow", type=float, default=0.008,
                    help="phase-3 b_a random-walk rate, m/s^2 per sqrt(s), for the "
                         "ekf_ba_slow track. Replay showed this still tracks road "
                         "grade; compare against ekf_ba_tight.")
    ap.add_argument("--ekf-sigma-ba-tight", type=float, default=0.001,
                    help="phase-3 b_a random-walk rate, m/s^2 per sqrt(s), for the "
                         "ekf_ba_tight track. Every other parameter is shared.")
    ap.add_argument("--ekf-ba-fast-window", type=float, default=10.0,
                    help="phase 2 duration, seconds from first motion. Exit is "
                         "time only and one-way: phase 2 is never re-entered.")
    ap.add_argument("--ekf-ba-p0-sd", type=float, default=0.15,
                    help="initial b_a uncertainty sd, m/s^2. Never persisted.")
    ap.add_argument("--ekf-joseph", choices=("on", "off"), default="on",
                    help="Joseph-form covariance update (default on). R_r is "
                         "~9e-6 against order-1 P entries, and b_r/theta_g are "
                         "near-collinear, so the naive (I-KH)P form loses "
                         "precision exactly where it matters. 'off' is for A/B.")
    ap.add_argument("--ekf-diag-s", type=float, default=5.0,
                    help="throttle for the [ekf] diagnostic line, seconds.")
    ap.add_argument("--odom-frame", default="odom")

    argv = [a for a in sys.argv[1:] if not a.startswith("__")]
    args, _ = ap.parse_known_args(argv)

    # Before ANY ROS state exists — a refused start must leave nothing behind.
    _acquire_singleton(args.allow_multiple)

    rclpy.init()
    node = BydOdomNode(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl-C: the finally block below still saves the run.
        pass
    except ExternalShutdownException:
        # SIGTERM (`kill`, or a launch shutting the node down). rclpy raises
        # this out of spin(); without catching it the process exits 1 with a
        # traceback, which looks like a crash. The run is still saved by the
        # finally block either way -- verified 2026-09-09.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
