#!/usr/bin/env python3
"""
Unit tests for the `ekf_fused` 8-state EKF in odom_node.py.

These import the REAL predict_ekf/update_ekf rather than re-implementing them.
The other odometry tests in this directory re-implement their target, which is
right for proving a five-line algebraic identity but wrong for a filter: a
re-implementation only tests itself. odom_node.py imports rclpy and the message
packages at module level, so those are stubbed into sys.modules first -- nothing
in the file CALLS ROS at import time, so bare module objects are enough. Same
trick as test_capture_session.py.

    python3 claude/tests/test_ekf_fused.py

Exits 0 if every case passes, 1 otherwise.
"""
import importlib.util
import math
import os
import random
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
NODE = os.path.expanduser(
    "~/ros2_ws/src/byd_odom_ros/byd_odom_ros/odom_node.py")

# --- stub every ROS import odom_node.py makes at module level ---------------
for name in ("rclpy", "rclpy.executors", "rclpy.node", "rclpy.qos",
             "geometry_msgs", "geometry_msgs.msg", "nav_msgs", "nav_msgs.msg",
             "std_srvs", "std_srvs.srv", "tf2_ros"):
    sys.modules.setdefault(name, types.ModuleType(name))


class _Stub:
    def __init__(self, *a, **k):
        pass


for name, attrs in (
    ("rclpy.executors", ("ExternalShutdownException",)),
    ("rclpy.node", ("Node",)),
    ("rclpy.qos", ("QoSProfile", "ReliabilityPolicy", "HistoryPolicy")),
    ("geometry_msgs.msg", ("Quaternion", "TransformStamped", "PoseStamped")),
    ("nav_msgs.msg", ("Odometry", "Path")),
    ("std_srvs.srv", ("Trigger",)),
    ("tf2_ros", ("TransformBroadcaster",)),
):
    mod = sys.modules[name]
    for a in attrs:
        setattr(mod, a, _Stub)
sys.modules["rclpy"].executors = sys.modules["rclpy.executors"]
sys.modules["rclpy"].node = sys.modules["rclpy.node"]
sys.modules["rclpy"].qos = sys.modules["rclpy.qos"]

if not os.path.exists(NODE):
    print("FATAL: odom_node.py not found at %s" % NODE)
    sys.exit(1)
spec = importlib.util.spec_from_file_location("odom_node", NODE)
M = importlib.util.module_from_spec(spec)
spec.loader.exec_module(M)

PASS = FAIL = 0
N = M.EKF_N


def check(desc, ok, extra=""):
    global PASS, FAIL
    if ok:
        PASS += 1
        print("  PASS  %s" % desc)
    else:
        FAIL += 1
        print("  FAIL  %s   %s" % (desc, extra))


def cfg(**over):
    c = {
        "sigma_xy": 1e-3, "sigma_psi": 0.001, "sigma_r": 0.05,
        "sigma_a": 0.1, "sigma_br": 1e-5, "sigma_thetag": 0.02,
        "qv_mode": "quadratic", "qv_missing_mult": 100.0,
        "r_v": 0.15, "r_r": 0.003, "r_zupt": 1e-2,
        "theta_g_min": M.EKF_THETA_G_MIN, "theta_g_max": M.EKF_THETA_G_MAX,
        "joseph": True, "integrate_psi": True,
        "steer_ratio_nominal": 14.2, "tire_angle_max_rad": math.radians(45.0),
        "ba_active": False, "sigma_ba": 0.0,
        "sigma_ba_fast": 0.03, "sigma_ba_slow": 0.008,
        "ba_fast_window": 10.0,
        "p0_sd": (0.01, 0.01, 0.01, 1.0, 1.0, 0.01, 5.0, 0.15),
    }
    c.update(over)
    return c


def fresh_P(c=None):
    c = c or cfg()
    P = [[0.0] * N for _ in range(N)]
    for i, sd in enumerate(c["p0_sd"]):
        P[i][i] = sd * sd
    return P


def H_both():
    hv = [0.0] * N
    hv[M.IX_V] = 1.0
    hr = [0.0] * N
    hr[M.IX_R] = 1.0
    hr[M.IX_B_R] = 1.0
    return [hv, hr]


R_both = [[0.15 ** 2, 0.0], [0.0, 0.003 ** 2]]


def cholesky(P):
    """PSD oracle. Returns None if P is not positive definite."""
    n = len(P)
    L = [[0.0] * n for _ in range(n)]
    for i in range(n):
        for j in range(i + 1):
            ssum = sum(L[i][k] * L[j][k] for k in range(j))
            if i == j:
                d = P[i][i] - ssum
                if d <= 0:
                    return None
                L[i][j] = math.sqrt(d)
            else:
                L[i][j] = (P[i][j] - ssum) / L[j][j]
    return L


# --- 1. dt = 0 is a bitwise no-op ------------------------------------------
print("\n1. dt=0 is a bitwise no-op (forces dt-scaled Q)")
X0 = [1.0, 2.0, 0.3, 5.0, 0.1, 1e-4, 38.34, 0.0]
P0 = fresh_P()
X1, P1 = M.predict_ekf(list(X0), [r[:] for r in P0], 0.0, 0.05, 0.2, cfg())
# r is the deliberate exception: it is REPLACED (r = v*steer/theta_g), not
# integrated, so it changes even at dt=0 and F's r-row is nonzero regardless of
# dt. Everything else must be untouched, and crucially NO process noise may be
# injected -- that is what the dt-scaled Q buys and what a fixed per-step Q
# would break.
check("X unchanged except r", X1[:4] == X0[:4] and X1[5:] == X0[5:],
      "got %r" % (X1,))
_ch = [(i, j) for i in range(N) for j in range(N) if P1[i][j] != P0[i][j]]
check("P changes only in entries involving r",
      all(M.IX_R in (i, j) for i, j in _ch), "changed: %r" % (_ch,))
check("no process noise injected at dt=0",
      all(P1[i][i] == P0[i][i] for i in range(N) if i != M.IX_R))

# --- 2. standstill: b_r observable, pose frozen ----------------------------
print("\n2. standstill -- b_r learns, theta_g untouched, pose exactly 0")
c = cfg()
X = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 38.34, 0.0]
P = fresh_P(c)
TRUE_BR = 0.0021
tg_before = X[M.IX_THETA_G]
for _ in range(400):
    # v=0 so integrate_psi would be suppressed by the node; mirror that here
    c["integrate_psi"] = False
    X, P = M.predict_ekf(X, P, 0.02, math.radians(30.0), 0.0, c)
    X, P, innov, S, K = M.update_ekf(
        X, P, [0.0, TRUE_BR], H_both(), R_both,
        lock_theta_g=True, joseph=True)
check("b_r converges to the true residual bias",
      abs(X[M.IX_B_R] - TRUE_BR) < 1e-4,
      "b_r=%.8f want %.8f" % (X[M.IX_B_R], TRUE_BR))
check("theta_g untouched while locked",
      abs(X[M.IX_THETA_G] - tg_before) < 1e-9,
      "theta_g=%.12f" % X[M.IX_THETA_G])
# y and psi are EXACTLY zero: y integrates v*sin(psi) with psi==0, and psi is
# not integrated at all below the speed floor. x is NOT exactly zero, and that
# is correct EKF behaviour rather than a defect: predict gives P[x][v] a nonzero
# entry via F[x][v]=cos(psi)*dt, so the z_v=0 correction reaches x through that
# cross-covariance. It settles at a fraction of a millimetre rather than
# drifting, and ZUPT exists precisely to pin it.
check("y exactly 0.0", X[M.IX_Y] == 0.0, "y=%r" % X[M.IX_Y])
check("psi exactly 0.0", X[M.IX_PSI] == 0.0, "psi=%r" % X[M.IX_PSI])
check("x creep under 1 mm over 8 s", abs(X[M.IX_X]) < 1e-3,
      "x=%.3e m" % X[M.IX_X])

# --- 3. the excitation gate leaves P[theta_g][theta_g] BITWISE unchanged ----
print("\n3. excitation gate: P[tg][tg] bitwise unchanged, and ungated shrinks")
random.seed(7)
P = [[0.0] * N for _ in range(N)]
for i in range(N):
    for j in range(i, 7):
        v = random.uniform(-0.3, 0.3)
        P[i][j] = P[j][i] = v
    P[i][i] = abs(P[i][i]) + 1.0
X = [0.0, 0.0, 0.0, 8.0, 0.05, 0.0, 38.34, 0.0]
before = P[M.IX_THETA_G][M.IX_THETA_G]
_, Pg, _, _, _ = M.update_ekf(list(X), [r[:] for r in P], [8.0, 0.05],
                              H_both(), R_both, lock_theta_g=True)
_, Pu, _, _, _ = M.update_ekf(list(X), [r[:] for r in P], [8.0, 0.05],
                              H_both(), R_both, lock_theta_g=False)
check("gated P[tg][tg] bitwise equal",
      Pg[M.IX_THETA_G][M.IX_THETA_G] == before,
      "%r vs %r" % (Pg[M.IX_THETA_G][M.IX_THETA_G], before))
check("ungated P[tg][tg] strictly shrinks",
      Pu[M.IX_THETA_G][M.IX_THETA_G] < before,
      "%r vs %r" % (Pu[M.IX_THETA_G][M.IX_THETA_G], before))

# --- 4. missing / NaN / Inf inputs -----------------------------------------
print("\n4. missing, NaN and Inf inputs are skipped, never fabricated")
for bad in (None, float("nan"), float("inf"), -float("inf")):
    check("_fnum(%r) -> None" % (bad,), M._fnum(bad) is None)
check("_fnum('abc') -> None", M._fnum("abc") is None)
check("_fnum('1.5') -> 1.5", M._fnum("1.5") == 1.5)
X = [0.0, 0.0, 0.0, 5.0, 0.1, 0.0, 38.34, 0.0]
P = fresh_P()
Xn, Pn = M.predict_ekf(X, P, 0.02, 0.01, None, cfg())   # accel absent
check("accel=None still finite", M._all_finite(Xn, Pn))
check("accel=None inflates Q_v",
      Pn[M.IX_V][M.IX_V] > P[M.IX_V][M.IX_V] + (0.1 * 0.02) ** 2)
# single-row update when only yaw is available
hr = [0.0] * N
hr[M.IX_R] = 1.0
hr[M.IX_B_R] = 1.0
Xs, Ps, innov, S, K = M.update_ekf(list(Xn), [r[:] for r in Pn], [0.1],
                                   [hr], [[0.003 ** 2]])
check("1x7 update works and stays finite",
      K is not None and M._all_finite(Xs, Ps))

# --- 5. negative-steering mirror symmetry ----------------------------------
print("\n5. mirror symmetry under steer -> -steer, z_r -> -z_r")
def run(sign):
    c = cfg()
    X = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 38.34, 0.0]
    P = fresh_P(c)
    for k in range(300):
        st = sign * math.radians(12.0)
        X, P = M.predict_ekf(X, P, 0.02, st, 0.3, c)
        X, P, _, _, _ = M.update_ekf(X, P, [6.0, sign * 0.04],
                                     H_both(), R_both)
    return X
Xa, Xb = run(1.0), run(-1.0)
check("psi mirrors", abs(Xb[M.IX_PSI] + Xa[M.IX_PSI]) < 1e-12,
      "%r vs %r" % (Xa[M.IX_PSI], Xb[M.IX_PSI]))
check("y mirrors", abs(Xb[M.IX_Y] + Xa[M.IX_Y]) < 1e-12)
check("x identical", abs(Xb[M.IX_X] - Xa[M.IX_X]) < 1e-12)
check("b_r mirrors", abs(Xb[M.IX_B_R] + Xa[M.IX_B_R]) < 1e-12)
check("theta_g identical", abs(Xb[M.IX_THETA_G] - Xa[M.IX_THETA_G]) < 1e-12)

# --- 6. theta_g near zero / negative: guard the divide ---------------------
print("\n6. theta_g degenerate values are clamped at USE, never divide by zero")
for tg in (1e-12, 0.0, -1e-6, -40.0):
    X = [0.0, 0.0, 0.0, 10.0, 0.0, 0.0, tg, 0.0]
    try:
        Xn, Pn = M.predict_ekf(X, fresh_P(), 0.02, 0.1, 0.0, cfg())
        ok = M._all_finite(Xn, Pn) and abs(Xn[M.IX_R]) < 1e3
    except ZeroDivisionError:
        ok = False
    check("theta_g=%r does not raise and r stays bounded" % tg, ok)

# --- 7. P stays symmetric and PSD over a long random run -------------------
print("\n7. P symmetric + PSD over 10000 randomised steps")
random.seed(11)
c = cfg()
X = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 38.34, 0.0]
P = fresh_P(c)
worst_asym = 0.0
psd_fail = 0
for k in range(10000):
    dt = random.uniform(0.005, 0.05)
    st = math.radians(random.uniform(-40, 40))
    a = random.uniform(-2.0, 2.0)
    X, P = M.predict_ekf(X, P, dt, st, a, c)
    X, P, _, _, _ = M.update_ekf(
        X, P, [random.uniform(0, 15), random.uniform(-0.4, 0.4)],
        H_both(), R_both, lock_theta_g=(random.random() < 0.3))
    worst_asym = max(worst_asym,
                     max(abs(P[i][j] - P[j][i]) for i in range(N) for j in range(N)))
    if k % 100 == 0 and cholesky(P) is None:
        psd_fail += 1
check("P symmetric to 1e-12", worst_asym < 1e-12, "worst=%.3e" % worst_asym)
check("Cholesky never fails", psd_fail == 0, "%d failures" % psd_fail)
check("state finite after 10000 steps", M._all_finite(X, P))

# --- 8. Joseph vs naive ----------------------------------------------------
print("\n8. Joseph agrees with naive short-run, and stays PSD when naive fails")
Xa = [0.0, 0.0, 0.0, 5.0, 0.1, 0.0, 38.34, 0.0]
Pa = fresh_P()
Xb, Pb = list(Xa), [r[:] for r in Pa]
for _ in range(20):
    Xa, Pa, _, _, _ = M.update_ekf(Xa, Pa, [5.0, 0.1], H_both(), R_both,
                                   joseph=True)
    Xb, Pb, _, _, _ = M.update_ekf(Xb, Pb, [5.0, 0.1], H_both(), R_both,
                                   joseph=False)
dmax = max(abs(Pa[i][j] - Pb[i][j]) for i in range(N) for j in range(N))
check("Joseph and naive agree to 1e-9 short-run", dmax < 1e-9,
      "max diff %.3e" % dmax)
# High-gain stress. NOT 1e-10 against 1e6: that is a condition number of 1e16,
# past what float64 can represent at all, so failing it would say nothing about
# the Joseph form. This is the real operating regime -- R_r is genuinely 9e-6
# against order-1 P entries -- run long enough to compound.
Rt = [[0.15 ** 2, 0.0], [0.0, 0.003 ** 2]]
Xj = [0.0, 0.0, 0.0, 5.0, 0.1, 0.0, 38.34, 0.0]
Pj = [[0.0] * N for _ in range(N)]
for i in range(N):
    Pj[i][i] = 100.0
for _ in range(5000):
    Xj, Pj, _, _, _ = M.update_ekf(Xj, Pj, [5.0, 0.1], H_both(), Rt, joseph=True)
check("Joseph stays PSD over 5000 high-gain updates", cholesky(Pj) is not None)
check("Joseph diagonal stays non-negative",
      all(Pj[i][i] >= 0.0 for i in range(N)))

# --- 9. straight-line truth ------------------------------------------------
print("\n9. straight line: 10 m/s for 60 s")
c = cfg()
X = [0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 38.34, 0.0]
P = fresh_P(c)
for _ in range(3000):
    X, P = M.predict_ekf(X, P, 0.02, 0.0, 0.0, c)
    X, P, _, _, _ = M.update_ekf(X, P, [10.0, 0.0], H_both(), R_both)
check("x ~ 600 m", abs(X[M.IX_X] - 600.0) < 0.5, "x=%.3f" % X[M.IX_X])
check("y ~ 0", abs(X[M.IX_Y]) < 1e-6, "y=%.3e" % X[M.IX_Y])
check("psi ~ 0", abs(X[M.IX_PSI]) < 1e-6, "psi=%.3e" % X[M.IX_PSI])

# --- 10. constant-radius convergence of theta_g ----------------------------
print("\n10. theta_g converges from a wrong seed -- under VARYING excitation")
# Deliberately NOT constant-radius: that is precisely the degenerate case test
# 11 below proves unobservable, so demanding convergence there would be
# demanding the impossible. theta_g is identifiable only when v*delta varies.
TG_TRUE = 38.34
c = cfg()
X = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 45.0, 0.0]          # seeded 17% high
P = fresh_P(c)
V = 8.0
for k in range(6000):                              # 120 s at 50 Hz
    st = math.radians(30.0 * math.sin(k * 0.004))  # sweeping steer
    r_true = V * st / TG_TRUE
    X, P = M.predict_ekf(X, P, 0.02, st, 0.0, c)
    X, P, _, _, _ = M.update_ekf(X, P, [V, r_true], H_both(), R_both,
                                 lock_theta_g=abs(V * st) < 0.5)
err = abs(X[M.IX_THETA_G] - TG_TRUE) / TG_TRUE
check("theta_g within 5% of truth", err < 0.05,
      "theta_g=%.3f want %.3f (%.1f%% off)" % (X[M.IX_THETA_G], TG_TRUE, err * 100))

# --- 11. observability negative test ---------------------------------------
print("\n11. constant excitation -> b_r and theta_g become indistinguishable")
c = cfg()
X = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 38.34, 0.0]
P = fresh_P(c)
V_C, ST_C = 8.0, math.radians(25.0)      # CONSTANT v*delta: the degenerate case
r_true_c = V_C * ST_C / TG_TRUE
for _ in range(3000):
    X, P = M.predict_ekf(X, P, 0.02, ST_C, 0.0, c)
    X, P, _, _, _ = M.update_ekf(X, P, [V_C, r_true_c], H_both(), R_both,
                                 lock_theta_g=False)
pbb, ptt = P[M.IX_B_R][M.IX_B_R], P[M.IX_THETA_G][M.IX_THETA_G]
rho = P[M.IX_B_R][M.IX_THETA_G] / math.sqrt(pbb * ptt) if pbb > 0 and ptt > 0 else 0.0
check("|rho| > 0.9 under constant excitation", abs(rho) > 0.9,
      "rho=%.4f" % rho)

# --- 12. psi wrapping ------------------------------------------------------
print("\n12. psi stays wrapped to [-pi, pi] over 400 s of constant rotation")
c = cfg()
V, ST = 5.0, math.radians(20.0)
r_true = V * ST / 38.34          # consistent with the model, as a real car is
X = [0.0, 0.0, 0.0, V, r_true, 0.0, 38.34, 0.0]
P = fresh_P(c)
worst = 0.0
for _ in range(20000):           # 400 s, many full revolutions
    X, P = M.predict_ekf(X, P, 0.02, ST, 0.0, c)
    X, P, _, _, _ = M.update_ekf(X, P, [V, r_true], H_both(), R_both)
    worst = max(worst, abs(X[M.IX_PSI]))
check("|psi| <= pi throughout", worst <= math.pi + 1e-12, "worst=%.6f" % worst)
check("still finite", M._all_finite(X, P))

# --- 13. ZUPT pins v at zero ----------------------------------------------
print("\n13. ZUPT pseudo-measurement pins v hard at 0")
c = cfg()
X = [0.0, 0.0, 0.0, 0.4, 0.0, 0.0, 38.34, 0.0]          # spurious 0.4 m/s
P = fresh_P(c)
hz = [0.0] * N
hz[M.IX_V] = 1.0
for _ in range(200):
    c["integrate_psi"] = False
    X, P = M.predict_ekf(X, P, 0.02, 0.0, 0.0, c)
    X, P, _, _, _ = M.update_ekf(X, P, [0.0], [hz], [[1e-2 ** 2]])
check("v driven to ~0", abs(X[M.IX_V]) < 1e-3, "v=%.6f" % X[M.IX_V])
check("position creep bounded", abs(X[M.IX_X]) < 0.05, "x=%.6f" % X[M.IX_X])

# --- 14. full-lock parking: exact model ------------------------------------
print("\n14. full lock: exact tan model removes the ~11% yaw under-prediction")
SR, WB = 14.2, 2.70
TG = SR * WB
WHEEL = math.radians(460.0)            # steering-WHEEL angle, as the node feeds
Vp = 0.3                               # ~1 km/h creep
r_exact = Vp * math.tan(WHEEL / SR) / WB
Xn, _ = M.predict_ekf([0.0, 0.0, 0.0, Vp, 0.0, 0.0, TG, 0.0], fresh_P(), 0.0,
                      WHEEL, 0.0, cfg())
err_new = (Xn[M.IX_R] - r_exact) / r_exact
err_lin = (Vp * WHEEL / TG - r_exact) / r_exact
check("old linear form under-predicted by ~11% at 460 deg",
      -0.12 < err_lin < -0.10, "err_lin=%.4f" % err_lin)
check("prediction now equals the exact bicycle model",
      abs(err_new) < 1e-12, "err_new=%.3e" % err_new)
SMALL = math.radians(5.0)
Xs5, _ = M.predict_ekf([0.0, 0.0, 0.0, 10.0, 0.0, 0.0, TG, 0.0], fresh_P(), 0.0,
                       SMALL, 0.0, cfg())
# At 5 deg of WHEEL the tire angle is 0.35 deg, so tan(d)/d - 1 ~= d^2/3 ~= 1.3e-5.
# Assert the prediction differs from the linear form by exactly that analytic
# amount -- not by zero, which the exact model correctly does not give.
_d5 = SMALL / SR
_want = math.tan(_d5) / _d5 - 1.0
_got = Xs5[M.IX_R] / (10.0 * SMALL / TG) - 1.0
check("small angle: departs from linear by exactly tan(d)/d - 1 (~1.3e-5)",
      abs(_got - _want) < 1e-9, "got %.3e want %.3e" % (_got, _want))
check("small angle: still within 1e-4 of the linear form", abs(_got) < 1e-4)

# --- 15. Jacobian r-row agrees with the prediction it linearises -----------
print("\n15. Jacobian r-row matches finite differences at full lock")
c15 = cfg()
Vb = 1.5
Xb = [0.0, 0.0, 0.0, Vb, 0.0, 0.0, TG, 0.0]
def r_of(v, tg):
    X = list(Xb)
    X[M.IX_V] = v
    X[M.IX_THETA_G] = tg
    return M.predict_ekf(X, fresh_P(), 0.0, WHEEL, 0.0, c15)[0][M.IX_R]
def F_entry(col):
    # dt=0 gives Q=0 and F[col][col]=1, so with P = e_col e_col^T the
    # propagated P[r][col] is exactly F[r][col]: this reads F out of the real
    # function instead of re-deriving it.
    P = [[0.0] * N for _ in range(N)]
    P[col][col] = 1.0
    return M.predict_ekf(list(Xb), P, 0.0, WHEEL, 0.0, c15)[1][M.IX_R][col]
h = 1e-6
fd_v = (r_of(Vb + h, TG) - r_of(Vb - h, TG)) / (2 * h)
fd_tg = (r_of(Vb, TG + h) - r_of(Vb, TG - h)) / (2 * h)
check("dr/dv matches finite difference",
      abs(F_entry(M.IX_V) - fd_v) / abs(fd_v) < 1e-6,
      "F=%.9f fd=%.9f" % (F_entry(M.IX_V), fd_v))
check("dr/dtheta_g matches finite difference",
      abs(F_entry(M.IX_THETA_G) - fd_tg) / abs(fd_tg) < 1e-6,
      "F=%.9e fd=%.9e" % (F_entry(M.IX_THETA_G), fd_tg))

# --- 16. tan() domain guard, and odd symmetry at full lock -----------------
print("\n16. front-wheel clamp guards tan(); odd symmetry holds at full lock")
g89, cl89 = M._effective_steer(math.radians(89.0 * SR), cfg())
check("89 deg at the tire is clamped", cl89)
check("clamped term finite and bounded",
      math.isfinite(g89) and abs(g89) <= SR * math.tan(math.radians(45.0)) + 1e-9)
_, clmax = M._effective_steer(math.radians(491.3), cfg())
check("largest recorded lock (491.3 deg) is NOT clamped", not clmax)
Xp, Pp = M.predict_ekf([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, TG, 0.0], fresh_P(), 0.02,
                       WHEEL, 0.0, cfg())
Xm, Pm = M.predict_ekf([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, TG, 0.0], fresh_P(), 0.02,
                       -WHEEL, 0.0, cfg())
check("r odd-symmetric at full lock", abs(Xp[M.IX_R] + Xm[M.IX_R]) < 1e-15,
      "%r vs %r" % (Xp[M.IX_R], Xm[M.IX_R]))
check("P identical under steer -> -steer",
      all(abs(Pp[i][j] - Pm[i][j]) < 1e-15
          for i in range(N) for j in range(N)
          if not ((i == M.IX_R) ^ (j == M.IX_R))))

# --- 17. theta_g seed selection: persisted history, checks, fallback ------
print("\n17. theta_g seed: persisted history, qualification checks, fallback")
import csv as _csv, json as _json, shutil as _shutil, tempfile as _tempfile
WB17, SR17 = 2.70, 14.2
DEF17 = WB17 * SR17

def good_meta(**over):
    m = {"ekf_theta_g_final": 38.2, "ekf_theta_g_sd_final": 0.3,
         "ekf_resets": 0, "ekf_theta_g_clamps": 0, "ekf_singular": 0,
         "ekf_steer_clamps": 0, "duration_s": 500.0, "ekf_updates": 4000,
         "wheelbase_m": WB17, "corrected_steer_ratio": SR17}
    m.update(over)
    return m

def make_record_dir(runs):
    """runs: list of (run_dir, meta_or_None_or_'CORRUPT'), oldest first."""
    d = _tempfile.mkdtemp(prefix="ekfseed_")
    with open(os.path.join(d, "INDEX.csv"), "w", newline="") as fh:
        w = _csv.writer(fh)
        w.writerow(["saved", "run_dir", "samples", "duration_s", "reason"])
        for run_dir, meta in runs:
            w.writerow(["t", run_dir, 1, 1, "shutdown"])
            if meta is None:
                continue
            os.makedirs(os.path.join(d, run_dir), exist_ok=True)
            with open(os.path.join(d, run_dir, "meta.json"), "w") as mf:
                mf.write("{not json" if meta == "CORRUPT" else _json.dumps(meta))
    return d

def sel(d, override=None):
    return M._select_theta_g_seed(override, d, WB17, SR17)

missing_dir = os.path.join(_tempfile.gettempdir(), "definitely_not_a_record_dir_xyz")
seed, src, run, sd, _ = sel(missing_dir)
check("no INDEX.csv -> default seed, sd 5",
      abs(seed - DEF17) < 1e-12 and src == "default" and run is None and sd == 5.0,
      "%r %r %r %r" % (seed, src, run, sd))

seed, src, run, sd, _ = sel(missing_dir, override=39.1)
check("--ekf-theta-g override wins, keeps sd 5",
      seed == 39.1 and src == "override" and sd == 5.0)

d = make_record_dir([("2026/01/01/0001", good_meta(ekf_theta_g_final=38.05))])
seed, src, run, sd, _ = sel(d)
check("qualifying run -> persisted seed, sd 2",
      seed == 38.05 and src == "persisted" and run == "2026/01/01/0001" and sd == 2.0,
      "%r %r %r %r" % (seed, src, run, sd))
_shutil.rmtree(d)

d = make_record_dir([("old", good_meta(ekf_theta_g_final=37.9)),
                     ("new", good_meta(ekf_theta_g_final=36.25))])
seed, src, run, sd, note = sel(d)
check("newest rejected -> falls back to the older qualifying run",
      seed == 37.9 and run == "old" and sd == 2.0, "%r %r %s" % (seed, run, note))
_shutil.rmtree(d)

rejections = {
    "final below 37.0": good_meta(ekf_theta_g_final=36.99),
    "final above 40.0": good_meta(ekf_theta_g_final=40.01),
    "sd at bound 1.0": good_meta(ekf_theta_g_sd_final=1.0),
    "sd key missing (pre-round-2 run)": {k: v for k, v in good_meta().items()
                                         if k != "ekf_theta_g_sd_final"},
    "steer_clamps key missing": {k: v for k, v in good_meta().items()
                                 if k != "ekf_steer_clamps"},
    "a reset occurred": good_meta(ekf_resets=1),
    "a theta_g clamp occurred": good_meta(ekf_theta_g_clamps=1),
    "a singular update occurred": good_meta(ekf_singular=1),
    "a steer clamp occurred": good_meta(ekf_steer_clamps=2),
    "duration under 120 s": good_meta(duration_s=119.9),
    "under 1000 updates": good_meta(ekf_updates=999),
    "different wheelbase": good_meta(wheelbase_m=2.92),
    "different steer ratio": good_meta(corrected_steer_ratio=16.0),
    "non-finite final": good_meta(ekf_theta_g_final=float("nan")),
}
for label, meta in rejections.items():
    d = make_record_dir([("only", meta)])
    seed, src, run, sd, note = sel(d)
    check("reject: %s -> default, sd 5" % label,
          src == "default" and abs(seed - DEF17) < 1e-12 and sd == 5.0,
          "%r %r %s" % (src, seed, note))
    _shutil.rmtree(d)

d = make_record_dir([("good", good_meta(ekf_theta_g_final=38.4)),
                     ("corrupt", "CORRUPT"), ("gone", None)])
try:
    seed, src, run, sd, note = sel(d)
    ok = seed == 38.4 and run == "good" and sd == 2.0
except Exception as e:
    ok, note = False, repr(e)
check("corrupt and missing meta.json are skipped without raising", ok, note)
_shutil.rmtree(d)

check("qualifier returns a reason string, never raises, on garbage",
      isinstance(M._theta_g_run_qualifies(["not", "a", "dict"], WB17, SR17), str))

# --- 18. b_a phase 1: out of the dynamics until the car first moves --------
print("\n18. b_a phase 1: frozen at standstill even with ZUPT and a slope in accel")
t18 = M.EkfTrack(cfg(r_v=0.04), 38.34)
p0_ba = t18.P[M.IX_B_A][M.IX_B_A]
for k in range(400):                       # 8 s parked on a slope, unsubtracted
    t18.step(0.02, 0.0, 0.23, 0.0, 0.0, k % 5 == 0, True, False, True, False)
check("b_a exactly 0.0 while never moved", t18.X[M.IX_B_A] == 0.0,
      "b_a=%r" % t18.X[M.IX_B_A])
check("P[b_a][b_a] bitwise unchanged in phase 1",
      t18.P[M.IX_B_A][M.IX_B_A] == p0_ba,
      "%r vs %r" % (t18.P[M.IX_B_A][M.IX_B_A], p0_ba))
check("phase stays 1 and exit is 'never'",
      t18.ba_phase == 1 and t18.ba_phase2_exit == "never")
Xi = [0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 38.34, 0.2]
Xn_inact, _ = M.predict_ekf(list(Xi), fresh_P(), 0.02, 0.0, 0.0, cfg(ba_active=False))
Xn_act, _ = M.predict_ekf(list(Xi), fresh_P(), 0.02, 0.0, 0.0, cfg(ba_active=True))
check("inactive: prediction ignores b_a", Xn_inact[M.IX_V] == 5.0)
check("active: v += (a - b_a)*dt", abs(Xn_act[M.IX_V] - (5.0 - 0.2 * 0.02)) < 1e-15,
      "v=%r" % Xn_act[M.IX_V])

# --- 19. Jacobian b_a column matches finite differences --------------------
print("\n19. Jacobian: dv/db_a = -dt and dr/db_a = -dt*g/theta_g, both vs finite differences")
c19 = cfg(ba_active=True, sigma_xy=0.0, sigma_psi=0.0, sigma_r=0.0, sigma_a=0.0,
          sigma_br=0.0, sigma_thetag=0.0, sigma_ba=0.0)
DT19, ST19 = 0.02, math.radians(200.0)
Xb19 = [0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 38.34, 0.1]
def pred_of(ba):
    X = list(Xb19); X[M.IX_B_A] = ba
    return M.predict_ekf(X, fresh_P(), DT19, ST19, 0.4, c19)[0]
def F_col(row, col, cc):
    P = [[0.0] * N for _ in range(N)]
    P[col][col] = 1.0
    return M.predict_ekf(list(Xb19), P, DT19, ST19, 0.4, cc)[1][row][col]
h = 1e-6
fd_v = (pred_of(0.1 + h)[M.IX_V] - pred_of(0.1 - h)[M.IX_V]) / (2 * h)
fd_r = (pred_of(0.1 + h)[M.IX_R] - pred_of(0.1 - h)[M.IX_R]) / (2 * h)
Fv, Fr = F_col(M.IX_V, M.IX_B_A, c19), F_col(M.IX_R, M.IX_B_A, c19)
check("F[v][b_a] == -dt", abs(Fv + DT19) < 1e-15, "F=%r" % Fv)
check("F[v][b_a] matches finite difference", abs(Fv - fd_v) < 1e-9, "F=%r fd=%r" % (Fv, fd_v))
check("F[r][b_a] matches finite difference (chain rule through v_n)",
      abs(Fr - fd_r) / abs(fd_r) < 1e-6, "F=%.9e fd=%.9e" % (Fr, fd_r))
c19i = dict(c19); c19i["ba_active"] = False
check("inactive: F[v][b_a] and F[r][b_a] are 0",
      F_col(M.IX_V, M.IX_B_A, c19i) == 0.0 and F_col(M.IX_R, M.IX_B_A, c19i) == 0.0)

# --- 20. b_a converges on an oversubtracted accel ---------------------------
print("\n20. b_a converges to -0.23 when session-zero oversubtracted a +0.23 slope")
t20 = M.EkfTrack(cfg(r_v=0.04), 38.34)
V20 = 10.0
t20.X[M.IX_V] = V20
reached = None
for k in range(3000):                      # 60 s at 50 Hz, update at 10 Hz
    # true accel 0; accel fed = true - 0.23 (slope baked in at the parking spot)
    t20.step(0.02, 0.0, -0.23, V20, 0.0, k % 5 == 0, True, True, False, True)
    if reached is None and abs(t20.X[M.IX_B_A] + 0.23) < 0.03:
        reached = (k + 1) * 0.02
check("b_a within 0.03 of -0.23 inside 30 s", reached is not None and reached <= 30.0,
      "reached=%r, b_a=%.4f" % (reached, t20.X[M.IX_B_A]))
check("b_a settles near -0.23 by 60 s", abs(t20.X[M.IX_B_A] + 0.23) < 0.02,
      "b_a=%.4f" % t20.X[M.IX_B_A])
check("phase 2 exited on time", t20.ba_phase == 3 and t20.ba_phase2_exit == "time",
      "phase=%d exit=%s" % (t20.ba_phase, t20.ba_phase2_exit))

# --- 21. phase exits and one-way transition --------------------------------
print("\n21. phase 2 exits on TIME only (10 s default), never on P, and never re-enters")
tt = M.EkfTrack(cfg(r_v=0.04), 38.34)
exited_early = False
for k in range(700):                       # 14 s moving
    tt.step(0.02, 0.0, 0.0, 5.0, 0.0, k % 5 == 0, True, True, False, True)
    if tt.ba_phase == 3 and tt.ba_phase2_t < 10.0 - 1e-9:
        exited_early = True
check("no early exit even after sd(b_a) collapses", not exited_early)
check("exits on time at the 10 s default", tt.ba_phase2_exit == "time"
      and abs(tt.ba_phase2_exit_t - 10.0) < 0.021, "exit=%s t=%r" % (tt.ba_phase2_exit, tt.ba_phase2_exit_t))
check("converge criterion is gone from the code",
      "ba_converge_sd" not in open(NODE).read())
tc = tt
for k in range(200):                       # stop, then move again
    tc.step(0.02, 0.0, 0.0, 0.0, 0.0, k % 5 == 0, True, False, True, False)
for k in range(200):
    tc.step(0.02, 0.0, 0.0, 5.0, 0.0, k % 5 == 0, True, True, False, True)
check("phase stays 3 after stopping and moving again", tc.ba_phase == 3)
tn = M.EkfTrack(cfg(), 38.34)
tn._reset_numeric()
check("numeric reset keeps an 8-element state with b_a = 0",
      len(tn.X) == N and tn.X[M.IX_B_A] == 0.0)

# --- 22. two tracks differ ONLY in phase-3 sigma_ba ------------------------
print("\n22. ekf_ba_slow vs ekf_ba_tight: identical until phase 3, tight b_a moves less")
cs = cfg(r_v=0.04)
ct = dict(cs); ct["sigma_ba_slow"] = 0.001
ts_, tt_ = M.EkfTrack(cs, 38.34), M.EkfTrack(ct, 38.34)
diff_before = False
k = 0
# 1 s parked, then drive until the time-only exit. Loop on the phase rather than
# a fixed tick count: 0.02 s summed 500 times is just under 10.0 in floating
# point, so the exit lands one tick after the nominal 10 s -- correct behaviour.
while ts_.ba_phase < 3 and k < 700:
    mv = k >= 50
    for trk in (ts_, tt_):
        trk.step(0.02, 0.0, -0.2 if mv else 0.0, 8.0 if mv else 0.0, 0.0,
                 k % 5 == 0, True, mv, not mv, mv)
    if ts_.X != tt_.X or ts_.P != tt_.P:
        diff_before = True
    k += 1
check("bitwise identical through phases 1 and 2", not diff_before)
check("both in phase 3 now", ts_.ba_phase == 3 and tt_.ba_phase == 3)
b0s, b0t = ts_.X[M.IX_B_A], tt_.X[M.IX_B_A]
for k in range(3000):                      # 60 s phase 3 with a +0.3 m/s^2 grade step
    for trk in (ts_, tt_):
        trk.step(0.02, 0.0, 0.1, 8.0, 0.0, k % 5 == 0, True, True, False, True)
ds, dt_ = abs(ts_.X[M.IX_B_A] - b0s), abs(tt_.X[M.IX_B_A] - b0t)
check("tight track's b_a moves less under the same disturbance", dt_ < ds,
      "slow moved %.4f, tight moved %.4f" % (ds, dt_))

print("\n%d passed, %d failed" % (PASS, FAIL))
sys.exit(0 if FAIL == 0 else 1)
