#!/usr/bin/env python3
"""
analyze_yaw_lag.py — MEASUREMENT ONLY. Quantify the timing (and gain) offset
between the car's measured yaw rate and the kinematic-model prediction.

No ROS, no device, no rclpy. Reads a JSONL produced by
`byd_yaw_sensor_probe.py --log`, which already carries yaw_rate,
steer_deg_raw, v_ego and t on ONE shared CAN timebase.

WHY BOTH LAG AND GAIN
  A pure timing offset and a pure amplitude error look similar in a plot but
  need different fixes. Reporting them separately, per steering event, is what
  tells you which you have -- and whether it is constant or speed-dependent.

  lag_ms > 0  => the measured yaw happens AFTER the prediction (physical:
                 the car takes time to respond to steering input).
  gain   < 1  => measured yaw is SMALLER than predicted (the model
                 over-predicts; e.g. the steer ratio is too small).

DELIBERATELY NOT DONE HERE
  No fix, no ratio conclusion. ratio is pinned at 14.2 purely so the ratio
  question cannot confound the lag measurement -- a wrong ratio shows up as a
  gain error, not a timing one.

USAGE
  python3 claude/analyze_yaw_lag.py <log.jsonl>
  python3 claude/analyze_yaw_lag.py --selftest      # validates the estimator
"""
import argparse
import json
import math
import os
import re
import sys

import numpy as np

# --- reuse the node's own functions, verbatim, without importing ROS ---------
# odom_node.py imports rclpy at module level, so it cannot simply be imported
# here. Instead the two pure functions are exec'd straight from its source, so
# this analysis provably uses the SAME math the node uses rather than a
# reimplementation that could silently drift from it.
_NODE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "byd_odom_ros", "byd_odom_ros", "odom_node.py")
_ns = {"math": math}
if os.path.isfile(_NODE):
    _src = open(_NODE).read()
    for _fn in ("tire_angle_rad", "heading_rate_rad_s"):
        _m = re.search(rf"^def {_fn}\(.*?\n(?:(?:    .*)?\n)*", _src, re.M)
        if _m is None:
            sys.exit(f"could not extract {_fn}() from {_NODE}")
        exec(_m.group(0), _ns)
else:
    sys.exit(f"odom_node.py not found at {_NODE} — cannot reuse its math")
tire_angle_rad = _ns["tire_angle_rad"]
heading_rate_rad_s = _ns["heading_rate_rad_s"]

RATIO = 14.2          # pinned: see "DELIBERATELY NOT DONE HERE"
WHEELBASE = 2.70
GRID_HZ = 100.0       # uniform resample rate for correlation
MAX_LAG_S = 0.5       # +/- search window
STEER_ON = 15.0       # deg, event starts above this
STEER_OFF = 5.0       # deg, event ends below this
MIN_EVENT_S = 0.6
MIN_SPEED = 1.0       # m/s; below this the model predicts ~0 and lag is meaningless


def load(path):
    t, yaw, steer, v = [], [], [], []
    for line in open(path):
        line = line.strip()
        if not line:
            continue
        try:
            d = json.loads(line)
        except json.JSONDecodeError:
            continue
        if d.get("yaw_rate") is None or d.get("steer_deg_raw") is None:
            continue
        if d.get("v_ego") is None:
            continue
        t.append(d["t"]); yaw.append(d["yaw_rate"])
        steer.append(d["steer_deg_raw"]); v.append(d["v_ego"])
    return (np.asarray(t), np.asarray(yaw), np.asarray(steer), np.asarray(v))


def resample(t, *series):
    """Uniform grid — cross-correlation assumes even spacing, and the CAN log
    has jitter."""
    if len(t) < 2:
        return t, series
    grid = np.arange(t[0], t[-1], 1.0 / GRID_HZ)
    return grid, tuple(np.interp(grid, t, s) for s in series)


def predict(steer_deg, v_ms):
    return np.asarray([heading_rate_rad_s(vv, tire_angle_rad(ss, RATIO), WHEELBASE)
                       for ss, vv in zip(steer_deg, v_ms)])


def best_lag(meas, pred, dt):
    """Cross-correlate, return (lag_seconds, normalised peak correlation).

    Positive result = `meas` occurs LATER than `pred`. Both series are mean-
    removed so a constant offset in either cannot dominate the correlation.
    """
    m = meas - meas.mean()
    p = pred - pred.mean()
    if np.allclose(m, 0) or np.allclose(p, 0):
        return float("nan"), 0.0
    c = np.correlate(m, p, mode="full")
    lags = np.arange(-len(p) + 1, len(m)) * dt
    win = np.abs(lags) <= MAX_LAG_S
    if not win.any():
        return float("nan"), 0.0
    c_w, lags_w = c[win], lags[win]
    k = int(np.argmax(c_w))
    denom = math.sqrt(float((m * m).sum()) * float((p * p).sum()))
    return float(lags_w[k]), float(c_w[k] / denom) if denom else 0.0


def find_events(steer, v, dt):
    """Hysteresis: event starts above STEER_ON, ends when back under STEER_OFF."""
    a = np.abs(steer)
    events, i, n = [], 0, len(a)
    while i < n:
        if a[i] > STEER_ON:
            j = i
            while j < n and a[j] > STEER_OFF:
                j += 1
            if (j - i) * dt >= MIN_EVENT_S and v[i:j].mean() >= MIN_SPEED:
                events.append((max(0, i - int(0.3 / dt)), min(n, j + int(0.3 / dt))))
            i = j
        else:
            i += 1
    return events


def analyse(path, quiet=False):
    t, yaw, steer, v = load(path)
    if len(t) < 10:
        print(f"  too few usable samples in {path} ({len(t)})"); return None
    grid, (yaw_r, steer_r, v_r) = resample(t, yaw, steer, v)
    dt = 1.0 / GRID_HZ
    pred = predict(steer_r, v_r)

    if not quiet:
        print(f"  file        : {path}")
        print(f"  samples     : {len(t)} raw -> {len(grid)} on a {GRID_HZ:.0f} Hz grid")
        print(f"  duration    : {grid[-1] - grid[0]:.1f} s")
        print(f"  speed       : {v_r.min():.1f} .. {v_r.max():.1f} m/s")
        print(f"  |steer| max : {np.abs(steer_r).max():.1f} deg")
        print(f"  |yaw| max   : {np.abs(yaw_r).max():.4f} rad/s\n")

    g_lag, g_corr = best_lag(yaw_r, pred, dt)
    events = find_events(steer_r, v_r, dt)

    print(f"  GLOBAL   lag = {g_lag*1000:+7.1f} ms   peak corr = {g_corr:.3f}")
    if not events:
        print("\n  no distinct steering events found "
              f"(need |steer| > {STEER_ON:.0f} deg for >= {MIN_EVENT_S}s at >= {MIN_SPEED} m/s)")
        print("  -> a global number from continuous loop data mixes many manoeuvres;")
        print("     use a pulse-test drive for a trustworthy per-event answer.")
        return {"global_lag": g_lag, "events": []}

    print(f"\n  {len(events)} steering event(s):\n")
    print(f"  {'ev':>3} {'t_start':>8} {'dur':>6} {'speed':>7} {'lag_ms':>8} "
          f"{'corr':>6} {'peak_meas':>10} {'peak_pred':>10} {'gain':>6}")
    rows = []
    for k, (a, b) in enumerate(events):
        seg_m, seg_p, seg_v = yaw_r[a:b], pred[a:b], v_r[a:b]
        lag, corr = best_lag(seg_m, seg_p, dt)
        pm, pp = np.abs(seg_m).max(), np.abs(seg_p).max()
        gain = pm / pp if pp > 1e-9 else float("nan")
        sp = seg_v.mean()
        rows.append((k, grid[a] - grid[0], (b - a) * dt, sp, lag * 1000, corr, pm, pp, gain))
        print(f"  {k:>3} {grid[a]-grid[0]:8.1f} {(b-a)*dt:6.2f} {sp:7.2f} "
              f"{lag*1000:8.1f} {corr:6.3f} {pm:10.4f} {pp:10.4f} {gain:6.3f}")

    lags = np.array([r[4] for r in rows])
    gains = np.array([r[8] for r in rows])
    speeds = np.array([r[3] for r in rows])
    ok = np.isfinite(lags)
    print(f"\n  LAG   mean {np.mean(lags[ok]):+.1f} ms   median {np.median(lags[ok]):+.1f} ms   "
          f"sd {np.std(lags[ok]):.1f} ms   range {lags[ok].min():+.1f} .. {lags[ok].max():+.1f}")
    print(f"  GAIN  mean {np.nanmean(gains):.3f}   median {np.nanmedian(gains):.3f}   "
          f"sd {np.nanstd(gains):.3f}")
    if ok.sum() >= 3 and np.std(speeds[ok]) > 1e-6:
        r = float(np.corrcoef(speeds[ok], lags[ok])[0, 1])
        print(f"  lag vs speed correlation r = {r:+.3f}  "
              f"({'suggests speed-dependent' if abs(r) > 0.6 else 'no strong speed dependence'})")
    else:
        print("  lag vs speed: too few events / too little speed spread to say")
    return {"global_lag": g_lag, "events": rows}


def selftest():
    """Synthesise data with a KNOWN lag and gain, confirm both are recovered.
    This is what makes the sign convention trustworthy rather than assumed."""
    print("SELF-TEST — inject known lag/gain, check recovery\n")
    dt = 1.0 / GRID_HZ
    n = int(60 / dt)
    tt = np.arange(n) * dt
    steer = np.zeros(n)
    for c in (5, 15, 25, 35, 45):          # five pulses
        i0, i1 = int(c / dt), int((c + 2.5) / dt)
        steer[i0:i1] = 60.0 * np.sin(np.linspace(0, np.pi, i1 - i0))
    v = np.full(n, 8.0)
    pred = predict(steer, v)

    ok = True
    for true_lag_ms, true_gain in ((0, 1.0), (80, 1.0), (200, 1.0), (120, 0.85), (-60, 1.0)):
        shift = int(round(true_lag_ms / 1000.0 / dt))
        meas = np.zeros_like(pred)
        if shift >= 0:
            meas[shift:] = pred[:n - shift] * true_gain
        else:
            meas[:n + shift] = pred[-shift:] * true_gain
        meas = meas + np.random.default_rng(0).normal(0, 0.002, n)
        lag, corr = best_lag(meas, pred, dt)
        pm, pp = np.abs(meas).max(), np.abs(pred).max()
        err = lag * 1000 - true_lag_ms
        good = abs(err) <= 15.0 and abs(pm / pp - true_gain) < 0.08
        ok &= good
        print(f"  {'PASS' if good else 'FAIL'}  true lag {true_lag_ms:+5.0f} ms gain {true_gain:.2f}"
              f"  ->  measured {lag*1000:+7.1f} ms (err {err:+5.1f})  gain {pm/pp:.3f}  corr {corr:.3f}")
    print(f"\n  {'self-test PASSED — positive lag = measured occurs AFTER predicted' if ok else 'self-test FAILED'}")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log", nargs="?", help="JSONL from byd_yaw_sensor_probe.py --log")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        return selftest()
    if not args.log:
        ap.error("give a log file, or --selftest")
    analyse(args.log)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
