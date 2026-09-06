#!/usr/bin/env python3
"""
analyze_yaw_offset_drift.py — MEASUREMENT ONLY.

Is YAW_OFFSET (CAN 546) a stable calibration constant, or a genuinely drifting
bias? That decides whether the `yawsensor` odometry track needs a live bias
correction the way `corrected` uses liveParameters.angleOffsetAverageDeg.

Reads a JSONL from `byd_yaw_sensor_probe.py --log`. No ROS, no device.

METHOD
  1. Summary stats of yaw_offset over the session.
  2. Linear regression of yaw_offset vs time -> slope in rad/s per MINUTE,
     with R². Slope alone is not enough: a near-zero slope with a terrible R²
     means "noise, no trend", whereas a real slope with decent R² means drift.
  3. Validity cross-check on yaw_rate itself. If the car actually rotated, the
     window was not stationary and the drift numbers are suspect.

  Also reports the quantisation step, because a signal that only ever takes a
  couple of discrete values cannot show meaningful drift regardless of fit.

USAGE
  python3 claude/analyze_yaw_offset_drift.py <log.jsonl>
"""
import argparse
import json
import sys

import numpy as np

LSB = 0.002132603          # DBC factor for YAW_RATE / YAW_OFFSET
ROT_THRESH = 0.02          # rad/s; |yaw_rate| above this = real rotation
R2_REAL = 0.30             # below this, a slope is not a credible trend


def load(path):
    t, off, yaw, v = [], [], [], []
    for line in open(path):
        line = line.strip()
        if not line:
            continue
        try:
            d = json.loads(line)
        except json.JSONDecodeError:
            continue
        if d.get("yaw_offset") is None or d.get("yaw_rate") is None:
            continue
        t.append(d["t"]); off.append(d["yaw_offset"]); yaw.append(d["yaw_rate"])
        v.append(d.get("v_ego") if d.get("v_ego") is not None else 0.0)
    return map(np.asarray, (t, off, yaw, v))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    args = ap.parse_args()
    t, off, yaw, v = load(args.log)
    if len(t) < 100:
        sys.exit(f"too few samples ({len(t)})")

    dur_min = (t[-1] - t[0]) / 60.0
    print(f"  file     : {args.log}")
    print(f"  samples  : {len(t)}   duration: {t[-1]-t[0]:.1f} s ({dur_min:.2f} min)")
    print(f"  rate     : {len(t)/(t[-1]-t[0]):.1f} Hz\n")

    # ---- 3. validity FIRST: was it actually stationary? --------------------
    print("=== VALIDITY: was the car genuinely stationary? ===")
    n_rot = int((np.abs(yaw) > ROT_THRESH).sum())
    print(f"  |yaw_rate| max        : {np.abs(yaw).max():.5f} rad/s "
          f"({np.degrees(np.abs(yaw).max()):.2f} deg/s)")
    print(f"  samples > {ROT_THRESH} rad/s : {n_rot} / {len(yaw)} ({100*n_rot/len(yaw):.2f}%)")
    print(f"  v_ego max             : {np.max(v):.3f} m/s")
    stationary = n_rot == 0 and np.max(v) < 0.5
    if stationary:
        print("  => stationary confirmed. Drift numbers below are trustworthy.\n")
    else:
        print("  => *** NOT FULLY STATIONARY *** — the car rotated or moved.")
        print("     Treat the drift numbers below as SUSPECT for this window.\n")

    # ---- 1. summary --------------------------------------------------------
    print("=== YAW_OFFSET summary ===")
    uniq = np.unique(off)
    print(f"  min / max   : {off.min():.6f} / {off.max():.6f} rad/s")
    print(f"  span        : {off.max()-off.min():.6f} rad/s "
          f"= {(off.max()-off.min())/LSB:.2f} LSB")
    print(f"  mean        : {off.mean():.6f}   sd: {off.std():.6f} rad/s")
    print(f"  in deg/s    : mean {np.degrees(off.mean()):.4f}  span {np.degrees(off.max()-off.min()):.4f}")
    print(f"  distinct values: {len(uniq)}  -> {np.array2string(uniq[:8], precision=6)}"
          f"{' ...' if len(uniq) > 8 else ''}")
    if len(uniq) <= 3:
        print(f"  NOTE: only {len(uniq)} discrete level(s). At {LSB:.6f} rad/s per LSB the")
        print("        signal cannot express a small trend — quantisation dominates.")

    # ---- 2. regression -----------------------------------------------------
    print("\n=== DRIFT: linear fit of yaw_offset vs time ===")
    tm = (t - t[0]) / 60.0                      # minutes
    slope, intercept = np.polyfit(tm, off, 1)
    pred = slope * tm + intercept
    ss_res = float(((off - pred) ** 2).sum())
    ss_tot = float(((off - off.mean()) ** 2).sum())
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
    print(f"  slope       : {slope:+.8f} rad/s per minute")
    print(f"                {np.degrees(slope):+.6f} deg/s per minute")
    print(f"                {np.degrees(slope)*60:+.4f} deg/s per HOUR")
    print(f"  R^2         : {r2:.4f}")
    print(f"  first 10%   : mean {off[:len(off)//10].mean():.6f}")
    print(f"  last  10%   : mean {off[-len(off)//10:].mean():.6f}")
    print(f"  end - start : {off[-len(off)//10:].mean() - off[:len(off)//10].mean():+.6f} rad/s")

    # ---- verdict -----------------------------------------------------------
    print("\n=== VERDICT ===")
    span_lsb = (off.max() - off.min()) / LSB
    total_change = abs(slope) * dur_min
    if not stationary:
        v_txt = ("INCONCLUSIVE — the window was not stationary, so any apparent "
                 "trend may be real rotation rather than sensor drift.")
    elif len(uniq) <= 2 and span_lsb <= 1.01:
        v_txt = ("STABLE CALIBRATION CONSTANT — the value only ever occupies "
                 f"{len(uniq)} adjacent quantisation level(s). There is no drift to "
                 "correct; what movement exists is 1-LSB dither.")
    elif r2 < R2_REAL:
        v_txt = (f"STABLE (no credible trend) — slope is {slope:+.2e} rad/s/min but "
                 f"R^2 is only {r2:.3f}, i.e. the line explains almost none of the "
                 "variation. Bounded oscillation around a fixed mean, not drift.")
    elif total_change < LSB:
        v_txt = (f"STABLE IN PRACTICE — the fit is decent (R^2 {r2:.3f}) but the total "
                 f"modelled change over {dur_min:.1f} min is {total_change:.6f} rad/s, "
                 f"less than one {LSB:.6f} LSB. Not actionable.")
    else:
        v_txt = (f"GENUINE DRIFT — slope {slope:+.2e} rad/s/min with R^2 {r2:.3f}; "
                 f"total modelled change {total_change:.6f} rad/s over the window "
                 f"({total_change/LSB:.1f} LSB). A live bias correction is justified.")
    print("  " + v_txt)

    if stationary and off.mean() != 0:
        print(f"\n  For scale: integrating the MEAN offset {off.mean():.6f} rad/s "
              f"un-corrected\n  would add {np.degrees(off.mean())*60:.1f} deg of false heading per minute.")


if __name__ == "__main__":
    main()
