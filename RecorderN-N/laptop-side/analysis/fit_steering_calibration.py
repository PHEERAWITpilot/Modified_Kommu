#!/usr/bin/env python3
"""
fit_steering_calibration.py — closed-loop fit of (angle offset, steer ratio).

Standalone: no ROS, no device. Operates on a JSONL from
`byd_yaw_sensor_probe.py --log` that contains manual MARKER records.

THE IDEA
  Drive loops of KNOWN total rotation (a full circle = 360 deg, a figure-eight
  = 0, etc). Mark the start and end of each. The integrated heading from the
  bicycle model must equal that known rotation. Two unknowns -- the steering
  centre offset and the steer ratio -- are then fitted against several loops AT
  ONCE.

WHY SEVERAL LOOPS, FITTED TOGETHER
  A single loop cannot separate the two. An offset error adds rotation
  proportional to TIME; a ratio error scales rotation proportional to the
  STEERING ACTUALLY USED. One loop gives one equation for two unknowns, so
  infinitely many (offset, ratio) pairs fit it exactly. Loops that differ in
  duration, speed and steering magnitude break that degeneracy -- which is why
  this fits all segments simultaneously and then checks how well-constrained
  the answer actually is.

TRUE ROTATION COMES FROM YOU, NOT FROM THE SCRIPT
  Pass --true per segment. Use what you actually measured at drive time
  (e.g. 355 if the loop visibly failed to close by a few degrees), not an
  assumed 360.

INDEPENDENT CROSS-CHECK
  The same segments are integrated from `yaw_sensor_rate` (the car's own yaw
  sensor), which needs neither offset nor ratio. If that disagrees with your
  stated true rotation, the loops -- or the stated values -- are suspect, and
  the fit below is being anchored to a bad reference.

USAGE
  python3 claude/fit_steering_calibration.py drive.jsonl --true 360,360,-360,720
  python3 claude/fit_steering_calibration.py drive.jsonl --list      # segments only
"""
import argparse
import json
import math
import os
import re
import sys

import numpy as np

# --- reuse the node's own math, without importing ROS ------------------------
_NODE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "byd_odom_ros", "byd_odom_ros", "odom_node.py")
_ns = {"math": math}
if not os.path.isfile(_NODE):
    sys.exit(f"odom_node.py not found at {_NODE} — cannot reuse its math")
_src = open(_NODE).read()
for _fn in ("tire_angle_rad", "heading_rate_rad_s"):
    _m = re.search(rf"^def {_fn}\(.*?\n(?:(?:    .*)?\n)*", _src, re.M)
    if _m is None:
        sys.exit(f"could not extract {_fn}() from odom_node.py")
    exec(_m.group(0), _ns)
tire_angle_rad = _ns["tire_angle_rad"]
heading_rate_rad_s = _ns["heading_rate_rad_s"]

WHEELBASE = 2.70


def load(path):
    data, marks = [], []
    for line in open(path):
        line = line.strip()
        if not line:
            continue
        try:
            d = json.loads(line)
        except json.JSONDecodeError:
            continue
        if d.get("marker"):
            marks.append(d)
        elif d.get("steer_deg_raw") is not None and d.get("t") is not None:
            data.append(d)
    return data, marks


def segments(data, marks):
    """Consecutive marker pairs bound one segment each."""
    out = []
    ts = np.array([d["t"] for d in data])
    for i in range(len(marks) - 1):
        a, b = marks[i]["t"], marks[i + 1]["t"]
        lo, hi = int(np.searchsorted(ts, a)), int(np.searchsorted(ts, b))
        if hi - lo < 20:
            continue
        seg = data[lo:hi]
        out.append({
            "idx": len(out),
            "label": f"{marks[i].get('label')} -> {marks[i+1].get('label')}",
            "t": np.array([d["t"] for d in seg]),
            "steer": np.array([d["steer_deg_raw"] for d in seg]),
            "v": np.array([(d.get("v_ego") or 0.0) for d in seg]),
            "yaw": np.array([(d.get("yaw_rate") or 0.0) for d in seg]),
        })
    return out


def integrate(seg, offset_deg, ratio):
    """Heading change in degrees from the bicycle model."""
    dt = np.diff(seg["t"], prepend=seg["t"][0])
    tot = 0.0
    for st, vv, d_t in zip(seg["steer"], seg["v"], dt):
        tot += heading_rate_rad_s(vv, tire_angle_rad(st - offset_deg, ratio), WHEELBASE) * d_t
    return math.degrees(tot)


def integrate_yaw(seg):
    """Heading change in degrees from the car's own yaw sensor."""
    dt = np.diff(seg["t"], prepend=seg["t"][0])
    return math.degrees(float(np.sum(seg["yaw"] * dt)))


def cost(segs, truths, offset, ratio):
    return sum((integrate(s, offset, ratio) - tv) ** 2 for s, tv in zip(segs, truths))


def fit(segs, truths, o_rng=(-8.0, 8.0), r_rng=(10.0, 20.0)):
    """Coarse grid then successive refinement. No scipy dependency (it is not
    installed in the project venv)."""
    o_lo, o_hi = o_rng
    r_lo, r_hi = r_rng
    best = (None, None, float("inf"))
    for _ in range(6):
        os_ = np.linspace(o_lo, o_hi, 41)
        rs_ = np.linspace(r_lo, r_hi, 41)
        for o in os_:
            for r in rs_:
                c = cost(segs, truths, o, r)
                if c < best[2]:
                    best = (float(o), float(r), float(c))
        o, r = best[0], best[1]
        do, dr = (o_hi - o_lo) / 10.0, (r_hi - r_lo) / 10.0
        o_lo, o_hi = o - do, o + do
        r_lo, r_hi = max(1.0, r - dr), r + dr
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--true", dest="truth",
                    help="comma-separated TRUE heading change per segment, deg "
                         "(e.g. 360,360,-360). Signs matter.")
    ap.add_argument("--list", action="store_true", help="list segments and exit")
    args = ap.parse_args()

    data, marks = load(args.log)
    print(f"  {len(data)} data rows, {len(marks)} marker(s)")
    if len(marks) < 2:
        sys.exit("  need at least 2 markers to bound a segment")
    segs = segments(data, marks)
    if not segs:
        sys.exit("  no usable segments between markers")

    print(f"\n  {len(segs)} segment(s):")
    print(f"  {'seg':>3} {'label':<28} {'dur':>6} {'v_mean':>7} {'|steer|max':>10} {'yawsens_deg':>12}")
    for s in segs:
        print(f"  {s['idx']:>3} {s['label'][:28]:<28} {s['t'][-1]-s['t'][0]:6.1f} "
              f"{s['v'].mean():7.2f} {np.abs(s['steer']).max():10.1f} {integrate_yaw(s):12.1f}")
    if args.list or not args.truth:
        if not args.truth:
            print("\n  give --true <deg,deg,...> (one per segment) to fit")
        return 0

    truths = [float(x) for x in args.truth.split(",")]
    if len(truths) != len(segs):
        sys.exit(f"  --true has {len(truths)} values but there are {len(segs)} segments")

    print("\n=== INDEPENDENT CROSS-CHECK: the car's own yaw sensor ===")
    print("  (needs no ratio and no offset — if this disagrees with your stated")
    print("   truth, the loops or the stated values are suspect)")
    print(f"  {'seg':>3} {'stated_true':>12} {'yaw_sensor':>12} {'diff':>9}")
    for s, tv in zip(segs, truths):
        ys = integrate_yaw(s)
        print(f"  {s['idx']:>3} {tv:12.1f} {ys:12.1f} {ys-tv:+9.1f}")

    print("\n=== FIT (all segments simultaneously) ===")
    o, r, c = fit(segs, truths)
    print(f"  offset_deg  = {o:+.3f}")
    print(f"  steer_ratio = {r:.3f}")
    print(f"  RMS residual = {math.sqrt(c/len(segs)):.2f} deg")

    print(f"\n  {'seg':>3} {'true':>8} {'before':>9} {'err':>8} {'after':>9} {'err':>8}")
    BEFORE_O, BEFORE_R = 0.0, 14.2
    for s, tv in zip(segs, truths):
        b = integrate(s, BEFORE_O, BEFORE_R)
        a = integrate(s, o, r)
        print(f"  {s['idx']:>3} {tv:8.1f} {b:9.1f} {b-tv:+8.1f} {a:9.1f} {a-tv:+8.1f}")
    print(f"  ('before' = offset 0, ratio {BEFORE_R} — the current corrected-track defaults)")

    # --- is the fit actually constrained? ---
    print("\n=== IS THE FIT WELL-CONSTRAINED? (leave-one-out) ===")
    if len(segs) < 3:
        print("  fewer than 3 segments — cannot test. Treat the fit as UNRELIABLE:")
        print("  offset and ratio are degenerate without several varied loops.")
    else:
        os_, rs_ = [], []
        for k in range(len(segs)):
            sub = [s for j, s in enumerate(segs) if j != k]
            subt = [t for j, t in enumerate(truths) if j != k]
            oo, rr, _ = fit(sub, subt)
            os_.append(oo); rs_.append(rr)
            print(f"  without seg {k}: offset {oo:+.3f}  ratio {rr:.3f}")
        o_sd, r_sd = float(np.std(os_)), float(np.std(rs_))
        print(f"\n  spread: offset sd {o_sd:.3f} deg, ratio sd {r_sd:.3f}")
        if o_sd > 0.5 or r_sd > 0.5:
            print("  => POORLY CONSTRAINED. Dropping one loop moves the answer a lot,")
            print("     which means the loops were not varied enough (duration, speed,")
            print("     steering magnitude). Treat these numbers as provisional.")
        else:
            print("  => well constrained: the answer does not hinge on any single loop.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
