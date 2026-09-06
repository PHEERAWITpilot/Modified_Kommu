#!/usr/bin/env python3
"""
Numerical proof for the `startref` odometry track.

startref captures the RAW steering reading at startup as its base, then
accumulates per-tick deltas onto it:

    theta(t0) = steer(t0)
    theta(t)  = theta(t-1) + (steer(t) - steer(t-1))

Telescoping that sum:

    theta(t) = steer(t0) + SUM[ steer(i) - steer(i-1) ]
             = steer(t0) + steer(t) - steer(t0)
             = steer(t)

So theta is algebraically the raw steering angle itself. This script checks
that claim SAMPLE BY SAMPLE against a real log rather than asserting it, and
reports the maximum absolute difference in degrees.

  python3 claude/tests/test_startref_equivalence.py <log.jsonl>
"""
import json
import sys


def startref_theta(steer_series):
    """Exactly as implemented in odom_node.py's startref path."""
    theta = None
    prev = None
    out = []
    for s in steer_series:
        if prev is None:
            theta = s          # base = the REAL starting reading, not 0.0
            prev = s
        else:
            theta += s - prev
            prev = s
        out.append(theta)
    return out


def deltaref_theta(steer_series):
    """The existing deltaref track, for contrast: base = 0.0."""
    theta, prev, out = 0.0, None, []
    for s in steer_series:
        if prev is None:
            prev = s
        else:
            theta += s - prev
            prev = s
        out.append(theta)
    return out


def main():
    if len(sys.argv) < 2:
        sys.exit("usage: test_startref_equivalence.py <log.jsonl>")
    path = sys.argv[1]
    steer = []
    for line in open(path):
        line = line.strip()
        if not line:
            continue
        try:
            d = json.loads(line)
        except json.JSONDecodeError:
            continue
        if d.get("marker"):
            continue
        if d.get("steer_deg_raw") is None:
            continue
        steer.append(float(d["steer_deg_raw"]))
    if not steer:
        sys.exit("no steer_deg_raw samples found")

    sr = startref_theta(steer)
    dr = deltaref_theta(steer)

    diffs = [abs(a - b) for a, b in zip(sr, steer)]
    max_diff = max(diffs)
    n_nonzero = sum(1 for x in diffs if x != 0.0)

    print(f"  log            : {path}")
    print(f"  samples        : {len(steer)}")
    print(f"  steer range    : {min(steer):.1f} .. {max(steer):.1f} deg")
    print()
    print("  === startref theta  vs  raw steer_deg_raw ===")
    print(f"  MAX ABSOLUTE DIFFERENCE : {max_diff!r} deg")
    print(f"  samples differing at all: {n_nonzero} / {len(steer)}")
    print(f"  bitwise identical       : {sr == steer}")
    print()
    print("  === for contrast: deltaref theta vs raw steer_deg_raw ===")
    d2 = [abs(a - b) for a, b in zip(dr, steer)]
    print(f"  max abs difference      : {max(d2):.6f} deg")
    print(f"  (equals |steer(t0)| = {abs(steer[0]):.6f} — the constant deltaref discards)")
    print()
    # Tolerance, not exact equality: theta is an ACCUMULATED sum, so it differs
    # from the direct value by float rounding that grows with sample count. At
    # 1e-9 deg the two are the same number for every physical purpose -- one LSB
    # of the steering sensor is 0.1 deg, eight orders of magnitude larger.
    TOL = 1e-9
    if max_diff <= TOL:
        print(f"  CONFIRMED: startref reproduces the raw, uncorrected steering angle.")
        print(f"  Max difference {max_diff:.3e} deg is float accumulation noise --")
        print(f"  {0.1/max_diff:.1e}x smaller than the sensor's own 0.1 deg resolution.")
        print(f"  This track IS the deprecated kinematic track's input, at ratio 14.2.")
        return 0
    print(f"  CONTRADICTED: max difference {max_diff!r} deg exceeds {TOL:g}.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
