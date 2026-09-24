#!/usr/bin/env python3
"""
Proof for the `deltaref` odometry track's steering-angle reconstruction.

Pure Python — no rclpy, no cereal, no device. Run from the repo root:
    python3 claude/tests/test_deltaref_reconstruction.py

WHAT IS BEING PROVEN
  deltaref reconstructs the steering angle from the CHANGE between consecutive
  raw samples instead of subtracting a measured offset:
      theta += steer(t) - steer(t-1),  theta(t0) = 0
  Summing those deltas telescopes to steer(t) - steer(t0), so any CONSTANT
  sensor bias appears in both terms and cancels exactly, whatever its size.

  The tradeoff, proven just as explicitly below: it re-zeros ONCE at startup
  and holds that reference. It does NOT track drift that develops during the
  session. The `corrected` track does, via liveParameters.angleOffsetAverageDeg.
  Neither track is strictly better -- that is why both are published.
"""
import sys

TOL = 1e-9


def delta_reconstruct(raw_samples):
    """The deltaref method, matching odom_node.py exactly: first sample sets
    the zero reference and contributes no rotation."""
    theta, prev = 0.0, None
    out = []
    for s in raw_samples:
        if prev is None:
            prev = s
        else:
            theta += s - prev
            prev = s
        out.append(theta)
    return out


def fixed_offset(raw_samples, assumed_bias):
    """The OLD/alternative method: subtract a fixed assumed bias."""
    return [s - assumed_bias for s in raw_samples]


def live_offset(raw_samples, true_bias_per_sample):
    """What `corrected` does: subtract the CURRENT estimated bias each tick."""
    return [s - b for s, b in zip(raw_samples, true_bias_per_sample)]


PASS = FAIL = 0


def check(desc, ok):
    global PASS, FAIL
    if ok:
        PASS += 1; print(f"  PASS  {desc}")
    else:
        FAIL += 1; print(f"  FAIL  {desc}")


def close(a, b):
    return all(abs(x - y) < TOL for x, y in zip(a, b))


# Realistic motion: ramp out to 30 deg and back, then the other way.
TRUE = [0, 5, 10, 15, 20, 25, 30, 25, 20, 15, 10, 5, 0,
        -5, -10, -15, -10, -5, 0]

print(__doc__)
print("=" * 70)
print("1. CONSTANT bias cancels, regardless of magnitude or sign")
print("=" * 70)
for bias in (1.537, 5.0, -2.0, 0.0, 47.3):
    raw = [t + bias for t in TRUE]
    rec = delta_reconstruct(raw)
    check(f"bias {bias:+7.3f} deg -> reconstruction matches true angle", close(rec, TRUE))

print()
print("=" * 70)
print("2. the fixed-offset method only works if the constant is exactly right")
print("=" * 70)
ACTUAL_BIAS = 1.537
raw = [t + ACTUAL_BIAS for t in TRUE]
check("fixed offset, assumed == actual (1.537) -> correct",
      close(fixed_offset(raw, 1.537), TRUE))
check("fixed offset, assumed 1.200 (stale) -> WRONG, as expected",
      not close(fixed_offset(raw, 1.200), TRUE))
err = fixed_offset(raw, 1.200)[-1] - TRUE[-1]
print(f"        residual error with a 0.337 deg stale constant: {err:+.3f} deg on every sample")
check("delta reconstruction needs no constant at all -> correct",
      close(delta_reconstruct(raw), TRUE))

print()
print("=" * 70)
print("3. THE TRADEOFF: in-session drift. deltaref is blind to it by design.")
print("=" * 70)
# bias creeps 1.5 -> 3.0 over the session (temperature, realignment, wear)
drift = [1.5 + 1.5 * i / (len(TRUE) - 1) for i in range(len(TRUE))]
raw_drift = [t + b for t, b in zip(TRUE, drift)]

rec_d = delta_reconstruct(raw_drift)
live_d = live_offset(raw_drift, drift)

check("deltaref does NOT recover true angle under drift (documented, expected)",
      not close(rec_d, TRUE))
check("live-offset (what `corrected` uses) DOES recover it",
      close(live_d, TRUE))
check("deltaref residual equals the accumulated drift exactly",
      abs((rec_d[-1] - TRUE[-1]) - (drift[-1] - drift[0])) < TOL)
print(f"        drift accumulated over session : {drift[-1] - drift[0]:+.3f} deg")
print(f"        deltaref final error           : {rec_d[-1] - TRUE[-1]:+.3f} deg")
print(f"        corrected final error          : {live_d[-1] - TRUE[-1]:+.3f} deg")

print()
print("=" * 70)
print("4. INTEGRATION: the two tracks must genuinely diverge under drift")
print("=" * 70)
# Same synthetic sequence through both paths, same 14.2 ratio downstream.
RATIO = 14.2
import math
def tire(deg):    return math.radians(deg) / RATIO
dref_tire = [tire(x) for x in rec_d]
corr_tire = [tire(x) for x in live_d]
diverged = any(abs(a - b) > 1e-6 for a, b in zip(dref_tire, corr_tire))
check("deltaref and corrected produce DIFFERENT tire angles under drift", diverged)
maxdiff = max(abs(a - b) for a, b in zip(dref_tire, corr_tire))
print(f"        max tire-angle difference: {math.degrees(maxdiff):.4f} deg "
      f"({maxdiff:.6f} rad) -- expected divergence, not a bug")

# And with NO drift they must agree, proving the divergence is drift-caused.
raw_c = [t + 1.537 for t in TRUE]
same = close(delta_reconstruct(raw_c), live_offset(raw_c, [1.537] * len(TRUE)))
check("with a purely CONSTANT bias, the two tracks agree exactly", same)

print()
print("=" * 70)
print("5. first sample contributes no rotation (zero-reference capture)")
print("=" * 70)
check("first reconstructed value is exactly 0.0", delta_reconstruct(raw)[0] == 0.0)
check("single-sample input yields [0.0], no crash", delta_reconstruct([12.7]) == [0.0])
check("empty input yields [], no crash", delta_reconstruct([]) == [])

print(f"\n  {PASS} passed, {FAIL} failed")
sys.exit(0 if FAIL == 0 else 1)
