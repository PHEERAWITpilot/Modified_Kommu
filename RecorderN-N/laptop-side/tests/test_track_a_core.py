#!/usr/bin/env python3
"""Tests for byd_odom_ros/track_a_core.py.

Two parts:
  1. unit checks of each pure function;
  2. acceptance: replay every recorded Odom_record/**/odom.csv through the
     module and compare to what the live node recorded -- the speed gate
     against v_integrated_ms, the yaw rate against trackA_yaw_rate (exact),
     and the integrated pose against trackA_persample_x/y/yaw_deg.

Replay notes. Each CSV row is written after one successful integration step,
and t_mono is the same `t` the node used for dt, so consecutive rows give the
exact dt -- except after a gap > DT_MAX_S, where the node consumed an
unrecorded tick; those rows are re-seeded from the recording and counted.
The pose is seeded from the first row; its yaw is stored in degrees, so the
seed can be 1 ulp off, hence the 1e-9 tolerance alongside the exact count.
"""
import csv
import glob
import importlib.util
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
CORE = os.environ.get("TRACK_A_CORE", os.path.join(
    HERE, "..", "byd_odom_ros", "byd_odom_ros", "track_a_core.py"))
RECORD_DIR = os.environ.get("ODOM_RECORD_DIR", os.path.join(HERE, "..", "..", "Odom_record"))
TOL = 1e-9

spec = importlib.util.spec_from_file_location("track_a_core", CORE)
T = importlib.util.module_from_spec(spec)
spec.loader.exec_module(T)

passed = failed = 0


def check(name, cond, detail=""):
    global passed, failed
    if cond:
        passed += 1
        print("  PASS  " + name)
    else:
        failed += 1
        print("  FAIL  " + name + ("  -- " + detail if detail else ""))


print("1. module is stdlib-only")
src = open(CORE).read()
imports = [l.strip() for l in src.splitlines() if l.startswith(("import ", "from "))]
check("only `import math`", imports == ["import math"], str(imports))

print("2. advance_clock")
check("first sample -> no dt", T.advance_clock(None, 5.0) == (5.0, None))
check("normal step", T.advance_clock(5.0, 5.02) == (5.02, 5.02 - 5.0))
check("dt == 0 rejected, last_t advances", T.advance_clock(5.0, 5.0) == (5.0, None))
check("negative dt rejected", T.advance_clock(5.0, 4.9) == (4.9, None))
check("dt exactly DT_MAX_S accepted", T.advance_clock(0.0, 1.0) == (1.0, 1.0))
check("dt > DT_MAX_S rejected, last_t advances", T.advance_clock(0.0, 1.0001) == (1.0001, None))
check("is_stale strict >", (not T.is_stale(10.5, 10.0)) and T.is_stale(10.5001, 10.0))

print("3. speed_gate / standstill_gate")
check("below 0.05 zeroed", T.speed_gate(0.049) == 0.0 and T.speed_gate(-0.049) == 0.0)
check("at/above 0.05 passes, sign kept", T.speed_gate(0.05) == 0.05 and T.speed_gate(-0.2) == -0.2)
check("standstill zeroes yaw", T.standstill_gate(0.3, 0.01) == 0.0)
check("moving keeps yaw", T.standstill_gate(0.3, 1.0) == 0.3)

print("4. sanitize_can_sample is the only boundary check")
check("valid sample", T.sanitize_can_sample(0.1, 0.02, True) == (0.1, 0.02, True))
for bad in (float("nan"), float("inf"), -float("inf"), None, "abc"):
    check("rate %r -> ok False" % (bad,), T.sanitize_can_sample(bad, 0.0, True)[2] is False)
    check("offset %r -> ok False" % (bad,), T.sanitize_can_sample(0.1, bad, True)[2] is False)
check("sensor not ok -> ok False", T.sanitize_can_sample(0.1, 0.0, False)[2] is False)
check("numeric string accepted", T.sanitize_can_sample("0.5", "0.25", True) == (0.5, 0.25, True))
check("NaN yields 0.0 yaw, never NaN", T.track_a_yaw_rate(float("nan"), 0.0, True, 5.0) == 0.0)

print("5. Integrator")
it = T.Integrator()
it.step(10.0, 0.0, 0.1)
check("straight line", (it.x, it.y, it.yaw) == (1.0, 0.0, 0.0))
it = T.Integrator()
for _ in range(2000):
    it.step(0.0, 3.0, 0.01)
check("yaw wrapped to [-pi, pi]", -math.pi <= it.yaw <= math.pi)

print("6. acceptance replay against recorded odom.csv")
files = sorted(glob.glob(os.path.join(RECORD_DIR, "**", "odom.csv"), recursive=True))
check("recordings found under %s" % RECORD_DIR, len(files) > 0, "none")
tot_rows = tot_gate = tot_yaw = tot_pose_exact = tot_reseed = 0
worst = 0.0
bad_files = []
for path in files:
    rows = list(csv.DictReader(open(path)))
    if not rows or "trackA_persample_x" not in rows[0]:
        continue
    f_gate = f_yaw = 0
    f_worst = 0.0
    it = T.Integrator()
    prev_t = None
    for r in rows:
        tot_rows += 1
        t = float(r["t_mono"])
        v_int = float(r["v_integrated_ms"])
        if T.speed_gate(float(r["v_signed_ms"])) != v_int:
            f_gate += 1
        v_ms = float(r["v_kmh"]) / 3.6
        yaw = T.track_a_yaw_rate(r["yaw_sensor_rate"], r["yaw_sensor_offset"],
                                 bool(int(r["yaw_sensor_ok"])), v_ms)
        if yaw != float(r["trackA_yaw_rate"]):
            f_yaw += 1
        rx, ry, ryaw = (float(r["trackA_persample_x"]), float(r["trackA_persample_y"]),
                        float(r["trackA_persample_yaw_deg"]))
        dt = None if prev_t is None else t - prev_t
        if dt is None or dt <= 0.0 or dt > T.DT_MAX_S:
            if dt is not None:
                tot_reseed += 1
            it.x, it.y, it.yaw = rx, ry, math.radians(ryaw)
        else:
            it.step(v_int, float(r["trackA_yaw_rate"]), dt)
        prev_t = t
        err = max(abs(it.x - rx), abs(it.y - ry), abs(math.degrees(it.yaw) - ryaw))
        if err == 0.0:
            tot_pose_exact += 1
        f_worst = max(f_worst, err)
    tot_gate += f_gate
    tot_yaw += f_yaw
    worst = max(worst, f_worst)
    if f_gate or f_yaw or f_worst > TOL:
        bad_files.append("%s gate=%d yaw=%d maxerr=%.3g" % (
            os.path.relpath(path, RECORD_DIR), f_gate, f_yaw, f_worst))

print("   %d recordings, %d rows, %d re-seeds after >%.1fs gaps, pose bit-exact on %d rows (%.4f%%), worst |err| %.3g"
      % (len(files), tot_rows, tot_reseed, T.DT_MAX_S, tot_pose_exact,
         100.0 * tot_pose_exact / max(tot_rows, 1), worst))
check("speed_gate matches v_integrated_ms on every row", tot_gate == 0, "%d mismatches" % tot_gate)
check("track_a_yaw_rate matches trackA_yaw_rate EXACTLY on every row", tot_yaw == 0, "%d mismatches" % tot_yaw)
check("integrated pose within %g of recorded on every row" % TOL, worst <= TOL, "worst %.3g" % worst)
for b in bad_files[:10]:
    print("        " + b)

print("\n%d passed, %d failed" % (passed, failed))
sys.exit(1 if failed else 0)
