#!/usr/bin/env python3
"""
byd_yaw_sensor_probe.py — READ-ONLY probe for BYD YAW_SENSOR (0x222 / 546).

Decodes through opendbc's own CANParser (never cantools — this DBC has C-style
comments cantools rejects), so the scaling matches what the car port would get.

  BO_ 546 YAW_SENSOR: 8
    SG_ YAW_RATE   : 0|12@1+ (0.002132603, -2.094216146)
    SG_ YAW_OFFSET : 12|12@1+ (0.002132603, -0.130088783)

WHY THIS EXISTS
  cs.yawRate is never assigned anywhere in the BYD port, so the `measured`
  odometry track integrates zero forever. This checks whether the car actually
  publishes a real yaw rate on CAN that could fill that gap — no steering
  ratio, no centre offset, no bicycle model.

⚠️ A YAW-RATE SENSOR DOES NOT RESPOND TO STEERING INPUT WHILE PARKED.
  It measures the car actually rotating. Turning the wheel with the car
  stationary should read ~0 — that is CORRECT, not a failure. The real test is
  a drive: the value must go positive one way round a turn, negative the other,
  and sit near zero on a straight.

RUN ON DEVICE (read-only, no TX, safe alongside a running bukapilot):
  python3 /data/openpilot/byd_yaw_sensor_probe.py --seconds 60
"""
import argparse
import json
import math
import os
import queue
import sys
import threading
import time

OPENPILOT_PATH = os.environ.get("OPENPILOT_PATH", "/data/openpilot")
if OPENPILOT_PATH not in sys.path:
    sys.path.insert(0, OPENPILOT_PATH)

try:
    import cereal.messaging as messaging
    from opendbc.can.parser import CANParser
    # Same path card.py uses: raw capnp strings -> list -> CANParser.update()
    from openpilot.selfdrive.pandad import can_capnp_to_list
except ImportError as e:
    sys.exit(f"ERROR: run inside the device env ({e}). "
             f"Try: cd /data/openpilot && python3 byd_yaw_sensor_probe.py")

DBC_NAME = "byd_general_pt"
MSG = "YAW_SENSOR"
STEER_MSG = "STEER_MODULE_2"   # 287 / 0x11F, carries STEER_ANGLE_2
WHEEL_MSG = "WHEEL_SPEED"      # 496, four per-wheel speeds
ADDR = 546
BUS = 0


def _marker_listener(q):
    """Read labels from stdin on a DAEMON thread and hand them to the main loop
    via a queue. The main loop stays the ONLY writer to the log file, so the
    ~95Hz capture is never blocked by terminal I/O and the file can never be
    interleaved mid-line. Blank line = unlabelled marker."""
    n = 0
    while True:
        try:
            label = sys.stdin.readline()
        except Exception:
            return
        if label == "":            # EOF (stdin closed / detached run)
            return
        n += 1
        q.put(label.strip() or f"mark{n}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--bus", type=int, default=BUS)
    ap.add_argument("--log", metavar="PATH", nargs="?", const="AUTO", default=None,
                    help="write FULL-RATE JSONL. Captures everything a later "
                         "steer-ratio fit needs (raw steer, yaw, per-wheel speeds, "
                         "offsets) on ONE CAN timebase, so that analysis never "
                         "requires a second drive.")
    args = ap.parse_args()

    # All three decoded from the SAME CAN drain => one timebase, no cereal-side
    # filtering or timing skew between yaw and steer. That matters for the lag
    # compensation a proper ratio fit will need.
    cp = CANParser(DBC_NAME, [(MSG, 50), (STEER_MSG, 100), (WHEEL_MSG, 50)], args.bus)

    log_f = None
    marker_q = None
    n_markers = 0
    if args.log is not None:
        path = args.log
        if path == "AUTO":
            path = f"/data/openpilot/yaw_probe_{time.strftime('%Y%m%d_%H%M%S')}.jsonl"
        log_f = open(path, "w")
        print(f"full-rate log -> {path}")
        # Manual checkpoint markers: type an optional label, press Enter.
        # Only when stdin is a real terminal — a detached run has no keyboard.
        if sys.stdin is not None and sys.stdin.isatty():
            marker_q = queue.Queue()
            threading.Thread(target=_marker_listener, args=(marker_q,),
                             daemon=True).start()
            print("MARKERS ENABLED: type a label (or nothing) + Enter to mark the log.")
        else:
            print("markers disabled (stdin is not a terminal)")
    sub_can = messaging.sub_sock("can", timeout=100)
    sm = messaging.SubMaster(["carState", "liveParameters"])

    print(f"probing {MSG} (0x{ADDR:X}/{ADDR}) on bus {args.bus} for {args.seconds:.0f}s")
    print("NOTE: parked, expect ~0. It responds to the CAR ROTATING, not to steering input.\n")
    print(f"  {'t':>6} {'YAW_RATE':>10} {'deg/s':>8} {'YAW_OFFSET':>11} "
          f"{'v_kmh':>7} {'steer':>7}  {'alive':>5}")

    start = time.monotonic()
    n = 0
    seen_min = seen_max = None
    last_print = 0.0
    while time.monotonic() - start < args.seconds:
        can_strs = messaging.drain_sock_raw(sub_can, wait_for_one=True)
        cp.update(can_capnp_to_list(can_strs))
        sm.update(0)
        t = time.monotonic() - start
        yr = cp.vl[MSG]["YAW_RATE"]
        yo = cp.vl[MSG]["YAW_OFFSET"]
        alive = cp.can_valid

        if log_f is not None and marker_q is not None:
            # Non-blocking drain. Markers share the exact clock of the data rows.
            while True:
                try:
                    lbl = marker_q.get_nowait()
                except queue.Empty:
                    break
                n_markers += 1
                log_f.write(json.dumps({
                    "marker": True, "label": lbl,
                    "t": t, "wall": time.time(),
                    "yaw_rate": yr, "steer_deg_raw": cp.vl[STEER_MSG]["STEER_ANGLE_2"],
                }) + "\n")
                print(f"  [MARK {n_markers}] t={t:.3f}s  label={lbl!r}")

        if log_f is not None:
            cs_ok = bool(sm.recv_frame["carState"])
            cs = sm["carState"]
            w = cp.vl[WHEEL_MSG]
            log_f.write(json.dumps({
                "t": t,                                   # s since probe start
                "wall": time.time(),
                # --- the reference signal ---
                "yaw_rate": yr,                           # rad/s, from CAN 546
                "yaw_offset": yo,
                # --- RAW, UNCORRECTED steer, same CAN timebase as yaw ---
                "steer_deg_raw": cp.vl[STEER_MSG]["STEER_ANGLE_2"],
                # --- so the fit can offset-correct later without a new drive ---
                "angle_offset_deg": (sm["liveParameters"].angleOffsetAverageDeg
                                     if sm.recv_frame["liveParameters"] else None),
                "steer_ratio_live": (sm["liveParameters"].steerRatio
                                     if sm.recv_frame["liveParameters"] else None),
                "steer_deg_cereal": cs.steeringAngleDeg if cs_ok else None,
                # --- speed: per-wheel AND filtered, for distance + a possible
                #     differential-yaw cross-check ---
                "wheel_fl": w["WHEELSPEED_FL"], "wheel_fr": w["WHEELSPEED_FR"],
                "wheel_bl": w["WHEELSPEED_BL"], "wheel_br": w["WHEELSPEED_BR"],
                "v_ego": cs.vEgo if cs_ok else None,
                "a_ego": cs.aEgo if cs_ok else None,
                "standstill": bool(cs.standstill) if cs_ok else None,
                "can_valid": bool(alive),
            }) + "\n")
        n += 1
        seen_min = yr if seen_min is None else min(seen_min, yr)
        seen_max = yr if seen_max is None else max(seen_max, yr)
        if t - last_print >= 0.5:
            last_print = t
            v = sm["carState"].vEgo * 3.6 if sm.recv_frame["carState"] else float("nan")
            st = sm["carState"].steeringAngleDeg if sm.recv_frame["carState"] else float("nan")
            print(f"  {t:6.1f} {yr:10.5f} {math.degrees(yr):8.2f} {yo:11.5f} "
                  f"{v:7.2f} {st:7.2f}  {str(alive):>5}")

    if log_f is not None:
        log_f.close()
        if n_markers:
            print(f"  {n_markers} marker(s) written")
    print(f"\n  samples={n}  YAW_RATE range: {seen_min:.5f} .. {seen_max:.5f} rad/s "
          f"({math.degrees(seen_min):.2f} .. {math.degrees(seen_max):.2f} deg/s)")
    span = (seen_max - seen_min) if (seen_min is not None) else 0.0
    if span < 1e-4:
        print("  => value never changed. Either the car never rotated, or the signal is static.")
        print("     Re-run during an actual drive with turns before concluding anything.")
    else:
        print("  => value MOVED. Confirm the sign matches turn direction before trusting it.")


if __name__ == "__main__":
    main()
