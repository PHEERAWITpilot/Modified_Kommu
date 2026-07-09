#!/usr/bin/env python3
# Kommu.AI — Manual Steering Target Writer (DEVICE SIDE, headless)
#
# WHY THIS EXISTS
#   The manual-steer SENDER (manual_steer_sender.py) needs a display + keyboard,
#   which the Kommu device does not have (headless; pygame falls back to a dummy
#   video driver and receives no key events). So the sender runs on the laptop
#   (INC), and this tiny writer runs ON THE DEVICE to land the target where the
#   carcontroller hook reads it.
#
#   Data flow:
#     [INC] manual_steer_sender.py  --(TCP {angle_deg})-->  [DEVICE] this writer
#         --> atomic write /tmp/manual_steer.json {angle_deg, ts=device clock}
#         --> carcontroller manual_steer_hook reads it (same machine, same /tmp)
#
# WHAT THIS IS (deliberately minimal — the "one number only" discipline)
#   * Receives a desired steering angle (float, degrees) over a TCP socket.
#   * Stamps the timestamp WITH THE DEVICE'S OWN CLOCK at write time, so the
#     hook's freshness check (STALE_S=0.20) is measured device-clock-to-device-
#     clock. No cross-machine clock-sync concern.
#   * Atomically writes /tmp/manual_steer.json (temp + os.replace) so the hook
#     never reads a half-written file.
#
# WHAT THIS IS NOT
#   * It does NOT encode CAN, compute checksums, cycle counters, or open the
#     Panda. It carries ONE number across the wire. All CAN/limit/gating logic
#     stays in the carcontroller + manual_steer_hook, unchanged.
#   * It does NOT gate on enable-file / speed / lat_active. Those gates all live
#     in the hook (next to the actuation), on purpose. This writer is dumb: it
#     writes whatever fresh angle arrives. The hook decides whether to USE it.
#   * It cannot by itself move anything. With /tmp/MANUAL_STEER_ENABLE absent
#     (the default), the hook no-ops regardless of what this writes.
#
# DEAD-MAN BEHAVIOUR (why stopping is safe)
#   The writer only writes when a fresh angle arrives from the sender. If the
#   sender stops (key released in hold mode still streams; but window close,
#   quit, or network drop stops the stream), NO new writes happen -> the file
#   goes stale within STALE_S -> the hook falls back to the stock planner angle.
#   The kill file `rm /tmp/MANUAL_STEER_ENABLE` is an independent instant revert.
#
# RUN ON DEVICE (bukapilot can stay running — no Panda/USB conflict):
#   cd /data/openpilot
#   screen -S msteer
#   python3 manual_steer_writer.py            # listens on 0.0.0.0:5557
#   # Ctrl+A then D to detach — writer keeps running
#
# PROTOCOL (newline-delimited JSON, one object per line):
#   sender -> writer:  {"angle_deg": <float>}\n
#   The writer ignores any 'ts' the sender might send and stamps its own.
#   A line that fails to parse or lacks a numeric angle_deg is skipped (no write,
#   no crash) so a garbled packet cannot poison the target.
#
# No external deps beyond stdlib.

import argparse
import json
import os
import socket
import time

TARGET_FILE = "/tmp/manual_steer.json"

# Hard clamp on this side too (defence in depth). The sender clamps, the hook
# clamps again to MANUAL_ABS_MAX_DEG, and the device _compute_apply_angle clamps
# to +/-10deg-of-measured + rate limit. This is the outermost of several clamps.
WRITER_ABS_MAX_DEG = 15.0


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def atomic_write_target(path, angle_deg):
    """Atomically write {angle_deg, ts} with the DEVICE's own wall clock.

    temp + os.replace is atomic on POSIX, so the hook's json.load never sees a
    partially written file. ts is stamped here (device side) so the hook's
    freshness window is same-clock.
    """
    payload = {
        "angle_deg": round(float(angle_deg), 3),
        "ts": time.time(),          # DEVICE clock — matches the hook's time.time()
    }
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(payload, f)
    os.replace(tmp, path)


def handle_client(conn, addr, target_file, abs_max_deg, verbose):
    print(f"[writer] client connected: {addr[0]}:{addr[1]}")
    buf = b""
    n_written = 0
    try:
        # Line-buffered read loop. Each complete line is one target update.
        while True:
            chunk = conn.recv(4096)
            if not chunk:
                break                      # client closed -> stop writing (dead-man)
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.strip()
                if not line:
                    continue
                try:
                    obj = json.loads(line.decode("utf-8"))
                    angle = float(obj["angle_deg"])   # KeyError/ValueError -> skip
                except (ValueError, KeyError, TypeError, UnicodeDecodeError):
                    # Garbled or incomplete packet: skip it. Do NOT write, do NOT
                    # crash. A bad packet must never poison the target file.
                    continue
                angle = _clamp(angle, -abs_max_deg, abs_max_deg)
                try:
                    atomic_write_target(target_file, angle)
                    n_written += 1
                    if verbose and (n_written % 50 == 0):
                        print(f"[writer] wrote {n_written} targets "
                              f"(last angle_deg={angle:+.2f})")
                except OSError as e:
                    # If we cannot write, do not crash the server; the file simply
                    # goes stale and the hook falls back to stock.
                    print(f"[writer] WARN could not write {target_file}: {e}")
    except (ConnectionResetError, BrokenPipeError):
        pass
    finally:
        conn.close()
        print(f"[writer] client disconnected ({addr[0]}:{addr[1]}); "
              f"wrote {n_written} targets. Target file left as-is; it will go "
              f"stale and the hook will fall back to stock within STALE_S.")


def run(host, port, target_file, abs_max_deg, verbose):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(1)                          # one sender at a time
    print(f"[writer] listening on {host}:{port} -> {target_file}")
    print(f"[writer] clamp +/-{abs_max_deg:.0f} deg. Enable file "
          f"/tmp/MANUAL_STEER_ENABLE must be present on THIS device for the hook "
          f"to use any target. This writer does NOT create it.")
    print("[writer] Ctrl+C to stop. (Run under `screen -S msteer` to survive SSH "
          "disconnect.)")
    try:
        while True:
            # Accept one client, serve it until it disconnects, then wait for the
            # next. Sequential (not threaded) — only one sender should ever drive.
            conn, addr = srv.accept()
            conn.setblocking(True)
            handle_client(conn, addr, target_file, abs_max_deg, verbose)
    except KeyboardInterrupt:
        print("\n[writer] stopped.")
    finally:
        srv.close()


def main():
    p = argparse.ArgumentParser(description="Manual-steer target writer (device side)")
    p.add_argument("--host", default="0.0.0.0",
                   help="bind address (default 0.0.0.0 — reachable over WiFi/hotspot)")
    p.add_argument("--port", type=int, default=5557,
                   help="listen port (default 5557; 5555/5556 are the CAN/cereal servers)")
    p.add_argument("--out", default=TARGET_FILE,
                   help=f"target file path (default {TARGET_FILE})")
    p.add_argument("--max-angle", dest="max_angle", type=float, default=WRITER_ABS_MAX_DEG,
                   help=f"outermost clamp on received angle, deg (default {WRITER_ABS_MAX_DEG})")
    p.add_argument("--verbose", action="store_true", help="log every 50th write")
    args = p.parse_args()
    run(args.host, args.port, args.out, args.max_angle, args.verbose)


if __name__ == "__main__":
    main()
