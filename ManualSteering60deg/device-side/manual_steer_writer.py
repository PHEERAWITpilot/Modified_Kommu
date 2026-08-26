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
#     [INC] manual_steer_sender.py  --(TCP {angle_deg, active})-->  [DEVICE] this writer
#         --> atomic write /tmp/manual_steer.json {angle_deg, ts=device clock, active}
#         --> carcontroller manual_steer_hook reads it (same machine, same /tmp)
#
# WHAT THIS IS (deliberately minimal — the "one number only" discipline)
#   * Receives a desired steering angle (float, degrees) plus an override-active
#     flag (bool) over a TCP socket.
#   * Stamps the timestamp WITH THE DEVICE'S OWN CLOCK at write time, so the
#     hook's freshness check (STALE_S=0.20) is measured device-clock-to-device-
#     clock. No cross-machine clock-sync concern.
#   * Atomically writes /tmp/manual_steer.json (temp + os.replace) so the hook
#     never reads a half-written file.
#   * Automates /tmp/MANUAL_STEER_ENABLE around the TCP connection lifecycle
#     (see ENABLE_FILE comment below) — no more manual touch/rm.
#
# WHAT THIS IS NOT
#   * It does NOT encode CAN, compute checksums, cycle counters, or open the
#     Panda. It carries ONE number (+ one flag) across the wire. All CAN/limit/
#     gating logic stays in the carcontroller + manual_steer_hook, unchanged.
#   * It does NOT gate on speed / lat_active — those gates live in the hook
#     (next to the actuation), on purpose. This writer is dumb about *content*:
#     it writes whatever fresh angle+active arrives. The hook decides whether
#     to USE it (including gate 2.5: active must be True).
#   * It cannot by itself move anything. With ENABLE_FILE absent (no session
#     connected) OR active=False (session connected but not overriding), the
#     hook no-ops regardless of what this writes.
#
# DEAD-MAN BEHAVIOUR (why stopping is safe)
#   The writer only writes when a fresh angle arrives from the sender. If the
#   sender stops (key released in hold mode still streams; but window close,
#   quit, or network drop stops the stream), NO new writes happen -> the file
#   goes stale within STALE_S -> the hook falls back to the stock planner angle.
#   Independently, disconnect/exit removes ENABLE_FILE -> instant revert even
#   before staleness kicks in. `rm /tmp/MANUAL_STEER_ENABLE` by hand still works
#   as a manual override-of-the-override, but should not be needed in normal use.
#
# RUN ON DEVICE (bukapilot can stay running — no Panda/USB conflict):
#   cd /data/openpilot
#   screen -S msteer
#   python3 manual_steer_writer.py            # listens on 0.0.0.0:5557
#   # Ctrl+A then D to detach — writer keeps running
#
# PROTOCOL (newline-delimited JSON, one object per line):
#   sender -> writer:  {"angle_deg": <float>, "active": <bool>}\n
#   The writer ignores any 'ts' the sender might send and stamps its own.
#   'active' defaults to False if absent (older sender / garbled field), so a
#   partial packet can never accidentally arm an override.
#   A line that fails to parse or lacks a numeric angle_deg is skipped (no write,
#   no crash) so a garbled packet cannot poison the target.
#
# No external deps beyond stdlib.

import argparse
import atexit
import json
import os
import signal
import socket
import time

TARGET_FILE = "/tmp/manual_steer.json"

# This file means ONLY "a session is live" (a sender is currently connected,
# or was until process exit). It does NOT mean "actively overriding" — that
# is decided per-frame by the "active" flag inside TARGET_FILE, toggled by
# the driver on the sender side (key M). The writer automates this file
# purely around the TCP connection lifecycle: cleared at startup, touched on
# connect, removed on disconnect and on process exit. Gate 2.5 in
# manual_steer_hook.py is what actually decides whether substitution happens.
ENABLE_FILE = "/tmp/MANUAL_STEER_ENABLE"

# Two-tier caps — second enforcement gate (sender clamped first; hook clamps third).
# NORMAL: 45 deg — 25 deg margin to Panda max_angle 70 deg (byd.h, confirmed).
# HIGH:   60 deg — 10 deg margin to Panda max_angle 70 deg.
#   HIGH only applies when the sender sends high_range=True with active=True.
#   high_range=True with active=False is rejected here and treated as NORMAL.
WRITER_ABS_MAX_DEG_NORMAL = 45.0
WRITER_ABS_MAX_DEG_HIGH   = 60.0
WRITER_ABS_MAX_DEG = WRITER_ABS_MAX_DEG_NORMAL   # default / --max-angle ceiling


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def _touch_enable(enable_file=ENABLE_FILE):
    """Best-effort create/refresh the enable ('session live') file."""
    try:
        open(enable_file, "a").close()
        os.utime(enable_file, None)
    except OSError as e:
        print(f"[writer] WARN could not touch {enable_file}: {e}")


def _remove_enable(enable_file=ENABLE_FILE):
    """Best-effort remove the enable ('session live') file.

    Returns True if a file was actually removed, False if none existed.
    """
    try:
        os.remove(enable_file)
        return True
    except FileNotFoundError:
        return False
    except OSError as e:
        print(f"[writer] WARN could not remove {enable_file}: {e}")
        return False


def atomic_write_target(path, angle_deg, active, high_range=False):
    """Atomically write {angle_deg, ts, active, high_range} with the DEVICE's own wall clock.

    temp + os.replace is atomic on POSIX, so the hook's json.load never sees a
    partially written file. ts is stamped here (device side) so the hook's
    freshness window is same-clock. `active` and `high_range` are passed through
    from the sender; high_range is never True when active=False (enforced here
    as belt-and-suspenders even if the caller already checked).
    """
    payload = {
        "angle_deg": round(float(angle_deg), 3),
        "ts": time.time(),          # DEVICE clock — matches the hook's time.time()
        "active": bool(active),
        "high_range": bool(active and high_range),  # never True when inactive
    }
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(payload, f)
    os.replace(tmp, path)


def handle_client(conn, addr, target_file, abs_max_deg, verbose):
    print(f"[writer] client connected: {addr[0]}:{addr[1]}")
    # Session-live marker: remove-then-touch so a stale leftover file (mtime,
    # any prior contents) can't be mistaken for this session.
    _remove_enable()
    _touch_enable()
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
                    active = bool(obj.get("active", False))  # older senders: default False
                    # high_range: fail-safe False if missing (older sender / garbled).
                    # Also enforce active guard here — high_range=True with active=False
                    # is rejected regardless of what the sender sent.
                    high_range = bool(obj.get("high_range", False)) and active
                except (ValueError, KeyError, TypeError, UnicodeDecodeError):
                    # Garbled or incomplete packet: skip it. Do NOT write, do NOT
                    # crash. A bad packet must never poison the target file.
                    continue
                cap = WRITER_ABS_MAX_DEG_HIGH if high_range else abs_max_deg
                angle = _clamp(angle, -cap, cap)
                try:
                    atomic_write_target(target_file, angle, active, high_range)
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
        # Session ended (clean or dropped, override was on or off — doesn't
        # matter): the dead-man is the enable file, not the target file. This
        # fires even if the link dropped mid-override, since this whole block
        # runs regardless of how the try above exited.
        _remove_enable()
        # Belt-and-suspenders: immediately write an inactive record so the
        # next session's first hook frame can never read a stale high_range=True
        # that was left by the previous session. The stale-ts guard in the hook
        # already handles this within STALE_S, but writing clean state now means
        # zero exposure even in the first 200 ms window after a reconnect.
        try:
            atomic_write_target(target_file, 0.0, False, False)
        except OSError:
            pass
        print(f"[writer] client disconnected ({addr[0]}:{addr[1]}); "
              f"wrote {n_written} targets. Enable file removed, target file "
              f"reset to inactive (high_range=False). Hook falls back to "
              f"stock within STALE_S regardless.")


def run(host, port, target_file, abs_max_deg, verbose):
    # Startup: clear any stale enable file left over from a prior crashed/
    # killed run — a fresh process start must never inherit a "session live"
    # marker from before.
    if _remove_enable():
        print(f"[writer] startup: cleared stale enable file {ENABLE_FILE}")
    else:
        print(f"[writer] startup: no stale enable file found ({ENABLE_FILE})")

    # Best-effort cleanup on process exit, including non-KeyboardInterrupt
    # signals (SIGTERM) and normal interpreter exit (atexit) — a backstop
    # behind the try/finally below, not a replacement for it.
    atexit.register(_remove_enable)

    def _on_signal(signum, frame):
        _remove_enable()
        raise SystemExit(0)

    signal.signal(signal.SIGTERM, _on_signal)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(1)                          # one sender at a time
    print(f"[writer] listening on {host}:{port} -> {target_file}")
    print(f"[writer] clamp +/-{abs_max_deg:.0f} deg. Enable file "
          f"{ENABLE_FILE} is now automated by this writer around the TCP "
          f"connection lifecycle (touched on connect, removed on disconnect/"
          f"exit) — it means ONLY 'a session is live', NOT 'actively "
          f"overriding'. Substitution additionally requires the sender's "
          f"override toggle (key M) to be ON.")
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
        _remove_enable()
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
