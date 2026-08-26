#!/usr/bin/env python3
# Kommu.AI — Manual Steering Sender (INC/laptop side, network edition)
#
# PURPOSE
#   Human-driven steering input for the BYD Dolphin manual-steering test. Turns
#   keyboard hold/release into a desired steering angle and STREAMS it over TCP
#   to manual_steer_writer.py running on the Kommu device, which lands it at
#   /tmp/manual_steer.json for the carcontroller hook to read.
#
#   This runs on INC (the laptop) because the Kommu device is headless — pygame
#   cannot get a real display or key events there. The device side is the dumb
#   writer; ALL CAN encoding, counter, checksum, rate-limiting, gating, and the
#   Panda safety model stay on the device (carcontroller + hook), unchanged.
#
#   This script does NOT touch CAN, does NOT import bukapilot, does NOT write the
#   target file, and cannot by itself move anything.
#
# SAFETY MODEL (read before running against a real car)
#   * The device writer stamps the timestamp; if THIS sender stops streaming
#     (window close, Q/ESC, or network drop), the device stops writing, the file
#     goes stale within ~200 ms, and the carcontroller falls back to the stock
#     planner angle. That is the dead-man.
#   * OVERRIDE toggle (key M) — separate from AUTO-RETURN, and the thing that
#     actually decides whether any of this reaches the wheel:
#       - OVERRIDE OFF (default, always, even while connected): bukapilot's own
#         planner angle is used untouched. Holding A/D still ramps the on-screen
#         TARGET ANGLE and it is still streamed to the device (with
#         "active": false), but the hook will NOT substitute it — a connected,
#         idle-or-even-actively-keyed session with OVERRIDE OFF is functionally
#         identical to no session at all.
#       - OVERRIDE ON: the streamed target angle actually substitutes for the
#         planner's desired angle (still subject to every downstream clamp/rate
#         limit in the hook + carcontroller).
#     Connecting to the device NEVER silently arms OVERRIDE — it always starts
#     OFF and must be explicitly toggled ON with M. This is on top of, not
#     instead of, the device's own enable-file automation (writer touches/
#     removes /tmp/MANUAL_STEER_ENABLE around the connection lifecycle; that
#     file now means only "a session is live", not "actively overriding").
#   * AUTO-RETURN toggle (key X), mirroring byd_level1_sim's behaviour:
#       - AUTO-RETURN ON  (default): release a steer key -> angle ramps to 0.
#       - AUTO-RETURN OFF (hold-value): release -> angle HOLDS where it is.
#         Use hold-value to sit at a steady angle for steer-ratio measurement.
#         ⚠️ In hold-value mode, releasing keys does NOT recentre. The only ways
#         back to centre are: press E (snap target to 0), quit, or toggling
#         OVERRIDE off (M). Everyone in the car must know which mode is active
#         before moving.
#   * Angle is hard-clamped to +/- MAX_ANGLE_DEG here, AND again by the device
#     writer, AND again by the hook, AND finally by _compute_apply_angle
#     (+/-10deg-of-measured + rate limit). Layered clamps.
#   * Per-frame step is capped so the Panda standstill rate limit (2.8 deg/frame)
#     is never the thing asking for too much; the device rate-limiter is the real
#     enforcement. Keep --rate <= 2.5.
#   * No steer-angle scale-factor compensation is applied or needed here. The
#     device-side encoder's deg->raw is confirmed 10.0 (not 10.2) on both
#     staging_v2 and cam_lka — do not add a compensating factor on this side.
#   * The connection stays OPEN across key releases (persistent stream). Only
#     quit / window-close / network drop stops it.
#
# CONTROLS
#   a / LEFT    hold to steer left  (negative angle)
#   d / RIGHT   hold to steer right (positive angle)
#   release     auto-return to centre  (only when AUTO-RETURN is ON)
#   m           toggle OVERRIDE on/off  (OFF = bukapilot has control; starts OFF)
#   x           toggle AUTO-RETURN on/off  (off = hold-value)
#   e           snap target to 0 immediately (still rate-limited on device)
#   q / ESC     quit (streams active=false once, then exits -> device file goes stale)
#
# USAGE (on INC / laptop):
#   python3 manual_steer_sender.py --host 192.168.1.100
#   python3 manual_steer_sender.py --host <device_ip> --port 5557 --max-angle 10 --rate 2.0
#
#   On a phone hotspot the device gets a new IP — just pass the new --host.
#   The DEVICE side must be running the writer:
#     python3 manual_steer_writer.py        # on device; automates the enable
#                                            # file around the connection
#                                            # lifecycle (no manual touch/rm)
#   Connecting alone does NOT arm anything — press M on the sender to toggle
#   OVERRIDE on; it always starts OFF.

import argparse
import json
import socket
import sys
import time

# ---- Defaults (overridable via CLI) -----------------------------------------
DEF_PORT             = 5557
HZ                   = 50.0     # write/stream rate; matches steering frame cadence
# Two-tier angle caps. Panda hard ceiling = 70 deg (byd.h max_angle=700 raw).
# NORMAL: 45 deg — 25 deg margin to 70 deg ceiling.
# HIGH:   60 deg — 10 deg margin to 70 deg ceiling. Deliberate intermediate
#         step — validate 45 before stepping HIGH toward the safety ceiling.
# HIGH is ONLY reachable via the H key toggle when OVERRIDE (M) is already ON.
# The CLI --max-angle arg is clamped to NORMAL (45 deg); HIGH is runtime-only
# and cannot be raised via any command-line flag.
ANGLE_CAP_NORMAL     = 45.0
ANGLE_CAP_HIGH       = 60.0
MAX_ANGLE_DEG        = ANGLE_CAP_NORMAL  # default for --max-angle arg
RATE_DEG_PER_FRAME   = 2.0      # ramp step per frame while held; < Panda 2.8/frame
RETURN_DEG_PER_FRAME = 2.0      # auto-return-to-centre step on release
HARD_MAX_ANGLE_DEG   = ANGLE_CAP_NORMAL  # CLI --max-angle ceiling; HIGH cap is internal


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class TargetLink:
    """Persistent TCP line-sender to the device writer, with auto-reconnect.

    Never raises on a transient send failure: if the link drops, sending is a
    no-op (the device sees no new lines -> file goes stale -> stock fallback),
    and we transparently try to reconnect on the next frame.
    """

    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = None
        self._last_connect_attempt = 0.0

    def _try_connect(self):
        now = time.monotonic()
        if now - self._last_connect_attempt < 0.5:   # throttle reconnect attempts
            return
        self._last_connect_attempt = now
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(1.0)
            s.connect((self.host, self.port))
            s.settimeout(None)
            self.sock = s
            print(f"[sender] connected to {self.host}:{self.port}")
        except OSError as e:
            self.sock = None
            print(f"[sender] connect failed ({e}); will retry")

    def send_angle(self, angle_deg, active, high_range=False):
        if self.sock is None:
            self._try_connect()
            if self.sock is None:
                return False
        line = (json.dumps({"angle_deg": round(float(angle_deg), 3),
                             "active": bool(active),
                             "high_range": bool(active and high_range),
                             }) + "\n").encode()
        try:
            self.sock.sendall(line)
            return True
        except OSError as e:
            print(f"[sender] send failed ({e}); dropping link, will reconnect")
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None
            return False

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None


def run(args):
    try:
        import pygame
    except ImportError:
        print("[error] pygame not installed. Run: pip install pygame")
        sys.exit(1)

    link = TargetLink(args.host, args.port)

    pygame.init()
    pygame.display.set_mode((820, 262))
    pygame.display.set_caption("Kommu.AI — BYD Dolphin Manual Steering Sender (network)")
    clock = pygame.time.Clock()

    try:
        font_hdr  = pygame.font.SysFont("monospace", 18, bold=True)
        font_main = pygame.font.SysFont("monospace", 20)
        font_sm   = pygame.font.SysFont("monospace", 15)
    except Exception:
        font_hdr = font_main = font_sm = pygame.font.Font(None, 22)

    COLOR_BG     = (20, 20, 30)
    COLOR_HEADER = (38, 38, 58)
    COLOR_WHITE  = (220, 220, 220)
    COLOR_LABEL  = (140, 140, 190)
    COLOR_GREEN  = (80, 220, 100)
    COLOR_YELLOW = (240, 200, 50)
    COLOR_RED    = (230, 70, 70)
    COLOR_KEY_ON = (240, 220, 60)
    COLOR_KEY_OFF= (70, 70, 90)

    target = 0.0
    auto_return = True            # X toggles this; ON = ramp to 0 on release
    override_active = False       # M toggles this; ALWAYS starts OFF — connecting
                                   # must never silently arm overriding.
    high_range_active = False     # H toggles this; only meaningful when M is ON.
                                   # M OFF auto-clears H. Always starts OFF.
    held = {"left": False, "right": False}
    exit_flag = False
    link_ok = False
    prev_link_ok = False          # reconnect detection: reset high_range on link recovery

    print(f"Manual steering sender -> {args.host}:{args.port}")
    print(f"max_angle={args.max_angle}  rate={args.rate}/frame  hz={args.hz}")
    print("Hold A/LEFT or D/RIGHT to steer; X toggles auto-return; E=zero; Q/ESC quit.")
    print("DEVICE must run manual_steer_writer.py AND have /tmp/MANUAL_STEER_ENABLE present.")

    try:
        while not exit_flag:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    exit_flag = True
                elif event.type == pygame.KEYDOWN:
                    if event.key in (pygame.K_q, pygame.K_ESCAPE):
                        exit_flag = True
                    elif event.key == pygame.K_e:
                        target = 0.0
                    elif event.key == pygame.K_x:
                        auto_return = not auto_return
                        print(f"[sender] AUTO-RETURN {'ON' if auto_return else 'OFF (hold-value)'}")
                    elif event.key == pygame.K_m:
                        override_active = not override_active
                        if not override_active:
                            high_range_active = False   # M OFF auto-clears H
                        print(f"[sender] OVERRIDE {'ON (commanding target)' if override_active else 'OFF (bukapilot has control)'}")
                    elif event.key == pygame.K_h:
                        if not override_active:
                            print("[sender] H has no effect — enable override first (press M)")
                        else:
                            high_range_active = not high_range_active
                            print(f"[sender] HIGH RANGE {'ON (±60°)' if high_range_active else 'OFF — back to NORMAL (±45°)'}")
                    elif event.key in (pygame.K_a, pygame.K_LEFT):
                        held["left"] = True
                    elif event.key in (pygame.K_d, pygame.K_RIGHT):
                        held["right"] = True
                elif event.type == pygame.KEYUP:
                    if event.key in (pygame.K_a, pygame.K_LEFT):
                        held["left"] = False
                    elif event.key in (pygame.K_d, pygame.K_RIGHT):
                        held["right"] = False

            # ---- Update target angle ------------------------------------------
            if held["left"] and not held["right"]:
                target -= args.rate
            elif held["right"] and not held["left"]:
                target += args.rate
            else:
                # Nothing (or both) held. Auto-return only when enabled.
                if auto_return:
                    if target > 0:
                        target = max(0.0, target - args.return_rate)
                    elif target < 0:
                        target = min(0.0, target + args.return_rate)
                # else: hold-value — leave target untouched.

            _active_cap = ANGLE_CAP_HIGH if (override_active and high_range_active) else args.max_angle
            target = clamp(target, -_active_cap, _active_cap)

            # Stream the target (persistent link; no-op if disconnected).
            link_ok = link.send_angle(target, override_active, override_active and high_range_active)

            # On reconnect: reset high_range to OFF so a fresh link never
            # inherits HIGH state from before the drop.
            if link_ok and not prev_link_ok:
                high_range_active = False
                print("[sender] reconnected — HIGH RANGE reset to OFF")
            prev_link_ok = link_ok

            # ---- Draw ----------------------------------------------------------
            surf = pygame.display.get_surface()
            surf.fill(COLOR_BG)
            pygame.draw.rect(surf, COLOR_HEADER, (0, 0, 820, 28))
            surf.blit(font_hdr.render("Kommu.AI — BYD Dolphin  |  Manual Steering Sender (network)",
                                      True, COLOR_WHITE), (10, 5))

            _display_cap = ANGLE_CAP_HIGH if (override_active and high_range_active) else args.max_angle
            mag = abs(target)
            tcol = COLOR_GREEN if mag < _display_cap * 0.5 else (
                   COLOR_YELLOW if mag < _display_cap * 0.9 else COLOR_RED)
            surf.blit(font_main.render(f"TARGET ANGLE : {target:+7.2f} deg", True, tcol), (12, 46))

            # OVERRIDE + range tier — the two most safety-critical lines on screen.
            if not override_active:
                override_txt = f"OVERRIDE: OFF (bukapilot has control)"
                override_col = COLOR_YELLOW
            elif not high_range_active:
                override_txt = f"OVERRIDE: ON — NORMAL  (±{ANGLE_CAP_NORMAL:.0f}°)"
                override_col = COLOR_GREEN
            else:
                override_txt = f"OVERRIDE: ON — HIGH RANGE (±{ANGLE_CAP_HIGH:.0f}°)  !!!"
                override_col = COLOR_RED
            surf.blit(font_hdr.render(override_txt, True, override_col), (12, 70))

            # H range sub-line — always rendered to keep layout stable.
            if override_active:
                hr_txt = (f"  H: HIGH RANGE {'ON  (±30°) — 15° margin to Panda ceiling' if high_range_active else 'off  (press H to widen to ±30°)'}")
                hr_col = COLOR_RED if high_range_active else COLOR_LABEL
            else:
                hr_txt = "  H: inactive — enable OVERRIDE first (press M)"
                hr_col = (55, 55, 75)
            surf.blit(font_sm.render(hr_txt, True, hr_col), (12, 94))

            # link + mode status lines (shifted down 22 px from original to clear H sub-line)
            link_txt = f"link {args.host}:{args.port} " + ("OK" if link_ok else "DOWN")
            link_col = COLOR_GREEN if link_ok else COLOR_RED
            surf.blit(font_sm.render(link_txt, True, link_col), (12, 116))
            mode_txt = "AUTO-RETURN: ON (release -> centre)" if auto_return else \
                       "AUTO-RETURN: OFF (hold-value; E/quit/M-off to centre)"
            mode_col = COLOR_GREEN if auto_return else COLOR_YELLOW
            surf.blit(font_sm.render(mode_txt, True, mode_col), (12, 136))

            surf.blit(font_sm.render(
                f"clamp +/-{_display_cap:.0f} deg   step {args.rate:.2f}/frame @ {args.hz:.0f} Hz",
                True, COLOR_LABEL), (12, 156))

            kx = 12
            for label, on in (("[A/<-]", held["left"]), ("[D/->]", held["right"])):
                col = COLOR_KEY_ON if on else COLOR_KEY_OFF
                ks = font_sm.render(label, True, col)
                surf.blit(ks, (kx, 178))
                kx += ks.get_width() + 10

            surf.blit(font_sm.render(
                "hold A/LEFT  D/RIGHT   M=override   H=high range (M must be ON)   X=auto-return   E=zero   Q/ESC=quit",
                True, (90, 90, 120)), (12, 202))
            surf.blit(font_sm.render(
                "device:  python3 manual_steer_writer.py   (enable file is now automated; M here decides override)",
                True, COLOR_LABEL), (12, 220))

            pygame.display.flip()
            clock.tick(args.hz)

    except KeyboardInterrupt:
        pass
    finally:
        # On exit: stream centre with override explicitly OFF so the hook
        # immediately stops substituting (not just waiting for staleness),
        # then drop the link — the writer's disconnect handler also removes
        # the device enable file at that point (session ended).
        link.send_angle(0.0, False, False)
        time.sleep(0.05)
        link.close()
        pygame.quit()
        print("Exited. Streamed target=0 / override=OFF and closed link. "
              "Device writer removes /tmp/MANUAL_STEER_ENABLE on disconnect.")


def parse_args():
    p = argparse.ArgumentParser(description="Manual steering sender (network, hold-to-steer)")
    p.add_argument("--host", required=True, help="device IP running manual_steer_writer.py")
    p.add_argument("--port", type=int, default=DEF_PORT, help=f"device writer port (default {DEF_PORT})")
    p.add_argument("--hz", type=float, default=HZ, help="stream rate Hz (default 50)")
    p.add_argument("--max-angle", dest="max_angle", type=float, default=MAX_ANGLE_DEG,
                   help="hard clamp on commanded angle, deg (default 20; equals the "
                        "hard refusal ceiling — see HARD_MAX_ANGLE_DEG)")
    p.add_argument("--rate", type=float, default=RATE_DEG_PER_FRAME,
                   help="ramp step per frame while held, deg (default 2.0; keep < 2.8)")
    p.add_argument("--return-rate", dest="return_rate", type=float, default=RETURN_DEG_PER_FRAME,
                   help="auto-return step per frame on release, deg (default 2.0)")
    return p.parse_args()


def main():
    args = parse_args()
    # Guard: never allow a per-frame step that could trip the Panda 2.8 deg/frame
    # standstill rate limit.
    if args.rate > 2.5:
        print(f"[warn] --rate {args.rate} exceeds safe 2.5/frame; clamping to 2.5")
        args.rate = 2.5
    if args.return_rate > 2.5:
        args.return_rate = 2.5
    # Hard refusal, not a warning: Panda's real hard ceiling is 70 deg
    # (byd.h max_angle=700 raw). All three layers (sender/writer/hook) now
    # agree at 45 deg (NORMAL) — this refusal ceiling is the last line of
    # defence, not one of several progressively-tighter clamps. args.max_angle
    # == 45.0 (the new default) passes this check (strict >), since 45.0 is not
    # greater than HARD_MAX_ANGLE_DEG (45.0); only values ABOVE 45 are refused.
    if args.max_angle > HARD_MAX_ANGLE_DEG:
        sys.exit(f"[refused] --max-angle {args.max_angle} exceeds the hard "
                 f"limit of {HARD_MAX_ANGLE_DEG} deg. Panda's real hard "
                 f"ceiling is 45 deg (byd.h max_angle=450 raw); this sender "
                 f"is the outermost of three layered clamps and must stay "
                 f"well under it. Re-verify against byd.h directly before "
                 f"raising this limit.")
    run(args)


if __name__ == "__main__":
    main()
