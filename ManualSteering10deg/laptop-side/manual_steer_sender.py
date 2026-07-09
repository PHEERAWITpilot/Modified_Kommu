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
#   * AUTO-RETURN toggle (key X), mirroring byd_level1_sim's behaviour:
#       - AUTO-RETURN ON  (default): release a steer key -> angle ramps to 0.
#       - AUTO-RETURN OFF (hold-value): release -> angle HOLDS where it is.
#         Use hold-value to sit at a steady angle for steer-ratio measurement.
#         ⚠️ In hold-value mode, releasing keys does NOT recentre. The only ways
#         back to centre are: press E (snap target to 0), quit, or the device
#         kill file (rm /tmp/MANUAL_STEER_ENABLE). Everyone in the car must know
#         which mode is active before moving.
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
#   x           toggle AUTO-RETURN on/off  (off = hold-value)
#   e           snap target to 0 immediately (still rate-limited on device)
#   q / ESC     quit (streams 0 once, then exits -> device file goes stale)
#
# USAGE (on INC / laptop):
#   python3 manual_steer_sender.py --host 192.168.1.100
#   python3 manual_steer_sender.py --host <device_ip> --port 5557 --max-angle 10 --rate 2.0
#
#   On a phone hotspot the device gets a new IP — just pass the new --host.
#   The DEVICE side must be running the writer AND have the enable file present:
#     python3 manual_steer_writer.py        # on device
#     touch /tmp/MANUAL_STEER_ENABLE        # on device, to ARM
#     rm    /tmp/MANUAL_STEER_ENABLE        # on device, instant kill -> stock

import argparse
import json
import socket
import sys
import time

# ---- Defaults (overridable via CLI) -----------------------------------------
DEF_PORT             = 5557
HZ                   = 50.0     # write/stream rate; matches steering frame cadence
# Panda's real hard ceiling is 45 deg (confirmed 2026-07-08/09, byd.h
# max_angle=450 raw). All three layered clamps in this pipeline — this
# sender's MAX_ANGLE_DEG (10 deg), the writer's WRITER_ABS_MAX_DEG (15 deg),
# and the hook's MANUAL_ABS_MAX_DEG (10 deg) — sit comfortably below that. Do
# not raise any of them close to 45 deg without re-verifying against byd.h
# directly.
MAX_ANGLE_DEG        = 10.0     # hard clamp on this side (device clamps again)
RATE_DEG_PER_FRAME   = 2.0      # ramp step per frame while held; < Panda 2.8/frame
RETURN_DEG_PER_FRAME = 2.0      # auto-return-to-centre step on release
HARD_MAX_ANGLE_DEG   = 20.0     # absolute ceiling for --max-angle; refuse above this


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

    def send_angle(self, angle_deg):
        if self.sock is None:
            self._try_connect()
            if self.sock is None:
                return False
        line = (json.dumps({"angle_deg": round(float(angle_deg), 3)}) + "\n").encode()
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
    pygame.display.set_mode((820, 220))
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
    held = {"left": False, "right": False}
    exit_flag = False
    link_ok = False

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

            target = clamp(target, -args.max_angle, args.max_angle)

            # Stream the target (persistent link; no-op if disconnected).
            link_ok = link.send_angle(target)

            # ---- Draw ----------------------------------------------------------
            surf = pygame.display.get_surface()
            surf.fill(COLOR_BG)
            pygame.draw.rect(surf, COLOR_HEADER, (0, 0, 820, 28))
            surf.blit(font_hdr.render("Kommu.AI — BYD Dolphin  |  Manual Steering Sender (network)",
                                      True, COLOR_WHITE), (10, 5))

            mag = abs(target)
            tcol = COLOR_GREEN if mag < args.max_angle * 0.5 else (
                   COLOR_YELLOW if mag < args.max_angle * 0.9 else COLOR_RED)
            surf.blit(font_main.render(f"TARGET ANGLE : {target:+7.2f} deg", True, tcol), (12, 46))

            # link + mode status line
            link_txt = f"link {args.host}:{args.port} " + ("OK" if link_ok else "DOWN")
            link_col = COLOR_GREEN if link_ok else COLOR_RED
            surf.blit(font_sm.render(link_txt, True, link_col), (12, 80))
            mode_txt = "AUTO-RETURN: ON (release -> centre)" if auto_return else \
                       "AUTO-RETURN: OFF (hold-value; E/quit/killfile to centre)"
            mode_col = COLOR_GREEN if auto_return else COLOR_YELLOW
            surf.blit(font_sm.render(mode_txt, True, mode_col), (12, 100))

            surf.blit(font_sm.render(
                f"clamp +/-{args.max_angle:.0f} deg   step {args.rate:.2f}/frame @ {args.hz:.0f} Hz",
                True, COLOR_LABEL), (12, 120))

            kx = 12
            for label, on in (("[A/<-]", held["left"]), ("[D/->]", held["right"])):
                col = COLOR_KEY_ON if on else COLOR_KEY_OFF
                ks = font_sm.render(label, True, col)
                surf.blit(ks, (kx, 144))
                kx += ks.get_width() + 10

            surf.blit(font_sm.render(
                "hold A/LEFT  D/RIGHT   X=toggle auto-return   E=zero   Q/ESC=quit",
                True, (90, 90, 120)), (12, 168))
            surf.blit(font_sm.render(
                "device:  python3 manual_steer_writer.py   |   arm: touch /tmp/MANUAL_STEER_ENABLE   kill: rm it",
                True, COLOR_LABEL), (12, 190))

            pygame.display.flip()
            clock.tick(args.hz)

    except KeyboardInterrupt:
        pass
    finally:
        # On exit: stream centre once so the wheel unwinds (if still engaged),
        # then drop the link. The device file then goes stale -> stock fallback.
        link.send_angle(0.0)
        time.sleep(0.05)
        link.close()
        pygame.quit()
        print("Exited. Streamed target=0 and closed link. "
              "Remove /tmp/MANUAL_STEER_ENABLE on the device to be safe.")


def parse_args():
    p = argparse.ArgumentParser(description="Manual steering sender (network, hold-to-steer)")
    p.add_argument("--host", required=True, help="device IP running manual_steer_writer.py")
    p.add_argument("--port", type=int, default=DEF_PORT, help=f"device writer port (default {DEF_PORT})")
    p.add_argument("--hz", type=float, default=HZ, help="stream rate Hz (default 50)")
    p.add_argument("--max-angle", dest="max_angle", type=float, default=MAX_ANGLE_DEG,
                   help="hard clamp on commanded angle, deg (default 10)")
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
    # Hard refusal, not a warning: Panda's real hard ceiling is 45 deg
    # (byd.h max_angle=450 raw). 20 deg leaves real headroom below that even
    # before the writer (15 deg) and hook (10 deg) clamps tighten it further.
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
