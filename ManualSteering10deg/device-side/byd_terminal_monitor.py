#!/usr/bin/env python3
"""
BYD Dolphin Terminal Monitor — reads from bukapilot cereal ZMQ stream.
Plain-terminal (curses) equivalent of byd_live_monitor.py, for headless SSH
sessions with no display. Works ALONGSIDE running bukapilot (no Panda USB
conflict) — this is a pure cereal.messaging.SubMaster subscriber, no TX,
no Panda access.

Run directly on device:
    python3 /data/openpilot/byd_terminal_monitor.py

Requires: only cereal (already on device). No pygame, no panda package.
Quit with 'q' or Ctrl-C.
"""

import curses
import os
import sys
import threading
import time
from collections import deque

OPENPILOT_PATH = os.environ.get("OPENPILOT_PATH", "/data/openpilot")
if OPENPILOT_PATH not in sys.path:
    sys.path.insert(0, OPENPILOT_PATH)

try:
    import cereal.messaging as messaging
    HAS_CEREAL = True
except ImportError:
    HAS_CEREAL = False

# ── CAN IDs ───────────────────────────────────────────────────────────────────
ID_STEER_READ = 0x11F
ID_STEER_CMD  = 0x1E2
ID_ACC_CMD    = 0x32E
ID_PCM_BTN    = 0x3B0


def byd_checksum(address, sig, d):
    """Real BYD nibble-sum checksum (byte_key=0xAF), verified against
    CONTEXT.md's known-good test frames. Ported from
    opendbc/car/byd/cam_lka/bydcan.py::byd_checksum (ignores address/sig,
    only uses the byte array itself)."""
    del address, sig
    byte_key = 0xAF
    sum_first = 0
    sum_second = 0
    for i in range(len(d) - 1):
        sum_first += d[i] >> 4
        sum_second += d[i] & 0xF
    remainder = (sum_second >> 4) & 0xFF
    sum_first += byte_key & 0xF
    sum_second += byte_key >> 4
    inv_first = (-sum_first + 0x9) & 0xF
    inv_second = (-sum_second + 0x9) & 0xF
    return (((inv_first + (5 - remainder)) & 0xF) << 4 | inv_second) & 0xFF


GEAR_LABELS = {
    "park": "P", "drive": "D", "neutral": "N", "reverse": "R",
    "sport": "S", "low": "L", "brake": "B", "eco": "E",
    "manumatic": "M", "unknown": "?",
}


# Derived button-combo mapping observed 2026-07-08/09 (developer testing
# SET/RES/ACC_ON/ICC while pressing physical controls). Not yet verified
# against DBC signal definitions or multiple sessions — treat as a working
# hypothesis, flag anything that doesn't match one of the known patterns.
#
# Two independent engage paths are known:
#   - ACC_ON (data[2] bit3) — the original SET/RES cluster button. This bit
#     is also read by byd.h's acc_pressed/cancel_pressed collision check, so
#     an ACC_ON-triggered engage CAN trip that bug when unpatched.
#   - ICC (data[0] bit6) — the separate steering-wheel-icon button. Confirmed
#     2026-07-09 (route 2026-07-09--03-36-48, t~511.35s) to engage cruise
#     without touching data[2] bit3 at all, sidestepping the collision bug
#     entirely for that engagement path.
def classify_pcm_buttons(set_btn, res_btn, acc_btn, icc_btn):
    if acc_btn and not set_btn and not res_btn and not icc_btn:
        return "ENGAGE (ACC_ON)", False
    if icc_btn and not set_btn and not res_btn and not acc_btn:
        return "ENGAGE (ICC)", False
    if set_btn and res_btn and not acc_btn and not icc_btn:
        return "ACC+", False
    if res_btn and not set_btn and not acc_btn and not icc_btn:
        return "ACC-", False
    if not set_btn and not res_btn and not acc_btn and not icc_btn:
        return "", False
    return (f"UNKNOWN: SET={int(set_btn)} RES={int(res_btn)} "
            f"ACC={int(acc_btn)} ICC={int(icc_btn)}", True)


def calc_rate(times):
    now = time.monotonic()
    recent = [t for t in times if t > now - 2.0]
    if len(recent) < 2:
        return 0.0
    span = recent[-1] - recent[0]
    return (len(recent) - 1) / span if span > 0.01 else 0.0


# ── Shared state ──────────────────────────────────────────────────────────────
class S:
    def __init__(self):
        self.lock         = threading.Lock()
        self.connected    = False
        self.error        = None
        self.start_t      = time.monotonic()
        # carState
        self.v_kmh        = 0.0
        self.steer_deg    = 0.0
        self.gear         = "unknown"
        self.cruise_on    = False
        self.cruise_avail = False
        self.brake        = False
        self.gas          = False
        # carControl
        self.lat_active   = False
        self.long_active  = False
        # selfdriveState
        self.op_enabled   = False
        self.op_state     = ""
        self.alert1       = ""
        self.alert_status = ""
        # onroadEvents
        self.last_event   = ""
        self.engage_count   = 0
        self.mismatch_count = 0
        # CAN
        self.can          = {}
        self.set_btn      = False
        self.res_btn      = False
        self.acc_btn      = False
        self.icc_btn      = False


# ── Cereal thread ─────────────────────────────────────────────────────────────
class CerealThread(threading.Thread):
    def __init__(self, state):
        super().__init__(daemon=True)
        self.state = state

    def run(self):
        if not HAS_CEREAL:
            self.state.error = (
                "cereal not found.\n"
                "Run directly on device:\n"
                "  python3 /data/openpilot/byd_terminal_monitor.py"
            )
            return

        try:
            sm = messaging.SubMaster(
                ['carState', 'carControl', 'selfdriveState', 'onroadEvents', 'can', 'sendcan'],
            )
        except Exception as e:
            self.state.error = f"SubMaster failed:\n{e}\n\nIs bukapilot running?"
            return

        self.state.connected = True

        while True:
            try:
                sm.update(timeout=100)
            except Exception:
                time.sleep(0.05)
                continue

            now = time.monotonic()
            st  = self.state

            with st.lock:
                # carState — speed, steering, gear, cruise, pedals
                if sm.updated['carState']:
                    cs = sm['carState']
                    st.v_kmh        = cs.vEgo * 3.6
                    st.steer_deg    = cs.steeringAngleDeg
                    gear_raw        = str(cs.gearShifter)
                    st.gear         = GEAR_LABELS.get(gear_raw, gear_raw)
                    st.cruise_on    = bool(cs.cruiseState.enabled)
                    st.cruise_avail = bool(cs.cruiseState.available)
                    st.brake        = bool(cs.brakePressed)
                    st.gas          = bool(cs.gasPressed)

                # carControl — latActive / longActive
                if sm.updated['carControl']:
                    cc = sm['carControl']
                    st.lat_active  = bool(cc.latActive)
                    st.long_active = bool(cc.longActive)

                # selfdriveState — enabled, state, alerts
                if sm.updated['selfdriveState']:
                    ss = sm['selfdriveState']
                    st.op_enabled = bool(ss.enabled)
                    try:
                        st.op_state     = str(ss.state)
                        st.alert1       = str(ss.alertText1)
                        st.alert_status = str(ss.alertStatus)
                    except Exception:
                        pass

                # onroadEvents — a list directly on the Event message (NOT
                # `.events` — that attribute doesn't exist; the old pygame
                # monitor's `sm['onroadEvents'].events` access silently threw
                # and was swallowed by a bare except, so its engage/mismatch
                # counters never actually fired).
                if sm.updated['onroadEvents']:
                    try:
                        names = [str(ev.name) for ev in sm['onroadEvents']]
                    except Exception:
                        names = []
                    for n in names:
                        ln = n.lower()
                        if 'pcmenable' in ln:
                            st.engage_count += 1
                            st.last_event = f"ENGAGE +{now - st.start_t:.1f}s"
                        elif 'mismatch' in ln:
                            st.mismatch_count += 1
                            st.last_event = f"MISMATCH +{now - st.start_t:.1f}s"
                        else:
                            st.last_event = f"{n} +{now - st.start_t:.1f}s"

                # CAN frames
                if sm.updated['can']:
                    for pkt in sm['can']:
                        if pkt.src != 0:
                            continue
                        addr = pkt.address
                        dat  = bytes(pkt.dat)
                        if addr not in st.can:
                            st.can[addr] = {
                                'data': b'', 'last_seen': 0.0,
                                'times': deque(maxlen=60),
                                'checksum_ok': True,
                            }
                        f = st.can[addr]
                        f['data']      = dat
                        f['last_seen'] = now
                        f['times'].append(now)

                        if addr == ID_PCM_BTN and len(dat) >= 3:
                            st.set_btn = bool((dat[0] >> 4) & 1)
                            st.res_btn = bool((dat[0] >> 3) & 1)
                            st.acc_btn = bool((dat[2] >> 3) & 1)
                            # ICC = data[0] bit6, byd.h's icc_pressed. This is the
                            # steering-wheel-icon button (separate from the SET/RES/
                            # ACC_ON cluster) — confirmed 2026-07-09 to engage cruise
                            # via a path that does not touch data[2] bit3 (the
                            # ACC_ON/cancel-collision bit), so it did not trigger the
                            # known cancel_pressed/acc_pressed collision bug that day.
                            st.icc_btn = bool((dat[0] >> 6) & 1)

                        if addr in (ID_STEER_CMD, ID_ACC_CMD) and len(dat) >= 8:
                            f['checksum_ok'] = (dat[7] == byd_checksum(addr, None, dat))

                # sendcan frames — bukapilot's TX output (0x1E2, 0x316, etc.).
                # These are NOT on the `can` topic; card.py publishes them here.
                # No src filter: pkt.src is the target bus, not a bus-origin tag.
                if sm.updated['sendcan']:
                    for pkt in sm['sendcan']:
                        addr = pkt.address
                        dat  = bytes(pkt.dat)
                        if addr not in st.can:
                            st.can[addr] = {
                                'data': b'', 'last_seen': 0.0,
                                'times': deque(maxlen=60),
                                'checksum_ok': True,
                            }
                        f = st.can[addr]
                        f['data']      = dat
                        f['last_seen'] = now
                        f['times'].append(now)

                        if addr in (ID_STEER_CMD, ID_ACC_CMD) and len(dat) >= 8:
                            f['checksum_ok'] = (dat[7] == byd_checksum(addr, None, dat))


# ── Renderer ──────────────────────────────────────────────────────────────────
def snap(st):
    with st.lock:
        d = {k: v for k, v in st.__dict__.items() if k != 'lock'}
        d['can'] = {k: dict(v) for k, v in st.can.items()}
    return d


def safe_addstr(stdscr, y, x, text, attr=0):
    max_y, max_x = stdscr.getmaxyx()
    if 0 <= y < max_y and x < max_x:
        try:
            stdscr.addstr(y, x, text[:max(0, max_x - x - 1)], attr)
        except curses.error:
            pass


def draw_error(stdscr, msg):
    stdscr.erase()
    safe_addstr(stdscr, 0, 0, "Kommu.AI — BYD Dolphin Terminal Monitor", curses.A_BOLD)
    safe_addstr(stdscr, 2, 2, "Cannot connect to bukapilot", curses.color_pair(3))
    for i, line in enumerate(msg.split("\n")):
        safe_addstr(stdscr, 4 + i, 2, line)
    stdscr.refresh()


def draw(stdscr, s):
    stdscr.erase()

    green   = curses.color_pair(1) | curses.A_BOLD
    yellow  = curses.color_pair(2) | curses.A_BOLD
    red     = curses.color_pair(3) | curses.A_BOLD
    cyan    = curses.color_pair(4) | curses.A_BOLD
    magenta = curses.color_pair(5) | curses.A_BOLD
    dim     = curses.A_DIM

    elapsed = time.monotonic() - s['start_t']
    hh, r = divmod(int(elapsed), 3600)
    mm, ss = divmod(r, 60)
    conn_col = green if s['connected'] else red
    safe_addstr(stdscr, 0, 0, "Kommu.AI — BYD Dolphin — Terminal Monitor", curses.A_BOLD)
    safe_addstr(stdscr, 0, 50, f"[{'LIVE' if s['connected'] else 'OFFLINE'} {hh:02d}:{mm:02d}:{ss:02d}]", conn_col)
    safe_addstr(stdscr, 1, 0, "-" * 78, dim)

    # Speed / steering / gear
    safe_addstr(stdscr, 3, 0, f"{s['v_kmh']:5.1f} km/h", cyan if s['v_kmh'] > 1 else dim)
    safe_addstr(stdscr, 3, 18, f"steer {s['steer_deg']:+6.1f} deg", yellow if abs(s['steer_deg']) > 5 else 0)
    safe_addstr(stdscr, 3, 42, f"gear: {s['gear']}", curses.A_BOLD)

    # Engagement — LAT / LONG / CRUISE always shown independently, not
    # collapsed into a single indicator.
    lat_txt   = "ACTIVE" if s['lat_active']  else "inactive"
    long_txt  = "ACTIVE" if s['long_active'] else "inactive"
    cruise_txt = "ENABLED" if s['cruise_on'] else "disabled"
    lat_col   = green if s['lat_active']  else dim
    long_col  = green if s['long_active'] else dim
    cruise_col = yellow if s['cruise_on'] else dim

    safe_addstr(stdscr, 5, 0,  f"LAT:    {lat_txt:8s}", lat_col)
    safe_addstr(stdscr, 5, 20, f"LONG:   {long_txt:8s}", long_col)
    safe_addstr(stdscr, 5, 40, f"CRUISE: {cruise_txt:8s}", cruise_col)

    safe_addstr(stdscr, 6, 0,  f"cruise.available={s['cruise_avail']}", 0)
    safe_addstr(stdscr, 6, 30, f"op_enabled={s['op_enabled']} state={s['op_state']}", 0)

    # Pedals
    safe_addstr(stdscr, 8, 0, "BRAKE", red if s['brake'] else dim)
    safe_addstr(stdscr, 8, 10, "GAS", green if s['gas'] else dim)

    # Alert
    if s['alert1']:
        ac = red if 'critical' in s['alert_status'].lower() else yellow
        safe_addstr(stdscr, 10, 0, f"ALERT: {s['alert1']}", ac)

    # PCM buttons — raw ground-truth CAN bit state, unchanged.
    row = 12
    safe_addstr(stdscr, row, 0, "PCM 0x3B0:", dim)
    items = [("SET", s['set_btn']), ("RES", s['res_btn']), ("ACC_ON", s['acc_btn']), ("ICC", s['icc_btn'])]
    x = 12
    for label, active in items:
        safe_addstr(stdscr, row, x, label, green if active else dim)
        x += len(label) + 2

    # Derived indicator (see classify_pcm_buttons docstring/comment above).
    derived_label, is_unknown = classify_pcm_buttons(s['set_btn'], s['res_btn'], s['acc_btn'], s['icc_btn'])
    if derived_label:
        derived_col = magenta if is_unknown else green
        safe_addstr(stdscr, row, x + 2, f"[{derived_label}]", derived_col)

    # Event / counters
    row += 2
    mc = red if s['mismatch_count'] > 0 else dim
    safe_addstr(stdscr, row, 0, f"engages={s['engage_count']}  mismatches={s['mismatch_count']}", mc)
    if s['last_event']:
        lc = red if 'MISMATCH' in s['last_event'] else 0
        safe_addstr(stdscr, row + 1, 0, f"last event: {s['last_event']}", lc)

    # CAN panel
    row += 3
    safe_addstr(stdscr, row, 0, "-" * 78, dim)
    row += 1
    safe_addstr(stdscr, row, 0, "CAN (bus 0):", curses.A_BOLD)
    row += 1
    now = time.monotonic()
    ids = [
        (ID_STEER_READ, "0x11F STEER_MODULE_2"),
        (ID_STEER_CMD,  "0x1E2 STEER_CMD"),
        (ID_ACC_CMD,    "0x32E ACC_CMD"),
    ]
    for addr, title in ids:
        fr = s['can'].get(addr)
        if fr and fr['last_seen']:
            age  = now - fr['last_seen']
            rate = calc_rate(fr['times'])
            cok  = fr.get('checksum_ok', True)
            col  = yellow if age > 0.25 else (red if not cok else green)
            dat  = fr['data']
            hexs = " ".join(f"{b:02X}" for b in dat[:8])
            extra = ""
            if addr == ID_STEER_READ and len(dat) >= 3:
                extra = f"angle_raw={dat[2]}"
            elif addr == ID_STEER_CMD and len(dat) >= 5:
                rraw = dat[3] | (dat[4] << 8)
                if rraw & 0x8000:
                    rraw -= 0x10000
                extra = f"cmd={rraw * 0.1 / 1.02:+.1f}deg csum={'OK' if cok else 'BAD'}"
            elif addr == ID_ACC_CMD and len(dat) >= 1:
                extra = f"accel={(dat[0]-100)/16.67:+.2f}m/s2 csum={'OK' if cok else 'BAD'}"
            safe_addstr(stdscr, row, 0, f"{title:24s} {hexs:24s} {extra:30s} {rate:3.0f}Hz age={age*1000:.0f}ms", col)
        else:
            safe_addstr(stdscr, row, 0, f"{title:24s} (no data)", dim)
        row += 1

    row += 1
    safe_addstr(stdscr, row, 0, "q / Ctrl-C to quit", dim)

    stdscr.refresh()


def main(stdscr):
    curses.curs_set(0)
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_GREEN, -1)
    curses.init_pair(2, curses.COLOR_YELLOW, -1)
    curses.init_pair(3, curses.COLOR_RED, -1)
    curses.init_pair(4, curses.COLOR_CYAN, -1)
    curses.init_pair(5, curses.COLOR_MAGENTA, -1)
    stdscr.nodelay(True)
    stdscr.timeout(250)

    st = S()
    thread = CerealThread(st)
    thread.start()

    while True:
        try:
            key = stdscr.getch()
        except curses.error:
            key = -1
        if key in (ord('q'), ord('Q'), 27):
            break

        if st.error:
            draw_error(stdscr, st.error)
        else:
            s = snap(st)
            draw(stdscr, s)


if __name__ == "__main__":
    try:
        curses.wrapper(main)
    except KeyboardInterrupt:
        pass
