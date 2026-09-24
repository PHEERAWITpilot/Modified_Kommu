#!/usr/bin/env python3
# Kommu.AI — Cereal-to-TCP Bridge (DEVICE SIDE, headless)
#
# WHY THIS EXISTS
#   The KommuAssist camera is headless — no display, so pygame monitors
#   (byd_live_monitor.py) cannot render on it. And byd_can_server.py reads the
#   Panda directly, which conflicts with bukapilot's exclusive USB access.
#
#   This server is the missing combination:
#     * reads the cereal STREAM (like byd_live_monitor.py)  -> runs ALONGSIDE
#       bukapilot, no Panda/USB conflict, bukapilot stays up
#     * broadcasts decoded state as newline-delimited JSON over TCP
#       (like byd_can_server.py)                            -> view on laptop
#
#   Pair with: byd_cereal_text_client.py  (run on laptop)
#
# RUN ON DEVICE (bukapilot running):
#   cd /data/openpilot
#   python3 byd_cereal_server.py            # listens on 0.0.0.0:5556
#
#   (use screen -S cerealsrv so it survives SSH disconnect)
#
# Client connects to <camera_ip>:5556 and receives one JSON line ~10x/sec.
# No external deps beyond cereal (already on device) + stdlib.

import argparse
import json
import socket
import threading
import time

import os, sys
OPENPILOT_PATH = os.environ.get("OPENPILOT_PATH", "/data/openpilot")
if OPENPILOT_PATH not in sys.path:
    sys.path.insert(0, OPENPILOT_PATH)

import cereal.messaging as messaging

# ── CAN IDs we decode for the test ───────────────────────────────────────────
ID_STEER_CMD = 0x1E2     # 482  STEERING_MODULE_ADAS  8 bytes
ID_ACC_CMD   = 0x32E     # 814  ACC_CMD               8 bytes
ID_PCM_BTN   = 0x3B0     # 944  PCM_BUTTONS


def byd_checksum(dat):
    # verified formula: (sum(payload) ^ 0xFF) & 0xFF  (payload = all but last byte)
    return (sum(dat[:-1]) ^ 0xFF) & 0xFF


# ── Shared state, written by cereal thread, read by broadcaster ──────────────
class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.d = {
            # carState
            "v_kmh": 0.0, "steer_deg": 0.0, "yaw_rate": 0.0,
            # steering-angle-sensor bias at true centre, as estimated by
            # bukapilot's own live parameter filter. Subtract it from
            # steer_deg before any bicycle-model use.
            "angle_offset_deg": 0.0,
            # The car's OWN physical yaw-rate sensor, decoded straight off
            # CAN 546 (YAW_SENSOR). This is a real angular-rate measurement --
            # no steer ratio, no centre offset, no bicycle model. Distinct from
            # "yaw_rate" above, which comes from cs.yawRate and is ALWAYS 0.0
            # because the BYD port never assigns it.
            "yaw_sensor_rate": 0.0, "yaw_sensor_offset": 0.0, "yaw_sensor_ok": False,
            # The car's OWN 2-axis accelerometer, decoded straight off CAN 547
            # (ACCEL_SENSOR), 12-bit, ~0.0271 m/s^2 per LSB. Raw and OFFSET are
            # exposed SEPARATELY and deliberately un-subtracted: the offset
            # channel's own behaviour is under investigation, exactly as
            # YAW_OFFSET's was. Consumers do their own subtraction.
            # NOTE: sign conventions for both axes are NOT yet established.
            # accel_ok is False until a valid frame is decoded.
            "accel_long": 0.0, "accel_long_offset": 0.0,
            "accel_lat": 0.0, "accel_lat_offset": 0.0, "accel_ok": False,
            # time.monotonic() (device clock) when the cereal thread last saw a
            # FRESH carState. If this freezes while "ts" keeps advancing, the
            # stall is on the device; if both freeze, the broadcast loop is blocked.
            "cereal_ts": 0.0,
            "gear": "n/a",
            "cruise_on": False, "cruise_avail": False,
            "brake": False, "gas": False,
            # carControl
            "lat_active": False, "long_active": False,
            # selfdriveState
            "op_enabled": False, "op_state": "",
            "alert1": "", "alert_status": "",
            # PCM 0x3B0 button bits (the focus of the Park-vs-D check)
            "set_btn": False, "res_btn": False, "acc_btn": False, "lkas_btn": False,
            "pcm_raw": "",
            # pandaStates — the REAL controls_allowed (Panda side, not car side)
            "controls_allowed": False, "safety_model": "", "safety_param": 0,
            "rx_checks_invalid": False,
            # 0x1E2 steer cmd
            "steer_cmd_deg": None, "steer_cmd_csum_ok": None, "steer_cmd_raw": "",
            # 0x32E acc cmd
            "acc_cmd": None, "acc_cmd_csum_ok": None, "acc_cmd_raw": "",
            # counters
            "engage_count": 0, "mismatch_count": 0, "last_event": "",
        }


class CerealThread(threading.Thread):
    def __init__(self, state):
        super().__init__(daemon=True)
        self.state = state

    def run(self):
        sm = messaging.SubMaster(
            ['carState', 'carControl', 'selfdriveState', 'can', 'onroadEvents',
             'pandaStates', 'liveParameters'])

        # --- YAW_SENSOR (CAN 546) decode -------------------------------------
        # Decoded through opendbc's own CANParser, never by hand and never via
        # cantools (this DBC has C-style comments cantools rejects). Its own raw
        # 'can' socket, because CANParser.update() wants the card.py-style list,
        # not SubMaster's parsed view. Entirely optional: if any of this fails the
        # server keeps serving every other field exactly as before.
        cp_yaw = None
        sub_can_raw = None
        try:
            from opendbc.can.parser import CANParser
            from openpilot.selfdrive.pandad import can_capnp_to_list
            cp_yaw = CANParser("byd_general_pt", [("YAW_SENSOR", 50)], 0)
            sub_can_raw = messaging.sub_sock("can", timeout=0)
            print("[server] YAW_SENSOR (CAN 546) decoding enabled")
        except Exception as e:
            print(f"[server] YAW_SENSOR decode unavailable ({e}) — yaw_sensor_* stay 0.0")

        # --- ACCEL_SENSOR (CAN 547) decode -----------------------------------
        # Its OWN parser, not appended to cp_yaw's signal list, so that a
        # problem with 547 cannot take the already-proven 546 path down with
        # it. Shares cp_yaw's raw 'can' socket: the socket is drained ONCE per
        # loop and the same message list is handed to both parsers, because a
        # second drain would return nothing.
        cp_accel = None
        try:
            if sub_can_raw is not None:
                cp_accel = CANParser("byd_general_pt", [("ACCEL_SENSOR", 50)], 0)
                print("[server] ACCEL_SENSOR (CAN 547) decoding enabled")
        except Exception as e:
            print(f"[server] ACCEL_SENSOR decode unavailable ({e}) — accel_* stay 0.0")

        while True:
            sm.update(timeout=100)
            if sub_can_raw is not None:
                # ONE drain, shared by both parsers. Each update is guarded
                # separately so neither sensor can break the other.
                try:
                    _can_msgs = can_capnp_to_list(
                        messaging.drain_sock_raw(sub_can_raw, wait_for_one=False))
                except Exception:
                    _can_msgs = None
                if _can_msgs is not None:
                    if cp_yaw is not None:
                        try:
                            cp_yaw.update(_can_msgs)
                        except Exception:
                            pass
                    if cp_accel is not None:
                        try:
                            cp_accel.update(_can_msgs)
                        except Exception:
                            pass
            d = self.state.d
            with self.state.lock:
                if cp_yaw is not None:
                    try:
                        d["yaw_sensor_rate"] = round(cp_yaw.vl["YAW_SENSOR"]["YAW_RATE"], 6)
                        d["yaw_sensor_offset"] = round(cp_yaw.vl["YAW_SENSOR"]["YAW_OFFSET"], 6)
                        d["yaw_sensor_ok"] = bool(cp_yaw.can_valid)
                    except Exception:
                        d["yaw_sensor_ok"] = False

                if cp_accel is not None:
                    try:
                        a = cp_accel.vl["ACCEL_SENSOR"]
                        d["accel_long"] = round(a["LONGITUDE"], 6)
                        d["accel_long_offset"] = round(a["LONGITUDE_OFFSET"], 6)
                        d["accel_lat"] = round(a["LATERAL"], 6)
                        d["accel_lat_offset"] = round(a["LATERAL_OFFSET"], 6)
                        d["accel_ok"] = bool(cp_accel.can_valid)
                    except Exception:
                        d["accel_ok"] = False

                if sm.updated['liveParameters']:
                    lp = sm['liveParameters']
                    d["angle_offset_deg"] = round(lp.angleOffsetAverageDeg, 3)

                if sm.updated['carState']:
                    d["cereal_ts"] = time.monotonic()
                    cs = sm['carState']
                    d["v_kmh"] = round(cs.vEgo * 3.6, 2)
                    d["steer_deg"] = round(cs.steeringAngleDeg, 2)
                    d["yaw_rate"] = round(cs.yawRate, 5)
                    try:
                        d["gear"] = str(cs.gearShifter)  # park/drive/reverse/neutral/unknown
                    except Exception:
                        d["gear"] = "n/a"
                    d["cruise_on"] = bool(cs.cruiseState.enabled)
                    d["cruise_avail"] = bool(cs.cruiseState.available)
                    d["brake"] = bool(cs.brakePressed)
                    d["gas"] = bool(cs.gasPressed)
                if sm.updated['carControl']:
                    cc = sm['carControl']
                    d["lat_active"] = bool(cc.latActive)
                    d["long_active"] = bool(cc.longActive)
                if sm.updated['selfdriveState']:
                    ss = sm['selfdriveState']
                    d["op_enabled"] = bool(ss.enabled)
                    try:
                        d["op_state"] = str(ss.state)
                        d["alert1"] = str(ss.alertText1)
                        d["alert_status"] = str(ss.alertStatus)
                    except Exception:
                        pass
                if sm.updated['onroadEvents']:
                    try:
                        names = [str(ev.name) for ev in sm['onroadEvents'].events]
                    except Exception:
                        names = []
                    for n in names:
                        ln = n.lower()
                        if 'pcmenable' in ln or ('enable' in ln and 'disable' not in ln):
                            d["engage_count"] += 1
                            d["last_event"] = f"ENGAGE:{n}"
                        if 'mismatch' in ln:
                            d["mismatch_count"] += 1
                            d["last_event"] = f"MISMATCH:{n}"
                if sm.updated['pandaStates']:
                    for ps in sm['pandaStates']:
                        if str(ps.safetyModel) == 'byd':
                            d["controls_allowed"] = bool(ps.controlsAllowed)
                            d["safety_model"] = str(ps.safetyModel)
                            try:
                                d["safety_param"] = int(ps.safetyParam)
                                d["rx_checks_invalid"] = bool(ps.safetyRxChecksInvalid)
                            except Exception:
                                pass
                            break
                if sm.updated['can']:
                    for pkt in sm['can']:
                        if pkt.src != 0:
                            continue
                        addr = pkt.address
                        dat = bytes(pkt.dat)
                        if addr == ID_PCM_BTN and len(dat) >= 3:
                            d["set_btn"]  = bool((dat[0] >> 4) & 1)
                            d["res_btn"]  = bool((dat[0] >> 3) & 1)
                            d["lkas_btn"] = bool((dat[0] >> 6) & 1)
                            d["acc_btn"]  = bool((dat[2] >> 3) & 1)
                            d["pcm_raw"]  = dat.hex()
                        elif addr == ID_STEER_CMD and len(dat) >= 8:
                            r = dat[3] | (dat[4] << 8)
                            if r & 0x8000:
                                r -= 0x10000
                            d["steer_cmd_deg"] = round(r * 0.1 / 1.02, 2)
                            d["steer_cmd_csum_ok"] = (dat[7] == byd_checksum(dat))
                            d["steer_cmd_raw"] = dat.hex()
                        elif addr == ID_ACC_CMD and len(dat) >= 8:
                            d["acc_cmd"] = round((dat[0] - 100) / 16.67, 2)
                            d["acc_cmd_csum_ok"] = (dat[7] == byd_checksum(dat))
                            d["acc_cmd_raw"] = dat.hex()


def run_server(host, port, state, hz):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(5)
    srv.setblocking(False)
    print(f"[server] listening on {host}:{port} — waiting for clients (bukapilot can stay running)")

    clients = []
    period = 1.0 / hz
    last = time.monotonic()
    while True:
        try:
            conn, addr = srv.accept()
            conn.setblocking(True)
            clients.append(conn)
            print(f"[server] client connected: {addr[0]}:{addr[1]}")
        except BlockingIOError:
            pass

        now = time.monotonic()
        if now - last >= period:
            with state.lock:
                payload = dict(state.d)
            # Stamped HERE, immediately before the send, so anything after this
            # instant is provably network- or client-side, not device-side.
            # (It used to reuse `now` from the top of the loop, before the dict
            # copy and the lock, which measured scheduling rather than send.)
            payload["ts"] = time.monotonic()
            line = (json.dumps(payload) + "\n").encode()
            dead = []
            for c in clients:
                try:
                    c.sendall(line)
                except Exception:
                    dead.append(c)
            for c in dead:
                try:
                    c.close()
                except Exception:
                    pass
                clients.remove(c)
                print("[server] client disconnected")
            last = now
        time.sleep(0.005)


def main():
    p = argparse.ArgumentParser(description="Cereal-to-TCP bridge (device side)")
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=5556)
    p.add_argument("--hz", type=float, default=10.0, help="broadcast rate")
    args = p.parse_args()

    state = State()
    CerealThread(state).start()
    try:
        run_server(args.host, args.port, state, args.hz)
    except KeyboardInterrupt:
        print("\n[server] stopped.")


if __name__ == "__main__":
    main()
