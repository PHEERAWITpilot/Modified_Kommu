#!/usr/bin/env python3
# Kommu.AI — end-to-end dataset recorder (DEVICE SIDE, read-only)
#
# Records, into ONE rosbag2 (mcap) bag per session, on the device's own clock:
#   /kommu/road_camera/compressed  sensor_msgs/CompressedImage  (JPEG, same as
#                                  byd_road_camera_rosbag_recorder.py)
#   /kommu/can/steer_module_2      CAN 287 STEER_MODULE_2  (STEER_ANGLE_2 = the
#                                  source of carState.steeringAngleDeg on cam_lka)
#   /kommu/can/wheel_speed         CAN 496 WHEEL_SPEED  (4 wheels, raw km/h)
#   /kommu/can/yaw_sensor          CAN 546 YAW_SENSOR   (YAW_RATE, YAW_OFFSET)
# CAN topics use kommu_msgs/msg/CanFrameDecoded (definition embedded in the bag):
# every DBC signal of the frame, decoded by opendbc's CANParser exactly as
# byd_cereal_server.py does, plus the raw 8 payload bytes.
#
# WHAT THIS IS NOT
#   * Never subscribes to carState (its msgq has no free reader slot: a 16th
#     subscriber evicts every reader, controlsd included). Vehicle signals come
#     from the `can` socket only, and only after checking it has free slots.
#   * Never transmits on CAN, never writes params, never publishes anything.
#   * No ROS runtime, no DDS, no topics: rosbags only writes files, so there is
#     nothing that can overlap with odom_node.py / byd_drive.sh.
#   * Reads NO gear. Everything assumes forward (D) gear -- see README.txt that
#     is written into every session folder.
#   * Computes no positions and no labels. Raw signals only. How to compute
#     them offline (raw WHEEL_SPEED x wheelSpeedFactor into track_a_core.py,
#     no KF1D) is decided and written into every session's README.txt --
#     see README_TEXT below.
#
# CLOCK
#   Camera: VisionIPC timestamp_eof. CAN: logMonoTime of the pandad `can` batch
#   (all frames in one batch share it; batches arrive ~100 Hz). Both are
#   boot-relative; ONE epoch offset, taken once at start, is added to both, so
#   they stay on the same axis. BOOTTIME-MONOTONIC is logged at start and stop
#   (it was ~20 us when measured; a large value means the device suspended).
#
# STOPPING
#   SIGTERM or SIGINT -> bag closed cleanly, `.complete` written. Free space is
#   re-checked every --disk-check-s; below --stop-free-mb the session stops the
#   same way and leaves !!STOPPED_EARLY_LOW_DISK!!.txt at the top of the session
#   folder. SIGKILL / power loss leaves an mcap without metadata.yaml.
#
# RUN (normally via laptop-side byd_record_session.sh):
#   PYTHONPATH=/data/kommu_tools/pylibs:/data/kommu_tools:/data/openpilot \
#     /usr/local/venv/bin/python3 -u /data/kommu_tools/byd_e2e_recorder.py \
#     --session-dir /data/kommu_tools/e2e_sessions/<id>

import argparse
import fcntl
import hashlib
import json
import os
import shutil
import signal
import socket
import struct
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

OPENPILOT_PATH = os.environ.get("OPENPILOT_PATH", "/data/openpilot")
if OPENPILOT_PATH not in sys.path:
    sys.path.insert(0, OPENPILOT_PATH)

import numpy as np  # noqa: E402

import byd_road_camera_rosbag_recorder as camrec  # noqa: E402  camera convert/encode + clock offset
import cereal.messaging as messaging  # noqa: E402
from opendbc.can.parser import CANParser  # noqa: E402
from openpilot.selfdrive.pandad import can_capnp_to_list  # noqa: E402
from rosbags.rosbag2 import StoragePlugin, Writer  # noqa: E402
from rosbags.typesys import Stores, get_typestore, get_types_from_msg  # noqa: E402

DBC_NAME = "byd_general_pt"
CAN_BUS = 0
# (DBC message, address, frequency declared in cam_lka/carstate.py, topic)
CAN_MESSAGES = (
    ("STEER_MODULE_2", 287, 100, "/kommu/can/steer_module_2"),
    ("WHEEL_SPEED", 496, 50, "/kommu/can/wheel_speed"),
    ("YAW_SENSOR", 546, 50, "/kommu/can/yaw_sensor"),
)
CAN_MSGTYPE = "kommu_msgs/msg/CanFrameDecoded"
CAN_MSGDEF = """std_msgs/Header header
uint32 address
uint8 bus
bool can_valid
string[] signal_names
float64[] signal_values
uint8[] data
"""

LOW_DISK_MARKER = "!!STOPPED_EARLY_LOW_DISK!!.txt"
LOCK_PATH = "/tmp/byd_e2e_recorder.lock"
PID_PATH = "/tmp/byd_e2e_recorder.pid"
MSGQ_CAN_SHM = "/dev/shm/msgq_can"
MSGQ_NUM_READERS = 15          # msgq.h NUM_READERS; a 16th subscriber evicts all readers
DEFAULT_OUT_ROOT = "/data/kommu_tools/e2e_sessions"
CARPARAMS_PATH = "/data/params/d/CarParams"


def log(msg):
    print("[e2e] " + msg, flush=True)


def clock_gap_ns():
    return time.clock_gettime_ns(time.CLOCK_BOOTTIME) - time.clock_gettime_ns(time.CLOCK_MONOTONIC)


def msgq_num_readers(path):
    with open(path, "rb") as f:          # plain read of the header, no msgq API
        return struct.unpack("<Q", f.read(8))[0]


def free_mb(path):
    return shutil.disk_usage(path).free / 1e6


def file_md5(path):
    return hashlib.md5(Path(path).read_bytes()).hexdigest()


def read_car_params():
    """Fingerprint and wheelSpeedFactor from the params FILE (no subscription).
    WHEEL_SPEED is raw km/h; vEgoRaw = mean(4 wheels) / 3.6 * wheelSpeedFactor."""
    try:
        from cereal import car
        with car.CarParams.from_bytes(open(CARPARAMS_PATH, "rb").read()) as cp:
            return {"carFingerprint": str(cp.carFingerprint),
                    "wheelSpeedFactor": float(cp.wheelSpeedFactor)}
    except Exception as e:  # recorded, never fatal
        return {"error": repr(e)}


README_TEXT = """END-TO-END DATASET SESSION -- read before using this data

bag/        rosbag2 (mcap). Topics:
  /kommu/road_camera/compressed  sensor_msgs/msg/CompressedImage, JPEG
  /kommu/can/steer_module_2      CAN 287 STEER_MODULE_2, signal STEER_ANGLE_2 (deg)
  /kommu/can/wheel_speed         CAN 496 WHEEL_SPEED, WHEELSPEED_FL/FR/BL/BR (km/h, RAW)
  /kommu/can/yaw_sensor          CAN 546 YAW_SENSOR, YAW_RATE and YAW_OFFSET (rad/s)
  CAN topics are kommu_msgs/msg/CanFrameDecoded; the definition is embedded in
  the bag. With rosbags, register it before deserializing:
    typestore.register(get_types_from_msg(conn.msgdef.data, conn.msgtype))

ASSUMPTIONS AND CAVEATS
  * GEAR IS NOT RECORDED. Every sample is assumed to be FORWARD (D) gear.
    Any reverse in this session will look like forward motion.
  * Wheel speeds are RAW DBC values and are NOT true speed on their own.
    wheelSpeedFactor for this car is in session.json (0.6336 when measured).
  * Time: header.stamp is on the device clock (boot time + one epoch offset
    fixed at start) for camera and CAN alike. All CAN frames in one pandad
    batch share its timestamp (batches ~100 Hz).
  * No positions or labels are in this data.

COMPUTING POSITION / HEADING OFFLINE -- the accepted approach
  Use byd_odom_ros/track_a_core.py (stdlib only; the live odom_node imports
  the same file, so the math cannot drift):
    speed   v   = mean(WHEELSPEED_FL, _FR, _BL, _BR) / 3.6 * wheelSpeedFactor
                  then speed_gate(v)
    yaw     r   = track_a_yaw_rate(YAW_RATE, YAW_OFFSET, can_valid, v)
                  (sanitize_can_sample() is the only input check -- use it,
                  do not write your own)
    dt          = advance_clock(last_t, header.stamp in seconds)
    pose        Integrator.step(v, r, dt)   -- forward gear assumed
  PAIRING (both arrive at 50 Hz): step once per WHEEL_SPEED frame, timed by
  its header.stamp. Take the YAW_SENSOR frame with the EXACT same stamp (same
  pandad batch) when there is one; otherwise the most recent YAW_SENSOR frame
  stamped at or before it. Never use a yaw frame from the future. If no yaw
  frame exists yet at that time, skip that wheel-speed frame.
  DECIDED: raw WHEEL_SPEED x wheelSpeedFactor is used as-is. openpilot's
  vEgo additionally runs it through a Kalman filter (KF1D, tuned for a
  100 Hz loop, fed each 50 Hz wheel frame twice, state carried across
  calls). That filter is deliberately NOT reproduced: it cannot be made
  bit-identical from recorded frames (it depends on card's batch timing),
  and it only smooths speed. Positions from this data are therefore close
  to, but not bit-identical with, the live node's Track A.

session.json holds the settings, car params, clock checks and message counts.
!!STOPPED_EARLY_LOW_DISK!!.txt, if present, means the session stopped early.
.complete is written only after the bag was closed cleanly.
"""


class Session:
    """The bag plus everything written next to it. Thread-safe writes."""

    def __init__(self, session_dir: Path, typestore):
        self.dir = session_dir
        self.bag_path = session_dir / "bag"
        self.typestore = typestore
        self.lock = threading.Lock()
        self.writer = Writer(self.bag_path, version=8, storage_plugin=StoragePlugin.MCAP)
        self.writer.open()
        self.img_type = "sensor_msgs/msg/CompressedImage"
        self.cam_conn = self.writer.add_connection(camrec.TOPIC, self.img_type, typestore=typestore)
        self.can_conns = {addr: self.writer.add_connection(topic, CAN_MSGTYPE, typestore=typestore)
                          for _, addr, _, topic in CAN_MESSAGES}
        self.counts = {camrec.TOPIC: 0}
        self.counts.update({topic: 0 for _, _, _, topic in CAN_MESSAGES})
        self.first_ts = {}
        self.last_ts = {}
        self.closed = False

    def write(self, conn, topic, ts_ns, payload):
        with self.lock:
            if self.closed:
                return
            self.writer.write(conn, ts_ns, payload)
            self.counts[topic] += 1
            self.first_ts.setdefault(topic, ts_ns)
            self.last_ts[topic] = ts_ns

    def close(self):
        with self.lock:
            if not self.closed:
                self.closed = True
                self.writer.close()


class CanThread(threading.Thread):
    """Reads the `can` socket, decodes the three messages, writes them."""

    def __init__(self, session, typestore, epoch_offset, stop_event, stopper):
        super().__init__(daemon=True, name="can")
        self.session = session
        self.ts = typestore
        self.epoch_offset = epoch_offset
        self.stop_event = stop_event
        self.stopper = stopper
        self.CanMsg = typestore.types[CAN_MSGTYPE]
        self.Header = typestore.types["std_msgs/msg/Header"]
        self.Time = typestore.types["builtin_interfaces/msg/Time"]
        self.parsers = {addr: CANParser(DBC_NAME, [(name, hz)], CAN_BUS)
                        for name, addr, hz, _ in CAN_MESSAGES}
        self.topics = {addr: topic for _, addr, _, topic in CAN_MESSAGES}
        self.names = {addr: name for name, addr, _, _ in CAN_MESSAGES}
        self.raw_mismatch = 0
        self.batches = 0

    def run(self):
        try:
            sock = messaging.sub_sock("can", timeout=100)
            log("subscribed to `can`")
            while not self.stop_event.is_set():
                raw = messaging.drain_sock_raw(sock, wait_for_one=True)
                if raw:
                    for entry in can_capnp_to_list(raw):
                        self._handle(entry)
        except Exception as e:
            self.stopper("can thread failed: %r" % (e,))

    def _handle(self, entry):
        nanos, frames = entry
        self.batches += 1
        ts_ns = int(nanos) + self.epoch_offset
        sec, nsec = divmod(ts_ns, 1_000_000_000)
        for addr, parser in self.parsers.items():
            raw = [bytes(dat) for a, dat, src in frames if src == CAN_BUS and a == addr]
            if addr not in parser.update([entry]):
                continue
            vals = parser.vl_all[addr]
            names = sorted(vals)
            n = len(vals[names[0]]) if names else 0
            if len(raw) != n:
                self.raw_mismatch += 1
            valid = bool(parser.can_valid)
            for k in range(n):
                data = raw[k] if len(raw) == n else b""
                msg = self.CanMsg(
                    header=self.Header(stamp=self.Time(sec=sec, nanosec=nsec), frame_id=self.names[addr]),
                    address=addr, bus=CAN_BUS, can_valid=valid,
                    signal_names=names,
                    signal_values=np.array([vals[s][k] for s in names], dtype=np.float64),
                    data=np.frombuffer(data, dtype=np.uint8))
                self.session.write(self.session.can_conns[addr], self.topics[addr], ts_ns,
                                   self.ts.serialize_cdr(msg, CAN_MSGTYPE))


class DiskThread(threading.Thread):
    def __init__(self, path, stop_free_mb, period_s, stop_event, stopper):
        super().__init__(daemon=True, name="disk")
        self.path, self.stop_free_mb, self.period_s = path, stop_free_mb, period_s
        self.stop_event, self.stopper = stop_event, stopper
        self.last_free = None

    def run(self):
        while not self.stop_event.wait(self.period_s):
            self.last_free = free_mb(self.path)
            if self.last_free < self.stop_free_mb:
                self.stopper("low_disk", "free space %.0f MB < --stop-free-mb %.0f"
                             % (self.last_free, self.stop_free_mb))
                return


def camera_loop(session, typestore, epoch_offset, args, stop_event):
    CompressedImage = typestore.types["sensor_msgs/msg/CompressedImage"]
    Header = typestore.types["std_msgs/msg/Header"]
    Time = typestore.types["builtin_interfaces/msg/Time"]
    out_size = camrec.PRESETS[args.preset]["size"]
    min_interval = 1.0 / args.fps if args.fps > 0 else 0.0
    last_write = 0.0
    last_report = time.monotonic()
    client = None
    while not stop_event.is_set():
        if client is None:
            client = camrec.connect_camera()
            if client is None:
                stop_event.wait(1.0)
                continue
            log("camera connected: %dx%d" % (client.width, client.height))
        buf = client.recv()
        if buf is None:
            client = None           # reconnect our own client only; never touch camerad
            continue
        ts_ns = int(client.timestamp_eof) + epoch_offset
        now = time.monotonic()
        if now - last_report >= args.report_s:
            last_report = now
            log("counts " + ", ".join("%s=%d" % (t.rsplit("/", 1)[-1], c)
                                      for t, c in session.counts.items()))
        if now - last_write < min_interval:
            continue
        last_write = now
        jpeg = camrec.encode_jpeg(camrec.buf_to_rgb(buf, args.preset), out_size, args.quality)
        sec, nsec = divmod(ts_ns, 1_000_000_000)
        msg = CompressedImage(header=Header(stamp=Time(sec=sec, nanosec=nsec), frame_id=camrec.FRAME_ID),
                              format="jpeg", data=np.frombuffer(jpeg, dtype=np.uint8))
        session.write(session.cam_conn, camrec.TOPIC, ts_ns,
                      typestore.serialize_cdr(msg, "sensor_msgs/msg/CompressedImage"))


def main():
    ap = argparse.ArgumentParser(description="End-to-end dataset recorder (device side)")
    ap.add_argument("--session-dir", required=True, help="new folder for this session")
    ap.add_argument("--preset", choices=sorted(camrec.PRESETS), default="cityscapes")
    ap.add_argument("--quality", type=int, default=80)
    ap.add_argument("--fps", type=float, default=8.0,
                    help="camera cap; frames land on a 50 ms grid, so 8 gives ~6.4 Hz, 10.5 gives 10 Hz")
    ap.add_argument("--min-free-mb", type=float, default=500.0, help="refuse to start below this")
    ap.add_argument("--stop-free-mb", type=float, default=300.0, help="stop cleanly below this")
    ap.add_argument("--disk-check-s", type=float, default=5.0)
    ap.add_argument("--max-can-readers", type=int, default=12,
                    help="refuse to subscribe if `can` already has this many msgq readers")
    ap.add_argument("--cpus", default="0,1,2,5", help="CPU affinity; keep off 3/4/6/7 (pandad, controls, camerad, modeld)")
    ap.add_argument("--nice", type=int, default=10)
    ap.add_argument("--report-s", type=float, default=30.0)
    args = ap.parse_args()

    session_dir = Path(args.session_dir)
    lock_f = open(LOCK_PATH, "w")
    try:
        fcntl.flock(lock_f, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        sys.exit("[e2e] another byd_e2e_recorder is already running (%s held)" % LOCK_PATH)
    if (session_dir / "bag").exists():
        sys.exit("[e2e] %s already holds a bag; refusing to overwrite" % session_dir)
    session_dir.mkdir(parents=True, exist_ok=True)
    fm = free_mb(session_dir)
    if fm < args.min_free_mb:
        sys.exit("[e2e] REFUSING to start: %.0f MB free < --min-free-mb %.0f" % (fm, args.min_free_mb))
    readers = msgq_num_readers(MSGQ_CAN_SHM)
    if readers >= args.max_can_readers:
        sys.exit("[e2e] REFUSING to subscribe: `can` has %d/%d msgq readers (limit %d). Slots of exited "
                 "subscribers are only freed when card restarts (car off/on or reboot)."
                 % (readers, MSGQ_NUM_READERS, args.max_can_readers))

    cpus = {int(c) for c in args.cpus.split(",") if c.strip()}
    os.sched_setaffinity(0, cpus)
    os.nice(args.nice)
    Path(PID_PATH).write_text("%d\n" % os.getpid())
    signal.signal(signal.SIGHUP, signal.SIG_IGN)

    typestore = get_typestore(Stores.ROS2_JAZZY)
    typestore.register(get_types_from_msg(CAN_MSGDEF, CAN_MSGTYPE))
    epoch_offset = camrec.boottime_to_epoch_offset_ns()
    stop_event = threading.Event()
    stop_info = {"reason": None, "detail": ""}

    def stopper(reason, detail=""):
        if stop_info["reason"] is None:
            stop_info["reason"], stop_info["detail"] = reason, detail
            log("stopping: %s %s" % (reason, detail))
        stop_event.set()

    signal.signal(signal.SIGTERM, lambda s, f: stopper("signal", "SIGTERM"))
    signal.signal(signal.SIGINT, lambda s, f: stopper("signal", "SIGINT"))

    meta = {
        "session_dir": str(session_dir), "host": socket.gethostname(),
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "recorder_md5": file_md5(__file__), "camrec_md5": file_md5(camrec.__file__),
        "gear_assumption": "FORWARD (D) ALWAYS -- gear is not recorded",
        "car_params": read_car_params(),
        "camera": {"preset": args.preset, "quality": args.quality, "fps_cap": args.fps},
        "can": {"dbc": DBC_NAME, "bus": CAN_BUS, "messages": [
            {"name": n, "address": a, "declared_hz": hz, "topic": t} for n, a, hz, t in CAN_MESSAGES],
            "msgq_can_readers_before_subscribe": readers},
        "clock": {"epoch_offset_ns": epoch_offset, "boottime_minus_monotonic_ns_start": clock_gap_ns()},
        "disk": {"free_mb_start": round(fm), "min_free_mb": args.min_free_mb, "stop_free_mb": args.stop_free_mb},
        "cpus": sorted(cpus), "nice": args.nice, "pid": os.getpid(),
    }
    (session_dir / "README.txt").write_text(README_TEXT)
    (session_dir / "session.json").write_text(json.dumps(meta, indent=2))

    session = Session(session_dir, typestore)
    log("recording -> %s  (%.0f MB free, can readers %d/%d)" % (session.bag_path, fm, readers, MSGQ_NUM_READERS))
    can_t = CanThread(session, typestore, epoch_offset, stop_event, stopper)
    disk_t = DiskThread(session_dir, args.stop_free_mb, args.disk_check_s, stop_event, stopper)
    can_t.start()
    disk_t.start()
    t0 = time.monotonic()
    try:
        camera_loop(session, typestore, epoch_offset, args, stop_event)
    except Exception as e:
        stopper("camera loop failed", repr(e))
    finally:
        stop_event.set()
        can_t.join(timeout=5.0)
        if stop_info["reason"] == "low_disk":
            (session_dir / LOW_DISK_MARKER).write_text(
                "Recording stopped early: %s at %s UTC. The bag was closed cleanly and is valid up to "
                "that point.\n" % (stop_info["detail"], datetime.now(timezone.utc).isoformat()))
        session.close()
        meta.update({
            "stopped_utc": datetime.now(timezone.utc).isoformat(),
            "duration_s": round(time.monotonic() - t0, 3),
            "stop_reason": stop_info["reason"] or "camera loop ended", "stop_detail": stop_info["detail"],
            "counts": session.counts,
            "first_ts_ns": session.first_ts, "last_ts_ns": session.last_ts,
            "can_batches": can_t.batches, "can_raw_count_mismatch": can_t.raw_mismatch,
        })
        meta["clock"]["boottime_minus_monotonic_ns_stop"] = clock_gap_ns()
        meta["disk"]["free_mb_stop"] = round(free_mb(session_dir))
        (session_dir / "session.json").write_text(json.dumps(meta, indent=2))
        (session_dir / ".complete").touch()
        try:
            os.unlink(PID_PATH)
        except OSError:
            pass
        log("closed %s: %s" % (session.bag_path, session.counts))


if __name__ == "__main__":
    main()
