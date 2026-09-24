#!/usr/bin/env python3
# Kommu.AI — Road Camera -> rosbag2 Recorder (DEVICE SIDE, read-only)
#
# WHAT THIS IS
#   Records the road-facing camera (VisionIPC VISION_STREAM_ROAD) live, JPEG-
#   compresses each frame, and writes it into a rosbag2 (ROS2 Jazzy) bag using
#   the mcap storage plugin, via the pure-Python `rosbags` library. No ROS
#   install is required — not on this device, and not on whatever machine
#   later reads the bags back.
#
#   Frames are emitted at the standard Cityscapes semantic-segmentation input
#   resolution (1024x512) by default — see PRESETS below.
#
# WHAT THIS IS NOT
#   * Does not import, call, or reference panda, carcontroller.py,
#     manual_steer_hook.py, or any CAN module. No Panda access, no CAN.
#   * Does not write any param, does not publish to any cereal socket, does
#     not touch bukapilot's own recording pipeline (/data/media/0/realdata).
#   * Not added to manager.py — start/stop manually, same policy as the
#     manual-steer writer and byd_terminal_monitor.py.
#   * Pure read-only VisionIPC subscriber, exactly the pattern already used by
#     system/camerad/snapshot.py and claude/road_cam_streamer.py. Multiple
#     concurrent consumers of camerad's streams are already normal here
#     (modeld + dmonitoringmodeld run alongside), so this adds one more
#     read-only consumer and nothing else.
#
# RECONNECT BEHAVIOUR (why it's safe)
#   If VisionIpcClient.recv() returns None (camerad restarted, buffer torn
#   down, transient hiccup) this script re-connect()s its OWN client and keeps
#   going. It never starts, stops, or restarts camerad, and never touches
#   manager.py. Worst case this process wedges or is killed — zero effect on
#   bukapilot, modeld, or steering.
#
# MEASURED ON THIS DEVICE (RK3588, bumpbump v10.1.0, 2026-08-17)
#   Road camera is 1920x1200 NV12 @ 20 fps (stride 1920, uv_offset 2304000).
#   Per-frame cost, convert + JPEG encode, measured while OFFROAD (idle):
#     cityscapes  1024x512 :  58 + 47 ms = 105 ms ->  9.6 fps ceiling
#     seg-native   960x480 :  58 +  4 ms =  62 ms -> 16.1 fps ceiling
#     native-half  960x600 :  66 +  4 ms =  70 ms -> 14.2 fps ceiling
#     native-full 1920x1200: 276 + 15 ms = 291 ms ->  3.4 fps ceiling
#   Full-resolution recording therefore cannot keep up with the 20 fps camera.
#   The fast path below downsamples the Y plane by 2 (which makes it exactly
#   match the native U/V plane size, so no chroma upsampling is needed at all)
#   before the colour conversion — that is what buys the ~5x speedup.
#
#   These ceilings are IDLE numbers. Onroad, modeld/camerad/encoderd are
#   competing for the same cores, so expect less. --fps is a cap, not a
#   promise; the default 8 leaves margin. If you need more headroom, prefer
#   --preset seg-native: it is the same 2:1 crop as cityscapes but skips the
#   960->1024 upscale, which is what costs cityscapes its extra ~43 ms of
#   encode time while adding no actual image information. Resizing 960x480 up
#   to 1024x512 in the training dataloader is equivalent and free here.
#
# STORAGE (the real constraint on this device)
#   /data has ~5.1 GB free and bukapilot's own realdata already uses ~1.1 GB
#   and grows. There is no larger storage on this device — no SD, no USB
#   mount. On real road imagery (not a synthetic test pattern) expect roughly
#   55-70 KB per frame at q80, i.e. ~30 MB/min at the default 8 fps, so /data
#   alone holds on the order of three hours. Run the laptop-side
#   companion (byd_road_camera_bag_sync.py) to pull completed bags off the
#   device and free the space as you go. This script does NOT prune old bags
#   itself; it only refuses to start a new bag below --min-free-mb.
#
# RUN ON DEVICE (bukapilot keeps running untouched):
#   cd /data/openpilot
#   screen -S roscam
#   PYTHONPATH=/data/kommu_tools/pylibs:/data/openpilot \
#     /usr/local/venv/bin/python3 /data/kommu_tools/byd_road_camera_rosbag_recorder.py
#   # Ctrl+A then D to detach — recorder keeps running
#   # Stop cleanly: screen -r roscam, then Ctrl+C
#
# DEPENDENCY NOTE (why PYTHONPATH, not a venv install)
#   `rosbags` is installed into /data/kommu_tools/pylibs via
#     /usr/local/venv/bin/pip3 install --target /data/kommu_tools/pylibs rosbags
#   deliberately NOT into /usr/local/venv: rosbags pulls numpy>=2, while
#   openpilot pins numpy<2.0.0, so installing it into the venv would break
#   bukapilot. The bundled numpy/zstandard were deleted from that target dir
#   so the device's own numpy 1.26.4 is used instead. Keep it that way.
#
# OUTPUT LAYOUT
#   <OUT_ROOT>/YYYY/MM/DD/HH-MM-SS/     <- a rosbag2 bag DIRECTORY, containing
#                                          metadata.yaml + one .mcap data file
#   A bag directory also gets a `.complete` marker file written into it once it
#   has been closed cleanly. The laptop-side sync tool only transfers bags
#   carrying that marker, so it can never copy a half-written bag.

import argparse
import io
import os
import shutil
import signal
import sys
import time
from datetime import datetime
from pathlib import Path

OPENPILOT_PATH = os.environ.get("OPENPILOT_PATH", "/data/openpilot")
if OPENPILOT_PATH not in sys.path:
    sys.path.insert(0, OPENPILOT_PATH)

import numpy as np
from PIL import Image

# cv2 is NOT installed on this device — PIL is used for resize/JPEG instead.
from msgq.visionipc import VisionIpcClient, VisionStreamType

from rosbags.rosbag2 import StoragePlugin, Writer
from rosbags.typesys import Stores, get_typestore

# Single named constant so adding a second stream later is a small diff.
STREAM = VisionStreamType.VISION_STREAM_ROAD
TOPIC = "/kommu/road_camera/compressed"
FRAME_ID = "road_camera"

DEFAULT_OUT_ROOT = Path("/data/kommu_tools/rosbags")

# Output presets. "cityscapes" is the default: the standard semantic-
# segmentation input spec (1024x512, 2:1). The 1920x1200 sensor is 1.6:1, so
# reaching 2:1 requires a CENTRE CROP of the vertical FOV (120 px off the top
# and 120 px off the bottom at full res) rather than an aspect-distorting
# squash. Sky and hood are what get trimmed, which is normal practice for
# driving-scene segmentation. Use native-half to keep the full FOV instead.
PRESETS = {
    "cityscapes":  {"size": (1024, 512), "aspect": 2.0},
    "seg-native":  {"size": None,        "aspect": 2.0},    # 960x480, no upscale
    "native-half": {"size": None,        "aspect": None},   # 960x600, full FOV
    "native-full": {"size": None,        "aspect": None},   # 1920x1200, full FOV
}

# BT.601 full-range YUV->RGB, same matrix system/camerad/snapshot.py uses.
_YUV2RGB = np.array([
    [1.00000,  1.00000, 1.00000],
    [0.00000, -0.39465, 2.03211],
    [1.13983, -0.58060, 0.00000],
])


def nv12_planes(buf):
    """Split a VisionIPC NV12 buffer into y, u, v planes.

    Field layout verified live against this device: data / width / height /
    stride / uv_offset, with len(data) == stride * height * 3 // 2. Note the
    planes are sliced to `width` because stride may exceed width on other
    camera configs (they are equal at 1920 here).
    """
    d = np.frombuffer(buf.data, dtype=np.uint8)
    y = d[:buf.uv_offset].reshape((-1, buf.stride))[:buf.height, :buf.width]
    u = d[buf.uv_offset::2].reshape((-1, buf.stride // 2))[:buf.height // 2, :buf.width // 2]
    v = d[buf.uv_offset + 1::2].reshape((-1, buf.stride // 2))[:buf.height // 2, :buf.width // 2]
    return y, u, v


def _yuv_to_rgb(y, u, v):
    """Colour-convert planes that are ALREADY the same shape (no upsampling)."""
    yuv = np.dstack((y, u, v)).astype(np.int16)
    yuv[:, :, 1:] -= 128
    return np.dot(yuv, _YUV2RGB).clip(0, 255).astype(np.uint8)


def _yuv_to_rgb_full(y, u, v):
    """Full-resolution path: chroma upsampled 2x to match luma. Slow (~458 ms)."""
    ul = np.repeat(np.repeat(u, 2).reshape(u.shape[0], y.shape[1]), 2, axis=0).reshape(y.shape)
    vl = np.repeat(np.repeat(v, 2).reshape(v.shape[0], y.shape[1]), 2, axis=0).reshape(y.shape)
    return _yuv_to_rgb(y, ul, vl)


def buf_to_rgb(buf, preset):
    """NV12 buffer -> RGB ndarray at the preset's geometry.

    Cropping is done on the YUV planes BEFORE colour conversion so we never
    pay to convert pixels we are about to throw away.
    """
    y, u, v = nv12_planes(buf)

    if preset == "native-full":
        return _yuv_to_rgb_full(y, u, v)

    target_aspect = PRESETS[preset]["aspect"]
    if target_aspect is not None:
        # Centre-crop the vertical FOV to the target aspect, in full-res rows,
        # keeping the crop even so the half-res chroma planes stay aligned.
        keep_h = int(round(buf.width / target_aspect))
        if keep_h < buf.height:
            top = ((buf.height - keep_h) // 2) & ~1
            keep_h = keep_h & ~1
            y = y[top:top + keep_h, :]
            u = u[top // 2:(top + keep_h) // 2, :]
            v = v[top // 2:(top + keep_h) // 2, :]

    # Fast path: downsample luma 2x so it exactly matches the native chroma
    # plane size — no chroma upsampling required at all.
    return _yuv_to_rgb(y[::2, ::2], u, v)


def encode_jpeg(rgb, size, quality):
    img = Image.fromarray(rgb)
    if size is not None and img.size != tuple(size):
        img = img.resize(tuple(size), Image.BILINEAR)
    bio = io.BytesIO()
    img.save(bio, format="JPEG", quality=quality)
    return bio.getvalue()


def boottime_to_epoch_offset_ns():
    """VisionIPC timestamps are CLOCK_BOOTTIME nanoseconds, NOT wall clock.

    Verified on-device: client.timestamp_eof tracked CLOCK_BOOTTIME to within
    0.06 s while time.time_ns() was ~1.79e18. Writing the raw value into a bag
    would date every message to 1970 and break `ros2 bag info` / rqt_bag time
    alignment, so we add this offset to put messages on the real epoch.
    """
    return time.time_ns() - time.clock_gettime_ns(time.CLOCK_BOOTTIME)


def new_bag_path(out_root: Path) -> Path:
    """<OUT_ROOT>/YYYY/MM/DD/HH-MM-SS, with -1/-2 suffixes on collision.

    rosbags' Writer refuses to open into an existing directory, so two runs
    started inside the same second would otherwise crash the second one.
    """
    now = datetime.now()
    day = out_root / f"{now:%Y}" / f"{now:%m}" / f"{now:%d}"
    day.mkdir(parents=True, exist_ok=True)
    base = day / f"{now:%H-%M-%S}"
    if not base.exists():
        return base
    for n in range(1, 1000):
        cand = day / f"{now:%H-%M-%S}-{n}"
        if not cand.exists():
            return cand
    raise RuntimeError(f"cannot find a free bag name under {day}")


def connect_camera():
    client = VisionIpcClient("camerad", STREAM, True)
    if not client.connect(False):
        return None
    return client


class BagSession:
    """One open rosbag2 bag, closed and marked `.complete` on exit."""

    def __init__(self, path: Path, typestore, msgtype: str):
        self.path = path
        self.writer = Writer(path, version=8, storage_plugin=StoragePlugin.MCAP)
        self.writer.open()
        self.conn = self.writer.add_connection(TOPIC, msgtype, typestore=typestore)
        self.started = time.monotonic()
        self.frames = 0

    def close(self):
        try:
            self.writer.close()
        finally:
            # Marker the laptop-side sync tool keys off, so a bag still being
            # written is never transferred.
            try:
                (self.path / ".complete").touch()
            except OSError:
                pass

    @property
    def size_mb(self):
        return sum(f.stat().st_size for f in self.path.glob("*") if f.is_file()) / 1e6


def record(args):
    out_root = Path(args.out).expanduser()
    preset = PRESETS[args.preset]
    out_size = preset["size"]

    typestore = get_typestore(Stores.ROS2_JAZZY)
    CompressedImage = typestore.types["sensor_msgs/msg/CompressedImage"]
    Header = typestore.types["std_msgs/msg/Header"]
    Time = typestore.types["builtin_interfaces/msg/Time"]
    msgtype = CompressedImage.__msgtype__

    epoch_offset = boottime_to_epoch_offset_ns()

    client = None
    while client is None:
        client = connect_camera()
        if client is None:
            print("[roscam] waiting for camerad VisionIPC...", flush=True)
            time.sleep(1.0)
    print(f"[roscam] connected: {client.width}x{client.height} NV12", flush=True)

    min_interval = 1.0 / args.fps if args.fps > 0 else 0.0
    last_write = 0.0
    session = None
    stopping = False

    def _stop(_signum, _frame):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGTERM, _stop)

    def free_mb():
        return shutil.disk_usage(out_root).free / 1e6

    def open_bag():
        out_root.mkdir(parents=True, exist_ok=True)
        free = free_mb()
        if free < args.min_free_mb:
            raise SystemExit(
                f"[roscam] REFUSING to open a new bag: only {free:.0f} MB free on "
                f"{out_root} (--min-free-mb {args.min_free_mb}). Run the laptop-side "
                f"sync to pull completed bags off the device, then restart."
            )
        s = BagSession(new_bag_path(out_root), typestore, msgtype)
        print(f"[roscam] recording -> {s.path}  ({free:.0f} MB free)", flush=True)
        return s

    try:
        session = open_bag()
        while not stopping:
            buf = client.recv()
            if buf is None:
                # camerad restarted or buffers torn down — reconnect our own
                # client only; never touch camerad itself.
                print("[roscam] stream dropped, reconnecting...", flush=True)
                client = None
                while client is None and not stopping:
                    client = connect_camera()
                    if client is None:
                        time.sleep(1.0)
                continue

            # Read the timestamp off the CLIENT (the buffer has no timestamp
            # attributes — verified: VisionBuf exposes only data/fd/height/
            # idx/stride/uv_offset/width).
            ts_ns = int(client.timestamp_eof) + epoch_offset

            now = time.monotonic()
            if now - last_write < min_interval:
                continue
            last_write = now

            if args.rotate_seconds > 0 and (now - session.started) >= args.rotate_seconds:
                print(f"[roscam] rotating: {session.frames} frames, "
                      f"{session.size_mb:.1f} MB", flush=True)
                session.close()
                session = open_bag()

            rgb = buf_to_rgb(buf, args.preset)
            jpeg = encode_jpeg(rgb, out_size, args.quality)

            sec, nsec = divmod(ts_ns, 1_000_000_000)
            msg = CompressedImage(
                header=Header(stamp=Time(sec=sec, nanosec=nsec), frame_id=FRAME_ID),
                format="jpeg",
                data=np.frombuffer(jpeg, dtype=np.uint8),
            )
            session.writer.write(session.conn, ts_ns,
                                 typestore.serialize_cdr(msg, msgtype))
            session.frames += 1

            if args.verbose and session.frames % 100 == 0:
                print(f"[roscam] {session.frames} frames, {session.size_mb:.1f} MB, "
                      f"{free_mb():.0f} MB free", flush=True)

    except KeyboardInterrupt:
        print("\n[roscam] stopping (Ctrl-C)", flush=True)
    finally:
        if session is not None:
            session.close()
            print(f"[roscam] closed {session.path}: {session.frames} frames, "
                  f"{session.size_mb:.1f} MB", flush=True)


def main():
    ap = argparse.ArgumentParser(
        description="Record the Kommu road camera into rosbag2 (ROS2 Jazzy) mcap bags.")
    ap.add_argument("--out", default=str(DEFAULT_OUT_ROOT), help="output root directory")
    ap.add_argument("--preset", choices=sorted(PRESETS), default="cityscapes",
                    help="output geometry: cityscapes=1024x512 (2:1 centre crop, "
                         "standard semseg input spec), seg-native=960x480 (same "
                         "crop, no upscale, ~1.7x cheaper), native-half=960x600 "
                         "full FOV, native-full=1920x1200 full FOV (~3.4 fps ceiling)")
    ap.add_argument("--quality", type=int, default=80, help="JPEG quality 1-100")
    ap.add_argument("--fps", type=float, default=8.0,
                    help="cap on recorded fps (0 = every available frame). This is "
                         "a cap, not a guarantee — see the per-preset ceilings above.")
    ap.add_argument("--rotate-seconds", type=int, default=600,
                    help="start a new bag every N seconds (0 = never)")
    ap.add_argument("--min-free-mb", type=float, default=500.0,
                    help="refuse to open a new bag below this much free space")
    ap.add_argument("--verbose", action="store_true", help="periodic progress lines")
    args = ap.parse_args()

    if args.preset == "native-full" and args.fps > 3:
        print(f"[roscam] NOTE: native-full tops out near 3.4 fps even idle on this "
              f"device; --fps {args.fps:g} will not be reached.", flush=True)

    record(args)


if __name__ == "__main__":
    main()
