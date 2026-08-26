#!/usr/bin/env python3
"""
byd_odom_node — ROS2 node publishing TWO parallel live odometry estimates for
the BYD Dolphin, for direct visual comparison in RViz.

    DEVICE (Kommu, no ROS)                    LAPTOP (ROS2 Jazzy)
    ----------------------                    -------------------
    bukapilot / cereal
        |
        v
    byd_cereal_server.py  --- TCP/JSON --->   THIS NODE
    (existing project tool,                       |
     port 5556)                                   +--> /byd/odom_measured   (nav_msgs/Odometry)
                                                   +--> /byd/path_measured   (nav_msgs/Path)
                                                   +--> TF: odom -> base_link_measured
                                                   |
                                                   +--> /byd/odom_corrected  (nav_msgs/Odometry)
                                                   +--> /byd/path_corrected  (nav_msgs/Path)
                                                   +--> TF: odom -> base_link_corrected
                                                            |
                                                            v
                                                          RViz2 (green = measured, orange = corrected)

  The raw uncorrected kinematic track (/byd/*_kinematic, steer_ratio=13.11,
  no offset removal) is DEPRECATED and commented out — see the block in
  __init__. It is no longer constructed, stepped, or published.

INTEGRATORS (same input stream, different heading source):

  MEASURED  — heading rate comes directly from the car's own yaw_rate signal
              (cs.yawRate via cereal). This is the car's actual sensed rotation,
              not derived from steering geometry at all.

  KINEMATIC — heading rate is derived purely from steer_deg + v_kmh through the
              bicycle model (tan(delta)/wheelbase). No sensor fusion, no
              correction — this is exactly what the earlier byd_odometry.py
              work characterized as having an ~8% yaw over-prediction at
              steer_ratio=13.11 (effective kinematic ratio nearer 14.1-14.3).

Both integrators consume the SAME v_kmh sample at the SAME timestep, so any
divergence you see between the two paths in RViz is attributable to the
heading-rate source alone, not to different speed data.

⚠️ REQUIRES byd_cereal_server.py to emit a "yaw_rate" field. If it's absent
from a given sample, the MEASURED integrator holds its heading constant for
that tick (does not fall back to the kinematic estimate silently — that would
defeat the point of having two independent traces to compare) and a one-time
warning is logged.

Constants override the CAR.BYD_SEAL CarSpecs placeholders (Seal values, not
Dolphin): wheelbase = 2.70 m (placeholder 2.92), steer_ratio = 13.11 (placeholder
16.0, and itself known to carry ~8% kinematic bias per prior validation).
"""

import argparse
import json
import math
import socket
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Quaternion, TransformStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from tf2_ros import TransformBroadcaster

WHEELBASE_M = 2.70
STEER_RATIO = 13.11
MIN_SPEED_MS = 0.05


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def tire_angle_rad(wheel_angle_deg: float, steer_ratio: float) -> float:
    return math.radians(wheel_angle_deg) / steer_ratio


def heading_rate_rad_s(v_ms: float, tire_rad: float, wheelbase: float) -> float:
    return v_ms * math.tan(tire_rad) / wheelbase


class DeviceStreamClient(threading.Thread):
    """Reads newline-delimited JSON from the device's cereal TCP server.
    Own thread so a network stall never blocks the ROS executor. Reconnects
    indefinitely."""

    def __init__(self, host, port, logger):
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.logger = logger
        self.lock = threading.Lock()
        self.latest = None
        self.connected = False
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def get_latest(self):
        with self.lock:
            return self.latest

    def run(self):
        while not self._stop.is_set():
            sock = None
            try:
                self.logger.info(f"connecting to device {self.host}:{self.port} ...")
                sock = socket.create_connection((self.host, self.port), timeout=5.0)
                sock.settimeout(2.0)
                self.connected = True
                self.logger.info("connected to device cereal stream")
                buf = b""
                while not self._stop.is_set():
                    try:
                        chunk = sock.recv(4096)
                    except socket.timeout:
                        continue
                    if not chunk:
                        raise ConnectionError("device closed the stream")
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            d = json.loads(line)
                        except json.JSONDecodeError:
                            continue
                        with self.lock:
                            self.latest = (d, time.monotonic())
            except Exception as e:
                self.connected = False
                self.logger.warn(f"device stream lost ({e}); retrying in 2s")
                time.sleep(2.0)
            finally:
                if sock is not None:
                    try:
                        sock.close()
                    except Exception:
                        pass


class Integrator:
    """One independent x/y/yaw dead-reckoning track."""

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

    def step(self, v_ms: float, yaw_rate: float, dt: float):
        self.yaw += yaw_rate * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        self.x += v_ms * math.cos(self.yaw) * dt
        self.y += v_ms * math.sin(self.yaw) * dt


class BydOdomNode(Node):
    def __init__(self, args):
        super().__init__("byd_odom_node")

        self.wheelbase = args.wheelbase
        self.steer_ratio = args.steer_ratio
        self.corrected_steer_ratio = args.corrected_steer_ratio
        self.stale_s = args.stale_timeout
        self.frame_odom = args.odom_frame

        self.meas = Integrator()
        # ─────────────────────────────────────────────────────────────────────
        # DEPRECATED — raw/uncalibrated kinematic track. Kept commented out for
        # reference only. DO NOT re-enable in ./byd_drive.sh or any launch config.
        #
        # This is the ORIGINAL bicycle-model integrator: steer_ratio=13.11 with
        # NO steering-angle offset correction. 13.11 is the port's STATIC
        # control-tuned value (opendbc car/byd/values.py, also this node's
        # --steer-ratio default) — it is NOT a liveParameters online-calibrated
        # figure. It over-predicts yaw rate by ~8% per prior validation;
        # independently corroborated here on 2026-08-25, where switching to
        # 14.2 alone moved the yaw estimate -7.7%. It also ignores the steering
        # centre-offset bias, measured live on 2026-08-25 at 1.537 deg via
        # liveParameters.angleOffsetAverageDeg — a further -5.1% at 30 deg
        # steer, and strongly steer-dependent (-31% at 6 deg, -9% at 90 deg),
        # which is the signature of a constant bias and shows up as consistent
        # curvature error on gentle steering rather than a uniform scale error.
        #
        # Superseded by the `corrected` track (steer_ratio=14.2, offset-removed),
        # which is now the only steer-derived track this package publishes.
        # Left here, disabled, purely so the old behaviour can be reproduced for
        # comparison if ever needed again. `delta` and `yaw_rate_kin` are used
        # by nothing else, so this commenting-out is self-contained.
        # ─────────────────────────────────────────────────────────────────────
        # self.kin = Integrator()
        self.corrected = Integrator()
        self.last_t = None
        self._warned_no_yaw_rate = False

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub_odom_meas = self.create_publisher(Odometry, "/byd/odom_measured", qos)
        self.pub_path_meas = self.create_publisher(Path, "/byd/path_measured", qos)
        # DEPRECATED (see block above)
        # self.pub_odom_kin = self.create_publisher(Odometry, "/byd/odom_kinematic", qos)
        # self.pub_path_kin = self.create_publisher(Path, "/byd/path_kinematic", qos)
        self.pub_odom_corr = self.create_publisher(Odometry, "/byd/odom_corrected", qos)
        self.pub_path_corr = self.create_publisher(Path, "/byd/path_corrected", qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.path_meas = Path()
        self.path_meas.header.frame_id = self.frame_odom
        # DEPRECATED (see block above)
        # self.path_kin = Path()
        # self.path_kin.header.frame_id = self.frame_odom
        self.path_corr = Path()
        self.path_corr.header.frame_id = self.frame_odom
        self.max_path_poses = args.max_path_poses
        # Path republish is decoupled from the odom/TF tick rate: a Path message
        # carries its whole history, so publishing it costs O(len(poses)) and, at
        # full tick rate, eventually overruns the timer period as the path grows.
        # Poses are still appended every tick — only the publish is throttled.
        self.path_publish_every_n = max(1, round(args.rate / args.path_publish_hz))
        self._tick_count = 0

        self.client = DeviceStreamClient(args.host, args.port, self.get_logger())
        self.client.start()

        self.timer = self.create_timer(1.0 / args.rate, self.tick)
        self.get_logger().info(
            f"byd_odom_node up: wheelbase={self.wheelbase} m. "
            f"Publishing /byd/*_measured (needs cereal yaw_rate) and /byd/*_corrected "
            f"(steer-derived, steer_ratio={self.corrected_steer_ratio}, angle-offset removed). "
            f"The raw /byd/*_kinematic track (steer_ratio={self.steer_ratio}) is DEPRECATED "
            f"and not published. "
            f"odom+TF at {args.rate:.0f} Hz; Path republished every {self.path_publish_every_n} "
            f"ticks (~{args.rate / self.path_publish_every_n:.1f} Hz)."
        )

    def _publish_track(self, integ: Integrator, path_msg: Path, pub_odom, pub_path,
                        child_frame: str, stamp, v_ms: float, yaw_rate: float):
        quat = yaw_to_quaternion(integ.yaw)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.frame_odom
        tf.child_frame_id = child_frame
        tf.transform.translation.x = integ.x
        tf.transform.translation.y = integ.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation = quat
        self.tf_broadcaster.sendTransform(tf)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_odom
        odom.child_frame_id = child_frame
        odom.pose.pose.position.x = integ.x
        odom.pose.pose.position.y = integ.y
        odom.pose.pose.orientation = quat
        odom.twist.twist.linear.x = v_ms
        odom.twist.twist.angular.z = yaw_rate
        pub_odom.publish(odom)

        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = self.frame_odom
        ps.pose = odom.pose.pose
        path_msg.poses.append(ps)
        if len(path_msg.poses) > self.max_path_poses:
            path_msg.poses = path_msg.poses[-self.max_path_poses:]
        path_msg.header.stamp = stamp
        # Throttled: see path_publish_every_n. Gated on the per-tick counter (not a
        # per-track one) so both tracks' Paths go out on the same ticks, in sync.
        if (self._tick_count % self.path_publish_every_n) == 0:
            pub_path.publish(path_msg)

    def tick(self):
        item = self.client.get_latest()
        if item is None:
            return
        d, rx_mono = item

        if (time.monotonic() - rx_mono) > self.stale_s:
            return

        v_ms = float(d.get("v_kmh", 0.0)) / 3.6
        steer_deg = float(d.get("steer_deg", 0.0))

        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9
        if self.last_t is None:
            self.last_t = t
            return
        dt = t - self.last_t
        self.last_t = t
        if dt <= 0.0 or dt > 1.0:
            return

        v_for_integration = 0.0 if abs(v_ms) < MIN_SPEED_MS else v_ms

        # --- DEPRECATED uncorrected kinematic yaw rate (see block in __init__) ---
        # delta = tire_angle_rad(steer_deg, self.steer_ratio)
        # yaw_rate_kin = heading_rate_rad_s(v_for_integration, delta, self.wheelbase)

        # --- Corrected kinematic yaw rate: same bicycle model as `kin`, but with
        # (1) a separately-configurable steer ratio, and (2) the steering-angle
        # sensor's centre bias removed. Both are independent error sources; `kin`
        # is deliberately left uncorrected so the three tracks stay comparable.
        angle_offset_deg = float(d.get("angle_offset_deg", 0.0) or 0.0)
        effective_steer_deg = steer_deg - angle_offset_deg
        delta_corr = tire_angle_rad(effective_steer_deg, self.corrected_steer_ratio)
        yaw_rate_corr = heading_rate_rad_s(v_for_integration, delta_corr, self.wheelbase)

        # --- Measured yaw rate: from the car's own sensor, via cereal ---
        raw_yaw_rate = d.get("yaw_rate")
        if raw_yaw_rate is None:
            if not self._warned_no_yaw_rate:
                self.get_logger().warn(
                    "cereal stream has no 'yaw_rate' field — the MEASURED track "
                    "will hold its heading constant (NOT falling back to the "
                    "kinematic estimate) until the server is updated to emit it."
                )
                self._warned_no_yaw_rate = True
            yaw_rate_meas = 0.0
        else:
            yaw_rate_meas = float(raw_yaw_rate)
            if abs(v_ms) < MIN_SPEED_MS:
                yaw_rate_meas = 0.0

        self.meas.step(v_for_integration, yaw_rate_meas, dt)
        # self.kin.step(v_for_integration, yaw_rate_kin, dt)   # DEPRECATED
        self.corrected.step(v_for_integration, yaw_rate_corr, dt)

        stamp = now.to_msg()
        self._tick_count += 1
        self._publish_track(self.meas, self.path_meas, self.pub_odom_meas, self.pub_path_meas,
                             "base_link_measured", stamp, v_for_integration, yaw_rate_meas)
        # DEPRECATED — no longer published:
        # self._publish_track(self.kin, self.path_kin, self.pub_odom_kin, self.pub_path_kin,
        #                      "base_link_kinematic", stamp, v_for_integration, yaw_rate_kin)
        self._publish_track(self.corrected, self.path_corr, self.pub_odom_corr, self.pub_path_corr,
                             "base_link_corrected", stamp, v_for_integration, yaw_rate_corr)

    def destroy_node(self):
        self.client.stop()
        return super().destroy_node()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="172.20.10.3", help="device IP (dynamic — check per network)")
    ap.add_argument("--port", type=int, default=5556)
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--path-publish-hz", type=float, default=10.0,
                    help="how often the accumulated Path messages are republished. "
                         "Poses are still APPENDED every tick; only the publish is "
                         "throttled, because republishing the whole Path costs O(n) "
                         "in its length.")
    ap.add_argument("--wheelbase", type=float, default=WHEELBASE_M)
    ap.add_argument("--steer-ratio", type=float, default=STEER_RATIO)
    ap.add_argument("--corrected-steer-ratio", type=float, default=14.2,
                    help="steer ratio for the CORRECTED track. 13.11 is the "
                         "control-tuned value and over-predicts yaw by ~8%%; prior "
                         "validation put the odometry-fitted value near 14.1-14.3.")
    ap.add_argument("--stale-timeout", type=float, default=0.5)
    ap.add_argument("--max-path-poses", type=int, default=20000)
    ap.add_argument("--odom-frame", default="odom")

    argv = [a for a in sys.argv[1:] if not a.startswith("__")]
    args, _ = ap.parse_known_args(argv)

    rclpy.init()
    node = BydOdomNode(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
