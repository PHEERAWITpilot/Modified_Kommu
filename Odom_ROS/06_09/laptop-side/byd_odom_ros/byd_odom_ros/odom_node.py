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
import fcntl
import json
import math
import os
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

    def step_absolute_heading(self, v_ms: float, heading_rad: float, dt: float):
        """Position integration with heading SET, not accumulated.

        Used only by the `steerproxy` track. Everything else calls step(), which
        integrates a yaw RATE. Here the heading is overwritten each tick with an
        absolute value, so no history is carried. x/y integration is otherwise
        identical, so the two tracks differ in exactly one respect.
        """
        self.yaw = math.atan2(math.sin(heading_rad), math.cos(heading_rad))
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
        self.deltaref = Integrator()
        self.yawsensor = Integrator()
        self.startref = Integrator()
        self.steerproxy = Integrator()
        self.last_t = None
        self._warned_no_yaw_rate = False
        # --- deltaref track state (independent of `corrected`) ---
        # Reconstructs steering angle from the CHANGE between consecutive raw
        # samples, so any CONSTANT sensor bias cancels: summing deltas
        # telescopes to steer(t) - steer(t0), and the bias appears in both
        # terms. The tradeoff is that it re-zeros ONCE at startup and holds
        # that reference for the session -- it does NOT track in-session
        # drift, which `corrected` does via liveParameters.angleOffsetAverageDeg.
        self._deltaref_prev_steer_deg = None   # last raw sample; None until first
        self._deltaref_theta = 0.0             # running reconstruction, 0 at t0
        self._warned_deltaref_zero_ref = False
        self._warned_no_yaw_sensor = False
        # --- startref state -------------------------------------------------
        # Same delta accumulation as deltaref, but based at the REAL starting
        # reading instead of 0.0. That sum telescopes to steer(t0) + steer(t)
        # - steer(t0) = steer(t), so this track IS the raw uncorrected steering
        # angle. Proven sample-by-sample against a 55522-sample real log:
        # max difference 1.78e-15 deg (float noise, 5.6e13x below the sensor's
        # own 0.1 deg resolution). See tests/test_startref_equivalence.py.
        self._startref_prev_steer_deg = None
        self._startref_theta = 0.0
        self._warned_startref = False
        self._warned_steerproxy = False

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
        self.pub_odom_dref = self.create_publisher(Odometry, "/byd/odom_deltaref", qos)
        self.pub_path_dref = self.create_publisher(Path, "/byd/path_deltaref", qos)
        self.pub_odom_yaws = self.create_publisher(Odometry, "/byd/odom_yawsensor", qos)
        self.pub_path_yaws = self.create_publisher(Path, "/byd/path_yawsensor", qos)
        self.pub_odom_sref = self.create_publisher(Odometry, "/byd/odom_startref", qos)
        self.pub_path_sref = self.create_publisher(Path, "/byd/path_startref", qos)
        self.pub_odom_sprx = self.create_publisher(Odometry, "/byd/odom_steerproxy", qos)
        self.pub_path_sprx = self.create_publisher(Path, "/byd/path_steerproxy", qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.path_meas = Path()
        self.path_meas.header.frame_id = self.frame_odom
        # DEPRECATED (see block above)
        # self.path_kin = Path()
        # self.path_kin.header.frame_id = self.frame_odom
        self.path_corr = Path()
        self.path_corr.header.frame_id = self.frame_odom
        self.path_dref = Path()
        self.path_dref.header.frame_id = self.frame_odom
        self.path_yaws = Path()
        self.path_yaws.header.frame_id = self.frame_odom
        self.path_sref = Path()
        self.path_sref.header.frame_id = self.frame_odom
        self.path_sprx = Path()
        self.path_sprx.header.frame_id = self.frame_odom
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
            f"(steer-derived, steer_ratio={self.corrected_steer_ratio}, angle-offset removed) "
            f"and /byd/*_deltaref (same ratio, delta-reconstructed angle) "
            f"and /byd/*_yawsensor (car's own CAN yaw sensor — no ratio, no offset) "
            f"and /byd/*_startref (raw uncorrected steer, ratio {self.corrected_steer_ratio} — expected to drift like the deprecated kinematic track). "
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

        # --- deltaref yaw rate: SAME ratio as `corrected`, different bias handling ---
        # Independent code path; reads nothing from and writes nothing to the
        # corrected track. See the state comments in __init__ for the tradeoff.
        if self._deltaref_prev_steer_deg is None:
            # First valid sample: capture the zero reference. No delta exists yet,
            # so theta stays 0.0 and this tick contributes no rotation.
            self._deltaref_prev_steer_deg = steer_deg
            if not self._warned_deltaref_zero_ref:
                self.get_logger().warn(
                    "deltaref track: steering-angle zero-reference captured at startup "
                    f"(raw steer_deg={steer_deg:.2f}) — ASSUMES THE WHEELS ARE STRAIGHT "
                    "RIGHT NOW. If they were not, that error carries as a residual bias "
                    "for this whole session. deltaref is immune to a CONSTANT sensor "
                    "bias but does NOT track in-session drift; the `corrected` track "
                    "does, via liveParameters.angleOffsetAverageDeg."
                )
                self._warned_deltaref_zero_ref = True
        else:
            self._deltaref_theta += steer_deg - self._deltaref_prev_steer_deg
            self._deltaref_prev_steer_deg = steer_deg
        delta_dref = tire_angle_rad(self._deltaref_theta, self.corrected_steer_ratio)
        yaw_rate_dref = heading_rate_rad_s(v_for_integration, delta_dref, self.wheelbase)

        # --- startref: delta accumulation based at the REAL startup reading ----
        # Identical in form to deltaref except for the base. Because the sum
        # telescopes, theta == steer_deg every tick: this track is the RAW,
        # UNCORRECTED steering angle, i.e. exactly what the deprecated kinematic
        # track integrated (only at ratio 14.2 instead of 13.11). It is expected
        # to reproduce that track's drift, and is published for visual comparison
        # rather than as a candidate correction.
        if self._startref_prev_steer_deg is None:
            self._startref_prev_steer_deg = steer_deg
            self._startref_theta = steer_deg          # base = real reading, NOT 0.0
            if not self._warned_startref:
                self.get_logger().warn(
                    f"startref track: based at the RAW startup reading "
                    f"(steer_deg={steer_deg:.2f}), then accumulating deltas. That sum "
                    "telescopes to steer(t), so this track IS the raw uncorrected "
                    "steering angle -- it applies NO offset correction and is expected, "
                    "by construction, to drift the same way the deprecated kinematic "
                    "track did. Published for comparison, not as a fix."
                )
                self._warned_startref = True
        else:
            self._startref_theta += steer_deg - self._startref_prev_steer_deg
            self._startref_prev_steer_deg = steer_deg
        delta_sref = tire_angle_rad(self._startref_theta, self.corrected_steer_ratio)
        yaw_rate_sref = heading_rate_rad_s(v_for_integration, delta_sref, self.wheelbase)

        # --- steerproxy: tire angle used DIRECTLY as heading -------------------
        # Structurally unlike every other track: there is no rate integration at
        # all. heading := tire_angle(steer) each tick, overwriting whatever came
        # before, so no turn history is retained.
        heading_sprx = tire_angle_rad(steer_deg, self.corrected_steer_ratio)
        if not self._warned_steerproxy:
            self.get_logger().warn(
                "steerproxy track: heading is NOT accumulated — it is set to the "
                "current tire angle every tick. HYPOTHESIS UNDER TEST: it will snap "
                "back toward 'facing straight' whenever the wheel returns to centre, "
                "regardless of how far the car actually turned earlier, and can never "
                "represent more than max_steer/ratio of heading. Checked against the "
                "2026-09-05 3-loop log: over 30 straight-after-turn segments this "
                "method averaged +0.06 deg while the accumulated heading averaged "
                "+530.3 deg. Published to see that failure, not as a candidate."
            )
            self._warned_steerproxy = True

        # --- yawsensor: the car's OWN physical yaw-rate sensor (CAN 546) --------
        # No steer ratio, no centre offset, no bicycle model. This is a direct
        # angular-rate measurement; the other tracks all INFER rotation from
        # steering geometry. Validated on a real drive 2026-09-03: sign matched
        # steering 45/45 samples, ~0 on straights, 22 deg/s peak at a tight
        # car-park turn. Distinct from `measured`, which reads cs.yawRate and is
        # structurally always 0 because the BYD port never assigns it.
        raw_yaw_sensor = d.get("yaw_sensor_rate")
        yaw_sensor_ok = bool(d.get("yaw_sensor_ok", False))
        if raw_yaw_sensor is None or not yaw_sensor_ok:
            if not self._warned_no_yaw_sensor:
                self.get_logger().warn(
                    "yawsensor track: cereal stream has no valid 'yaw_sensor_rate' — "
                    "the track will hold heading. Update byd_cereal_server.py "
                    "(needs the YAW_SENSOR CANParser block) or check can_valid."
                )
                self._warned_no_yaw_sensor = True
            yaw_rate_yaws = 0.0
        else:
            yaw_rate_yaws = float(raw_yaw_sensor)
            # Same standstill gate as the other tracks: below the speed floor we
            # are not travelling, so integrating sensor noise only adds drift.
            if abs(v_ms) < MIN_SPEED_MS:
                yaw_rate_yaws = 0.0

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
        self.deltaref.step(v_for_integration, yaw_rate_dref, dt)
        self.yawsensor.step(v_for_integration, yaw_rate_yaws, dt)
        self.startref.step(v_for_integration, yaw_rate_sref, dt)
        self.steerproxy.step_absolute_heading(v_for_integration, heading_sprx, dt)

        stamp = now.to_msg()
        self._tick_count += 1
        self._publish_track(self.meas, self.path_meas, self.pub_odom_meas, self.pub_path_meas,
                             "base_link_measured", stamp, v_for_integration, yaw_rate_meas)
        # DEPRECATED — no longer published:
        # self._publish_track(self.kin, self.path_kin, self.pub_odom_kin, self.pub_path_kin,
        #                      "base_link_kinematic", stamp, v_for_integration, yaw_rate_kin)
        self._publish_track(self.corrected, self.path_corr, self.pub_odom_corr, self.pub_path_corr,
                             "base_link_corrected", stamp, v_for_integration, yaw_rate_corr)
        self._publish_track(self.deltaref, self.path_dref, self.pub_odom_dref, self.pub_path_dref,
                             "base_link_deltaref", stamp, v_for_integration, yaw_rate_dref)
        self._publish_track(self.yawsensor, self.path_yaws, self.pub_odom_yaws, self.pub_path_yaws,
                             "base_link_yawsensor", stamp, v_for_integration, yaw_rate_yaws)
        self._publish_track(self.startref, self.path_sref, self.pub_odom_sref, self.pub_path_sref,
                             "base_link_startref", stamp, v_for_integration, yaw_rate_sref)
        self._publish_track(self.steerproxy, self.path_sprx, self.pub_odom_sprx, self.pub_path_sprx,
                             "base_link_steerproxy", stamp, v_for_integration, 0.0)

    def destroy_node(self):
        self.client.stop()
        return super().destroy_node()


SINGLETON_LOCK = "/tmp/byd_odom_node.lock"
_lock_fh = None          # module-global: the lock lives as long as the process


def _acquire_singleton(allow_multiple: bool) -> None:
    """Refuse to start if another odom_node already holds the lock.

    Two instances publish to the SAME topics with INDEPENDENT integrator state,
    so RViz interleaves two diverging trajectories and the path appears to jump.
    It is invisible to `ros2 topic hz` and it silently corrupted three separate
    measurements on 2026-09-05 before being spotted.

    The guard lives HERE rather than only in byd_drive.sh because every other
    entry path bypasses that script -- `ros2 launch ...` (which the README even
    documents as the manual fallback), `ros2 run ...`, or an IDE. An flock is
    released automatically when the holder dies, so a crash cannot leave a
    stale lock behind.
    """
    global _lock_fh
    if allow_multiple:
        print("[odom_node] --allow-multiple: skipping the single-instance guard. "
              "Two nodes will publish to the same topics with separate integrator "
              "state; expect the path to jump.")
        return
    _lock_fh = open(SINGLETON_LOCK, "w")
    try:
        fcntl.flock(_lock_fh, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        print(f"[odom_node] REFUSING to start: another odom_node already holds "
              f"{SINGLETON_LOCK}.", file=sys.stderr)
        print("[odom_node] Two instances publish to the same topics with separate "
              "integrator state, which makes the RViz path jump.", file=sys.stderr)
        print("[odom_node] Use  ./byd_drive.sh <ip> --kill-existing  to replace it, "
              "or pass --allow-multiple if you genuinely want two.", file=sys.stderr)
        sys.exit(1)
    _lock_fh.write(str(os.getpid()))
    _lock_fh.flush()


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
    ap.add_argument("--allow-multiple", action="store_true",
                    help="bypass the single-instance guard and start a second "
                         "node anyway. Expect a jumping path; see _acquire_singleton.")
    ap.add_argument("--corrected-steer-ratio", type=float, default=14.2,
                    help="steer ratio for the CORRECTED track. 13.11 is the "
                         "control-tuned value and over-predicts yaw by ~8%%; prior "
                         "validation put the odometry-fitted value near 14.1-14.3.")
    ap.add_argument("--stale-timeout", type=float, default=0.5)
    ap.add_argument("--max-path-poses", type=int, default=20000)
    ap.add_argument("--odom-frame", default="odom")

    argv = [a for a in sys.argv[1:] if not a.startswith("__")]
    args, _ = ap.parse_known_args(argv)

    # Before ANY ROS state exists — a refused start must leave nothing behind.
    _acquire_singleton(args.allow_multiple)

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
