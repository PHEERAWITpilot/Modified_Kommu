#!/usr/bin/env python3
"""byd_odom_replay.py — republish a saved odom.csv onto the live topic names.

A recording is just numbers on disk; RViz cannot open it. This node reads one
`Odom_record/YYYY/MM/DD/HHMM/odom.csv` and publishes the tracks it contains to
exactly the topics odom_node.py uses, so `rviz/byd_odom.rviz` renders a past
drive with no config changes — same track names, same colours.

    ./byd_replay.sh                       # newest recording + RViz
    ./byd_replay.sh path/to/odom.csv      # a specific one
    python3 byd_odom_replay.py FILE       # node only, no RViz

Two modes:

  static (default)  the whole drive is published at once and then republished
                    once a second so RViz picks it up whenever it connects.
                    This is what you want to review a finished drive.

  --realtime        walks the recording in time order, optionally scaled by
                    --speed, so you can watch the tracks diverge as they did
                    live. TF is published too, so the car frames move.

DO NOT run this while odom_node.py is running. Both would publish to the same
topics and RViz would draw the live drive and the recording interleaved — the
same duplicate-publisher failure the flock guard in byd_drive.sh exists to
prevent. byd_replay.sh checks for you.
"""

import argparse
import csv
import math
import os
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Quaternion, PoseStamped, TransformStamped
from nav_msgs.msg import Path
from tf2_ros import TransformBroadcaster

# Column prefix -> (topic suffix, TF child frame). Must match RECORD_TRACKS and
# the publisher names in odom_node.py, or RViz silently shows nothing.
TRACKS = [
    ("measured",         "measured",   "base_link_measured"),
    ("corrected",        "corrected",  "base_link_corrected"),
    ("deltaref",         "deltaref",   "base_link_deltaref"),
    ("yawsensor",        "yawsensor",  "base_link_yawsensor"),
    ("startref",         "startref",   "base_link_startref"),
    ("steerproxy",       "steerproxy", "base_link_steerproxy"),
    ("trackA_persample", "yawoffset",  "base_link_yawoffset"),
    ("trackB_windowed",  "yawavg",     "base_link_yawavg"),
    ("ekf",              "ekf",        "base_link_ekf"),           # recordings before 2026-09-15
    ("ekf_ba_slow",      "ekf_ba_slow",  "base_link_ekf_ba_slow"),
    ("ekf_ba_tight",     "ekf_ba_tight", "base_link_ekf_ba_tight"),
]


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def load(path):
    """Read the CSV into {prefix: [(x, y, yaw_rad), ...]} plus the time column.

    Tracks whose columns are absent are skipped rather than fatal, so a
    recording made by an older node still replays whatever it does contain.
    """
    with open(path, newline="") as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        raise SystemExit("empty recording: %s" % path)
    present = [t for t in TRACKS if (t[0] + "_x") in rows[0]]
    missing = [t[0] for t in TRACKS if t not in present]
    if missing:
        print("[replay] note: not in this recording: %s" % ", ".join(missing))
    data = {p: [] for p, _, _ in present}
    ts = []
    for r in rows:
        try:
            ts.append(float(r["t_mono"]))
        except (KeyError, ValueError):
            ts.append(len(ts) / 50.0)
        for p, _, _ in present:
            try:
                data[p].append((float(r[p + "_x"]), float(r[p + "_y"]),
                                math.radians(float(r[p + "_yaw_deg"]))))
            except (KeyError, ValueError):
                data[p].append((0.0, 0.0, 0.0))
    return present, data, ts


class Replay(Node):
    def __init__(self, args):
        super().__init__("byd_odom_replay")
        self.frame = args.odom_frame
        self.present, self.data, self.ts = load(args.csv)
        self.n = len(self.ts)
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pubs = {p: self.create_publisher(Path, "/byd/path_" + suffix, qos)
                     for p, suffix, _ in self.present}
        self.tf = TransformBroadcaster(self)
        dur = self.ts[-1] - self.ts[0] if self.n > 1 else 0.0
        self.get_logger().info(
            "replaying %s: %d samples, %.0f s, %d tracks"
            % (os.path.basename(os.path.dirname(args.csv)), self.n, dur, len(self.present)))

        if args.realtime:
            self.i = 0
            self.speed = max(0.01, args.speed)
            self.create_timer(1.0 / args.rate, self._step)
        else:
            # Build each Path once; republish on a timer because RViz only sees
            # messages sent after it subscribes, and it may connect late.
            self.msgs = {p: self._path_upto(p, self.n) for p, _, _ in self.present}
            self.create_timer(1.0, self._republish)
            self._republish()

    def _pose(self, x, y, yaw, stamp):
        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = self.frame
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation = yaw_to_quaternion(yaw)
        return ps

    def _path_upto(self, prefix, upto):
        stamp = self.get_clock().now().to_msg()
        m = Path()
        m.header.frame_id = self.frame
        m.header.stamp = stamp
        m.poses = [self._pose(x, y, yaw, stamp)
                   for x, y, yaw in self.data[prefix][:upto]]
        return m

    def _republish(self):
        stamp = self.get_clock().now().to_msg()
        for p, _, child in self.present:
            m = self.msgs[p]
            m.header.stamp = stamp
            self.pubs[p].publish(m)
            if m.poses:
                x, y, _ = self.data[p][-1]
                self._send_tf(child, x, y, self.data[p][-1][2], stamp)

    def _send_tf(self, child, x, y, yaw, stamp):
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.frame
        tf.child_frame_id = child
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation = yaw_to_quaternion(yaw)
        self.tf.sendTransform(tf)

    def _step(self):
        if self.i >= self.n:
            self.get_logger().info("replay finished — holding final paths")
            self.i = self.n
            self._hold()
            return
        # Advance to wherever we should be by now, so --speed works without
        # assuming the timer fires at exactly the recorded sample rate.
        self.i = min(self.n, self.i + max(1, int(round(self.speed))))
        stamp = self.get_clock().now().to_msg()
        for p, _, child in self.present:
            m = self._path_upto(p, self.i)
            self.pubs[p].publish(m)
            x, y, yaw = self.data[p][self.i - 1]
            self._send_tf(child, x, y, yaw, stamp)

    def _hold(self):
        for p, _, _ in self.present:
            self.pubs[p].publish(self._path_upto(p, self.n))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", help="path to a recorded odom.csv")
    ap.add_argument("--realtime", action="store_true",
                    help="animate in time order instead of publishing it all at once")
    ap.add_argument("--speed", type=float, default=1.0,
                    help="realtime playback speed multiplier (default 1.0)")
    ap.add_argument("--rate", type=float, default=50.0,
                    help="realtime tick rate, Hz (default 50)")
    ap.add_argument("--odom-frame", default="odom")
    argv = [a for a in sys.argv[1:] if not a.startswith("__")]
    args, _ = ap.parse_known_args(argv)
    if not os.path.isfile(args.csv):
        raise SystemExit("no such recording: %s" % args.csv)

    rclpy.init()
    node = Replay(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
