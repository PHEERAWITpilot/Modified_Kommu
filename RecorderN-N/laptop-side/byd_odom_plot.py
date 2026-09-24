#!/usr/bin/env python3
"""byd_odom_plot.py — static plot of a saved odom.csv. No ROS needed.

For a quick look at a finished drive without starting RViz, and for putting a
run in a report. Colours match rviz/byd_odom.rviz so a plot and the RViz view
of the same drive read the same way.

    python3 byd_odom_plot.py                       # newest recording -> PNG
    python3 byd_odom_plot.py 2026/09/09/1607       # that run
    python3 byd_odom_plot.py path/to/odom.csv -o out.png
    python3 byd_odom_plot.py --tracks trackA_persample,trackB_windowed

By default it plots only the tracks worth comparing. The steer-derived ones
drift by design, and drawing all eight rescales the axes so far that the good
ones collapse into a single line; --tracks all overrides that.
"""

import argparse
import csv
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patheffects as pe

SURFACE = "#fcfcfb"; INK = "#0b0b0b"; INK2 = "#52514e"; GRID = "#e5e4e0"

# prefix -> (label, colour). Colours are the RViz ones, as 0-255 triples there.
TRACKS = {
    "trackA_persample": ("A  per-sample offset",   "#ffff00"),
    "trackB_windowed":  ("B  windowed offset",     "#ff00c8"),
    "yawsensor":        ("yaw sensor, no offset",  "#00dcdc"),
    "corrected":        ("corrected (steer 14.2)", "#ffaa00"),
    "startref":         ("startref (raw steer)",   "#dcdcdc"),
    "steerproxy":       ("steerproxy",             "#e61e1e"),
    "deltaref":         ("deltaref",               "#aa28c8"),
    "measured":         ("measured (cs.yawRate)",  "#19ff64"),
}
DEFAULT = ["trackA_persample", "trackB_windowed", "yawsensor", "corrected"]


def find_csv(target, record_dir):
    if target and os.path.isfile(target):
        return target
    if target and os.path.isfile(os.path.join(target, "odom.csv")):
        return os.path.join(target, "odom.csv")
    if target and os.path.isfile(os.path.join(record_dir, target, "odom.csv")):
        return os.path.join(record_dir, target, "odom.csv")
    if target:
        raise SystemExit("cannot find a recording for: %s" % target)
    best = None
    for root, _, files in os.walk(record_dir):
        if "odom.csv" in files:
            p = os.path.join(root, "odom.csv")
            m = os.path.getmtime(p)
            if best is None or m > best[0]:
                best = (m, p)
    if best is None:
        raise SystemExit("no recordings under %s" % record_dir)
    return best[1]


def list_runs(record_dir):
    """Cross-check INDEX.csv against what is actually on disk.

    INDEX.csv is append-only, so a run that was saved and later deleted still
    has a line here. That is the point: it makes a missing recording visible
    instead of silent.
    """
    index = os.path.join(record_dir, "INDEX.csv")
    if not os.path.isfile(index):
        print("no INDEX.csv in %s (no runs saved yet)" % record_dir)
        return
    with open(index, newline="") as fh:
        rows = list(csv.DictReader(fh))
    missing = 0
    print("%-20s %-18s %8s %9s  %s" % ("saved", "run", "samples", "duration", "status"))
    for r in rows:
        d = os.path.join(record_dir, r["run_dir"])
        ok = os.path.isfile(os.path.join(d, "odom.csv"))
        if not ok:
            missing += 1
        print("%-20s %-18s %8s %9s  %s"
              % (r["saved"], r["run_dir"], r["samples"], r["duration_s"],
                 "ok" if ok else "*** MISSING ***"))
    print("\n%d run(s) indexed, %d missing" % (len(rows), missing))
    if missing:
        print("A missing run was saved once and has since been deleted.")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("target", nargs="?", help="odom.csv, its directory, or YYYY/MM/DD/HHMM")
    ap.add_argument("-o", "--out", help="output PNG (default: alongside the CSV)")
    ap.add_argument("--tracks", default=",".join(DEFAULT),
                    help="comma-separated track prefixes, or 'all'")
    ap.add_argument("--record-dir",
                    default=os.path.expanduser("~/Desktop/Kommu.AI/Odom_record"))
    ap.add_argument("--list", action="store_true",
                    help="list every run in INDEX.csv and flag any whose files "
                         "are gone, then exit without plotting")
    args = ap.parse_args()

    if args.list:
        list_runs(args.record_dir)
        return

    csv_path = find_csv(args.target, args.record_dir)
    with open(csv_path, newline="") as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        raise SystemExit("empty recording: %s" % csv_path)

    want = list(TRACKS) if args.tracks == "all" else [t.strip() for t in args.tracks.split(",")]
    want = [t for t in want if t in TRACKS and (t + "_x") in rows[0]]
    if not want:
        raise SystemExit("none of those tracks are in this recording")

    t0 = float(rows[0]["t_mono"])
    t = [float(r["t_mono"]) - t0 for r in rows]
    v = [float(r["v_kmh"]) for r in rows]
    xy = {p: ([float(r[p + "_x"]) for r in rows],
              [float(r[p + "_y"]) for r in rows],
              [float(r[p + "_yaw_deg"]) for r in rows]) for p in want}

    plt.rcParams.update({
        "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "axes.edgecolor": GRID,
        "axes.labelcolor": INK2, "text.color": INK, "xtick.color": INK2, "ytick.color": INK2,
        "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
        "axes.spines.top": False, "axes.spines.right": False,
        "font.size": 10, "figure.dpi": 130,
    })
    fig = plt.figure(figsize=(13.5, 6.6))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.5, 1], hspace=0.38, wspace=0.20)

    ax = fig.add_subplot(gs[:, 0])
    for p in want:
        x, y, _ = xy[p]
        ax.plot(x, y, color=TRACKS[p][1], lw=1.7, label=TRACKS[p][0],
                path_effects=[pe.Stroke(linewidth=2.9, foreground="#5a5a55"), pe.Normal()])
    ax.plot(0, 0, "o", color=INK, ms=6)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    run = os.path.basename(os.path.dirname(csv_path))
    dur = t[-1] if t else 0.0
    dist = sum(abs(s) / 3.6 * 0.02 for s in v)
    ax.set_title("run %s — %.0f s, ~%.0f m" % (run, dur, dist), fontsize=11, loc="left")
    fig.text(0.008, 0.005, csv_path, fontsize=7, color=INK2)
    ax.legend(loc="best", fontsize=8.5, framealpha=0.92)

    ax2 = fig.add_subplot(gs[0, 1])
    for p in want:
        ax2.plot(t, xy[p][2], color=TRACKS[p][1], lw=1.3,
                 path_effects=[pe.Stroke(linewidth=2.4, foreground="#5a5a55"), pe.Normal()])
    ax2.set_ylabel("heading (deg)"); ax2.set_xlabel("time (s)")
    ax2.set_title("heading", fontsize=10, loc="left")

    ax3 = fig.add_subplot(gs[1, 1])
    ax3.plot(t, v, color="#2a78d6", lw=1.0)
    ax3.set_ylabel("speed (km/h)"); ax3.set_xlabel("time (s)")
    ax3.set_title("speed", fontsize=10, loc="left")

    out = args.out or os.path.join(os.path.dirname(csv_path), "odom.png")
    fig.savefig(out, bbox_inches="tight")
    print("wrote %s" % out)


if __name__ == "__main__":
    main()
