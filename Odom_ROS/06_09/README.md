# Odom_ROS — snapshot 2026-09-06

Laptop-side live dead-reckoning odometry for the BYD Dolphin, rendered in
RViz2. The Kommu device runs **no ROS**; this bridges to it over the device's
`byd_cereal_server.py` TCP/JSON stream (port 5556).

Supersedes `../29_08/`. Full background and the measurement evidence are in the
main **`CONTEXT.md` → `## ODOMETRY / ROS2 VISUALIZATION`** section — this README
covers what changed and how to run it.

```
DEVICE (no ROS)                    LAPTOP (ROS2 Jazzy)
byd_cereal_server.py --TCP/JSON--> byd_odom_ros (odom_node.py)
  (port 5556)                        --> /byd/odom_corrected   + path + TF
  + yaw_sensor_rate                  --> /byd/odom_startref    + path + TF
  + angle_offset_deg                 --> /byd/odom_steerproxy  + path + TF
                                     --> /byd/odom_deltaref    (published, hidden)
                                     --> /byd/odom_yawsensor   (published, hidden)
                                     --> RViz2
```

---

## What changed since 29_08

**Five odometry tracks** (29_08 had two). All use `steer_ratio = 14.2` except
where noted, so differences between lines are the method, not the ratio.

| Track | RViz | Method |
|---|---|---|
| `corrected` | orange `255;170;0` | live `angle_offset_deg` removed + ratio 14.2 |
| `startref` | white `220;220;220` | raw uncorrected steer (see below) |
| `steerproxy` | red `230;30;30` | tire angle used DIRECTLY as heading, no integration |
| `deltaref` | hidden | delta reconstruction, zeroed at startup |
| `yawsensor` | hidden | the car's own CAN yaw sensor — no ratio, no offset |

Only the first three are enabled in `rviz/byd_odom.rviz`. **Hidden ≠ not
computed** — all five still integrate and publish at full rate.

**Two tracks are known-negative by construction, and were proven so numerically
before driving:**

- `startref` **is** the raw steering angle. Its delta sum telescopes to
  `steer(t)`. Proven sample-by-sample over 55,522 real samples: max difference
  **1.78e-15 deg**, i.e. float noise, 5.6e13x below the sensor's own 0.1 deg
  resolution. See `tests/test_startref_equivalence.py`.
- `steerproxy` carries no turn history. Over 30 straight-after-turn segments in
  the same log it averaged **+0.06 deg** while the accumulated heading averaged
  **+530.3 deg**. It is bounded by `max_steer/ratio` and can never represent a
  full turn.

**New device-side signal.** `byd_cereal_server.py` now also emits
`yaw_sensor_rate` / `yaw_sensor_offset` / `yaw_sensor_ok`, decoded from
**CAN 546 `YAW_SENSOR`** via opendbc's own `CANParser`. This is the car's real
angular-rate sensor — no steer ratio, no centre offset, no bicycle model.
Validated on a drive: sign matched steering 45/45 samples, ~0 on straights,
22 deg/s peak at a tight car-park turn.

> Note this is distinct from `cs.yawRate`, which the BYD port **never assigns**
> and which therefore reads 0.0 forever — that is why the old `measured` track
> drew a straight line.

**Two reliability guards** (see "Guards" below) — both added after real
failures, both verified on-device.

---

## Contents

```
laptop-side/
  byd_odom_ros/                    the ROS2 package (ament_python)
  byd_drive.sh                     one-command launcher + duplicate-node guard
  byd_ensure_cereal_server.sh      device pre-flight: check / restore / restart / verify
  analysis/
    analyze_yaw_lag.py             yaw-vs-model lag & gain, per steering event
    analyze_yaw_offset_drift.py    is YAW_OFFSET a constant or drifting?
    fit_steering_calibration.py    closed-loop fit of (offset, steer_ratio)
  tests/
    test_startref_equivalence.py   proves startref == raw steer
    test_deltaref_reconstruction.py  bias cancellation + the drift tradeoff
device-side/
  byd_cereal_server.py             TCP/JSON bridge (runs ON the device)
  byd_yaw_sensor_probe.py          read-only CAN 546 probe, --log for full-rate JSONL
```

`byd_drive.sh` calls `byd_ensure_cereal_server.sh` **from its own directory**
(`$SCRIPT_DIR`) — keep those two together.

---

## Guards — read this before debugging odometry

**1. Duplicate node → jumping path.** Two `odom_node` instances each hold their
own Integrator state while publishing to the same topics; RViz interleaves two
diverging trajectories and the path appears to jump. This is invisible to
`ros2 topic hz` and `ros2 topic list`, and it silently corrupted three separate
measurements on 2026-09-05 before being spotted.

> **The diagnostic that works:**
> `ros2 topic info /byd/odom_corrected` → **`Publisher count` must be 1.**
> Check this FIRST, before investigating any maths.

Fixed in two layers: `byd_drive.sh` refuses and prints the offending pid
(`--kill-existing` replaces, `--allow-multiple` overrides), and `odom_node.py`
holds an `fcntl.flock` on `/tmp/byd_odom_node.lock` taken **before**
`rclpy.init()`. The lock is the layer that matters — `ros2 launch`, `ros2 run`
and IDEs all bypass the wrapper. `flock` frees on process death, so a crash
cannot wedge it.

**2. Stale device server.** `byd_ensure_cereal_server.sh` previously checked
only that the file existed and a process ran — never that either was current.
It now compares live-vs-master md5 AND process-start-time vs file-mtime
(Python caches modules at import, so a process older than the file serves stale
code), and restarts automatically. ~2 s stream gap, announced. Verified
idempotent: three consecutive runs leave the same pid.

---

## Running

```bash
./byd_drive.sh 172.20.10.2                    # refuses if a node is already up
./byd_drive.sh 172.20.10.2 --kill-existing    # replace the running one
./byd_drive.sh 172.20.10.2 rviz:=false        # headless
```

The IP is **positional**, not `host:=`. The launch file uses `host:=`; the
wrapper does not. Easy to mix up.

Manual fallback (does **not** self-heal the device server, but IS covered by
the node's flock guard):

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch byd_odom_ros odom_rviz.launch.py host:=<ip>
```

Device IP is dynamic — get it from the KommuAI app or `nmap -sn <subnet>/24`.

### Capturing data

```bash
ssh -t kommu@<ip> "bash -lc 'cd /data/openpilot && python3 byd_yaw_sensor_probe.py --seconds 600 --log'"
```

`-t` is **required** — without a pty, `isatty()` is false, markers are disabled,
and a whole drive's checkpoints are lost. Confirm the startup line reads
`MARKERS ENABLED`, not `markers disabled`, before driving. Each marker echoes
`[MARK n] t=... label=...` as live confirmation.

Logs are large; **gzip before transferring** — plain `scp` truncated a 23.8 MB
log twice over the hotspot link, caught only by md5. `gzip -c` cut it to 2.2 MB
and it transferred intact.

---

## Open items

- **Steer ratio is unresolved.** 14.2 is a midpoint estimate, never fitted.
  Two independent low-speed datasets now suggest it *under*-predicts by ~4-7%
  (measured gain 1.04-1.07), which contradicts the prior "13.11 over-predicts
  by 8%". Not a conclusion — confounded by lag, low speed, and uncorrected
  steer. `fit_steering_calibration.py` is ready for a proper closed-loop fit.
- **The 3-loop calibration drive is not usable for that fit.** All six markers
  landed and the data is clean, but the loops were near-identical (duration
  spread 7.0%, speed 7.2%, peak-steer 14.0%), which cannot separate offset from
  ratio. A future drive must deliberately vary speed, loop size and direction.
- **Yaw lag is inconclusive.** Per-event median 0 ms, but the correlation curve
  is a plateau (peak beats zero-lag by 0.0033 on a 0.997 baseline) and speeds
  were 1-4.6 m/s. Needs a higher-speed pulse-test drive.
- **`YAW_OFFSET` is a stable constant, not drifting.** 10 min / 57,361 samples
  stationary: a single value, zero variation, slope 0.0, R² 0.0. But it sits at
  1 LSB (0.122 deg/s) — integrating it uncorrected would add **7.3 deg of false
  heading per minute**, and whether to subtract it is still open.
- **Performance scales with track count.** Path republish is O(poses) per
  track; five tracks cost ~2.5x what two did, so the tick budget is exhausted
  sooner on long drives. Hiding a track in RViz does not stop it publishing.

## Driving conditions matter

Record them. A run where `corrected` looked much worse turned out to be **rain**
— wheel slip means the wheels rotate further than the car travels, so
wheel-speed-derived distance over-reads and the whole trajectory inflates. The
`corrected` code path was verified byte-identical across every version, so this
was never a code regression. A wet run is not comparable with a dry one, and no
ratio or offset fit will reconcile them.
