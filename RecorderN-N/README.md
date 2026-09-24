# Odom_ROS — snapshot 2026-09-24

Two things are new here:

1. **An end-to-end dataset recorder** that runs entirely on the Kommu device:
   road camera + CAN steering angle, wheel speed and yaw sensor, one rosbag2
   (mcap) bag per session, on the device's own clock. It shares nothing at
   runtime with `odom_node.py` / `byd_drive.sh`.
2. **Track A extracted into `track_a_core.py`**, a stdlib-only module that the
   live `odom_node.py` now imports, and that offline tools use to compute
   position/heading from recorded sessions. One copy of the math, no drift.

First snapshot on the new laptop (Jetson, Ubuntu 20.04, ROS 2 Jazzy in Docker).
Supersedes `../15_09/`.

```
DEVICE (no ROS)                               LAPTOP
byd_e2e_recorder.py  <-- NEW                  byd_record_session.sh  <-- NEW
  VisionIPC road camera -> JPEG                 ssh: start (detached)
  `can` socket -> CAN 287 / 496 / 546           Ctrl-C -> ssh: SIGTERM, md5 list
  -> /data/kommu_tools/e2e_sessions/<id>/       scp -> ~/Desktop/ROSbag/end-end/<id>/
     bag/ session.json README.txt                     (md5-verified)

byd_cereal_server.py   unchanged              byd_odom_ros (odom_node.py)  <-- CHANGED
  --TCP/JSON (5556)-->                          imports track_a_core.py      <-- NEW
```

Current md5s: `odom_node.py` **`57aa7112`**, `track_a_core.py` **`7bd9b49c`**,
`byd_e2e_recorder.py` **`be95bd5a`** (identical on the device). The SSD archive
of 2026-09-16 still holds `odom_node.py` `b999a317`; this code is not archived yet.

---

## What changed since 15_09

### 0. Carried over from 2026-09-16 (never snapshotted separately)

Between 15_09 and the migration, the EKF gained an 8th state `b_a` (residual
accel bias) and was split into two tracks that differ only in the phase-3 b_a
random-walk rate: `/byd/*_ekf_ba_slow` (0.008) and `/byd/*_ekf_ba_tight`
(0.001). b_a phase 2 exits on time only, 10 s after first motion. That work
(`odom_node.py` `b999a317`) touched `byd_drive.sh`, `byd_odom_replay.py`, the
launch file, the RViz config and `test_ekf_fused.py` (now 94 tests). It is in
this snapshot unchanged; it is **not** re-documented here.

### 1. `track_a_core.py` — the shared Track A module

`laptop-side/byd_odom_ros/byd_odom_ros/track_a_core.py`, `import math` only:

| name | what |
|---|---|
| `Integrator` | moved verbatim from `odom_node.py`; heading first, then x/y with the new heading |
| `speed_gate(v)` | 0 below `MIN_SPEED_MS = 0.05` |
| `advance_clock(last_t, t)` | `(new_last_t, dt)`; `dt` None on first sample, `dt <= 0` or `dt > DT_MAX_S = 1.0` |
| `is_stale(now, t, stale_s)` | `STALE_S = 0.5` default |
| `sanitize_can_sample(rate, offset, ok)` | **the single input check**: ok only if the sensor says valid and both values are finite |
| `yaw_rate_offset_corrected`, `standstill_gate`, `track_a_yaw_rate` | Track A yaw: `rate - offset`, 0 without a valid sample, 0 at standstill |

The math is deliberately unguarded; validation lives only in
`sanitize_can_sample()`. No gear logic (the recorder assumes forward).

`odom_node.py` now imports all of this instead of keeping its own copy. Its
behaviour is unchanged for every input ever recorded; for non-finite yaw input
(never seen) Track A/B now hold heading and the EKF skips `z_r`, instead of
propagating NaN. When loaded by file path (the tests), it falls back to
importing `track_a_core` from its own directory.

### 2. `byd_e2e_recorder.py` — device side

| topic | source | type |
|---|---|---|
| `/kommu/road_camera/compressed` | VisionIPC road camera, same JPEG path as `byd_road_camera_rosbag_recorder.py` (imported, not copied) | `sensor_msgs/CompressedImage` |
| `/kommu/can/steer_module_2` | CAN 287 `STEER_MODULE_2.STEER_ANGLE_2`, bus 0 | `kommu_msgs/CanFrameDecoded` |
| `/kommu/can/wheel_speed` | CAN 496 `WHEEL_SPEED`, 4 wheels, raw km/h | same |
| `/kommu/can/yaw_sensor` | CAN 546 `YAW_SENSOR`, `YAW_RATE` + `YAW_OFFSET` | same |

`CanFrameDecoded` = header, address, bus, `can_valid`, all DBC signals of the
frame (decoded by opendbc `CANParser`, as `byd_cereal_server.py` does) and the
raw payload bytes. Its definition is embedded in the bag.

Why these choices:

- **Never subscribes to `carState`.** Its msgq socket has 15 reader slots and
  14 are taken by openpilot daemons; a 16th subscriber evicts every reader,
  controlsd included. Slots of exited subscribers are only freed when `card`
  restarts. Everything comes from `can` instead, and the recorder refuses to
  start if `can` already has `--max-can-readers` (12) readers.
- **Steering angle is `STEER_MODULE_2`**, the exact source of
  `carState.steeringAngleDeg` on this car: fingerprint `BYD_SEAL` ->
  `cam_lka/carstate.py:109`. Not `STEERING_MODULE_ADAS` (0x1E2), which is the
  ADAS steering *command*.
- **Gear is not recorded; forward is assumed**, stated in every session README.
- **One clock.** Camera `timestamp_eof` and CAN batch `logMonoTime` + one epoch
  offset taken at start. BOOTTIME−MONOTONIC is logged at start and stop.
- **Disk.** Refuses to start below `--min-free-mb` (500). Re-checks every 5 s
  and below `--stop-free-mb` (300) closes the bag cleanly and writes
  `!!STOPPED_EARLY_LOW_DISK!!.txt` at the top of the session folder.
- **Stop.** SIGTERM/SIGINT close the bag within ~1 s. Single instance
  (`/tmp/byd_e2e_recorder.lock`), pid in `/tmp/byd_e2e_recorder.pid`.
- **CPU.** Pinned to cores 0,1,2,5 at nice 10: off pandad (3), the control
  loop (4), camerad (6, normal priority) and modeld (7).

Each session folder: `bag/`, `session.json` (settings, `carFingerprint`,
`wheelSpeedFactor`, clock checks, counts), `README.txt`, `recorder.log`,
`.complete` after a clean close.

### 3. `byd_record_session.sh` — laptop side

`./byd_record_session.sh <device-ip> [recorder args]`. Touches the device
three times: one ssh to verify md5s and start detached, one ssh on Ctrl-C to
SIGTERM that recorder and list md5s, one scp, then every file is md5-verified.
If the stop ssh fails the recorder keeps recording and stops itself on low
disk. Never calls `byd_drive.sh`; nothing is deleted from the device.

### 4. Offline position/heading — the accepted approach

Written into every session's `README.txt`, so it travels with the data:

```
v   = speed_gate(mean(WHEELSPEED_FL, _FR, _BL, _BR) / 3.6 * wheelSpeedFactor)
r   = track_a_yaw_rate(YAW_RATE, YAW_OFFSET, can_valid, v)
dt  = advance_clock(last_t, stamp)
Integrator.step(v, r, dt)          # forward gear assumed
```

- One step per `WHEEL_SPEED` frame. Yaw: the `YAW_SENSOR` frame with the exact
  same stamp (same pandad batch) if present, else the most recent one at or
  before it, never a future one; no yaw yet -> skip that frame.
- `wheelSpeedFactor` is **0.6336** on this car; raw wheel speeds are wrong
  without it.
- **openpilot's KF1D speed filter is deliberately not reproduced.** It runs at
  100 Hz on 50 Hz wheel frames with carried state, so it cannot be made
  bit-identical from recorded frames. Positions are close to, not identical
  with, the live node's Track A.

---

## Verification (2026-09-24)

- `tests/test_track_a_core.py`: **32 / 32**. Replays all 72 recordings
  (798,847 rows) through the module: speed gate and yaw rate match
  **exactly** on every row; pose within **1.02e-12** (68.8% bit-exact; the
  rest is the first row's yaw seeded from degrees). Run with
  `ODOM_RECORD_DIR=~/Desktop/Kommu.AI/Odom_record`.
- `tests/test_ekf_fused.py`: **94 / 94** against the new `odom_node.py`.
- colcon build clean in the `byd-jazzy` container; `src/`, `install/` and the
  repo all `57aa7112` / `7bd9b49c`; the installed node's `Integrator` is the
  module's object.
- On-device smoke test (onroad, Park, not engaged), 60 s through the wrapper:
  all 13,073 messages deserialize with `rosbags`, `header.stamp == log_time`
  on every one; steer **99.97 Hz**, wheel/yaw **49.99 Hz**, camera 6.51 Hz;
  topics start within 5 ms of each other; `can_valid` 100%; clock gap ~−2 µs.
- Forced low disk: stopped at the first check, marker written, bag readable.
- Manual SIGTERM: exited in ~2 s, bag complete.

---

## Open items — found 2026-09-24

- **Every session leaves one dead `can` reader slot** until `card` restarts
  (car off/on or reboot). The recorder refuses at 12.
- **`carState` has one free slot**, which `byd_cereal_server.py` takes. After
  that any new subscriber, including a cereal-server restart, evicts all its
  readers. `manual_steer_dryrun*.py` also subscribe to it.
- **`byd_drive.sh` detects a stale node by `odom_node.py`'s ctime only.** A
  change to `track_a_core.py` alone will not restart a running node.
- **`/data` fell ~10 MB/min while onroad** before one reboot (bukapilot's own
  logging, not the recorder). One-bag sessions are sized by the worst case.
- `run_all.sh` "decoder round-trip" and "carcontroller gating" fail in the
  container: no `pycapnp`. Unrelated to this code.
- `CarName` says "BYD Dolphin 2023-26" but the fingerprint is `BYD_SEAL`
  (Seal vehicle params); `odom_node` uses a 2.70 m wheelbase.

---

## Layout

```
24_09/
  device-side/
    byd_e2e_recorder.py                 NEW  be95bd5a
    byd_road_camera_rosbag_recorder.py  f266c44b, imported by the recorder
    byd_cereal_server.py                unchanged 97a64cd1
    carstate.py                         unchanged f8a97e6b (= live device)
    byd_yaw_sensor_probe.py             unchanged
  laptop-side/
    byd_record_session.sh               NEW
    byd_odom_ros/                       odom_node.py CHANGED, track_a_core.py NEW
                                        (launch, rviz: 09-16 versions)
    byd_drive.sh, byd_odom_replay.py    09-16 versions
    byd_ensure_cereal_server.sh, byd_rviz.sh, byd_replay.sh,
    byd_yawcheck.sh, byd_odom_plot.py   unchanged
    tests/                              + test_track_a_core.py
    analysis/, figures/                 unchanged since 10_09
```

## Deploying this snapshot

```bash
# laptop (inside the byd-jazzy container)
rsync -a laptop-side/byd_odom_ros/ ~/ros2_ws/src/byd_odom_ros/
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --packages-select byd_odom_ros
python3 laptop-side/tests/test_ekf_fused.py        # expect 94 passed, 0 failed
ODOM_RECORD_DIR=~/Desktop/Kommu.AI/Odom_record python3 laptop-side/tests/test_track_a_core.py   # 32 passed

# device recorder
scp device-side/byd_e2e_recorder.py device-side/byd_road_camera_rosbag_recorder.py kommu@<device-ip>:/data/kommu_tools/
cp laptop-side/byd_record_session.sh ~/Desktop/Kommu.AI/claude/
./byd_record_session.sh <device-ip>                # Ctrl-C to stop and pull
```
