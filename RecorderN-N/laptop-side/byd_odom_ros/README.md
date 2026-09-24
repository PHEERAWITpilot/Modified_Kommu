# byd_odom_ros

Live bicycle-model odometry for the BYD Dolphin, visualised in RViz2.

## Architecture — why there's a bridge

The Kommu device runs **no ROS**, and its odometry inputs live in cereal, which
isn't reachable over the network. So this package is **laptop-side only**:

```
DEVICE (Kommu, no ROS)                     LAPTOP (ROS2 Jazzy)
----------------------                     -------------------
bukapilot / cereal
    |
    v
byd_cereal_server.py  --- TCP/JSON --->    odom_node
(existing project tool,                        |
 port 5556, emits yaw_rate +           +--> /byd/odom_measured    nav_msgs/Odometry
  angle_offset_deg)                            +--> /byd/path_measured    nav_msgs/Path
                                               +--> /byd/odom_corrected   nav_msgs/Odometry
                                               +--> /byd/path_corrected   nav_msgs/Path
                                               +--> TF: odom -> base_link_{measured,kinematic}
                                                        |
                                                        v
                                                      RViz2
```

Nothing is deployed to the device and nothing about it changes. It runs the
cereal TCP server this project already built and validated, exactly as-is.

## One-time setup (laptop)

Install ROS2 Jazzy per the official instructions, then:

```bash
mkdir -p ~/ros2_ws/src
cp -r byd_odom_ros ~/ros2_ws/src/
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select byd_odom_ros
```

> ⚠️ Do **not** install ROS2 into the `.venv` used elsewhere in this project.
> ROS2 needs the system Python. Keep the two environments separate — this is
> the same class of problem as the `rosbags`/numpy-2 pinning conflict already
> hit on the device.

## Running (every session)

**1. Device — start the cereal server** (if not already running):
```bash
ssh kommu@<device-ip> "cd /data/openpilot && screen -dmS cereal bash -c 'python3 -u byd_cereal_server.py > /tmp/cereal_server.log 2>&1'"
```

**2. Laptop — launch node + RViz together:**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch byd_odom_ros odom_rviz.launch.py host:=<device-ip>
```

The device IP is **dynamic** (home WiFi vs phone hotspot). Find it via the
KommuAI app or `nmap -sn <subnet>/24`. Don't rely on the default.

### Useful variants
```bash
# node only, no GUI
ros2 launch byd_odom_ros odom_rviz.launch.py host:=<ip> rviz:=false

# experiment with the kinematic steer ratio (see limitation below)
ros2 launch byd_odom_ros odom_rviz.launch.py host:=<ip> steer_ratio:=14.2

# change how often the Path trails are republished (default 10 Hz)
ros2 launch byd_odom_ros odom_rviz.launch.py host:=<ip> path_publish_hz:=5.0
```

`path_publish_hz` is decoupled from `rate` on purpose. A `nav_msgs/Path` carries
its entire history, so republishing it costs O(number of poses) — at the full
50 Hz tick rate the callback grew past its 20 ms budget once the trail passed
~4300 poses and the whole node throttled itself. Poses are still **appended
every tick**, so no positional resolution is lost; only the republish is
throttled. `/byd/odom_*` and TF stay ungated at the full tick rate.

### Verify it's working
```bash
ros2 topic hz /byd/odom_measured   # expect ~50 Hz  (live position, ungated)
ros2 topic hz /byd/odom_corrected  # expect ~50 Hz
ros2 topic hz /byd/path_corrected  # expect ~10 Hz  (= path_publish_hz)
ros2 run tf2_tools view_frames     # confirms odom -> base_link_{measured,kinematic}
```

## Active tracks — and one deprecated

Two tracks are published:

| Track | Heading source | Topics | RViz |
|---|---|---|---|
| **corrected** | steer angle, `steer_ratio=14.2`, centre offset removed | `/byd/*_corrected` | orange |
| measured | the car's `yawRate` via cereal | `/byd/*_measured` | green |

**The raw kinematic track is DEPRECATED and no longer published.** It was the
original bicycle-model integrator at `steer_ratio=13.11` with no offset
correction. It over-predicts yaw by ~8% (switching to 14.2 alone moves the
estimate -7.7%) and ignored a steering centre-offset measured live at
**1.537°** via `liveParameters.angleOffsetAverageDeg` — a further -5.1% at 30°
steer, and strongly steer-dependent (-31% at 6°, -9% at 90°), which is the
signature of a constant bias and shows up as curvature error on gentle
steering rather than a uniform scale error.

The code is **commented out, not deleted** (see the block in `odom_node.py`'s
`__init__`), so the old behaviour can be reproduced for comparison. Its RViz
displays are kept but set `Enabled: false`. `--steer-ratio` still exists and
feeds only that disabled path.

⚠️ **The measured track is currently flat.** `yawRate` is never assigned
anywhere in the BYD port, so it reads `0.0` forever and the track integrates a
perfectly straight line. Per-wheel speeds *are* decoded from `WHEEL_SPEED`
(ID 496) but `parse_wheel_speeds()` averages them into `vEgoRaw` and never
fills `carState.wheelSpeeds`, so a differential-wheel-speed yaw estimate is
possible but needs wiring first. See "Known limitation" below.

## Constants — deliberately NOT the SEAL placeholders

`CAR.BYD_SEAL` CarSpecs are Seal values, not Dolphin values. This package
overrides them:

| | This package | SEAL placeholder |
|---|---|---|
| wheelbase | **2.70 m** | 2.92 m |
| steer ratio | **13.11** | 16.0 |

## Known limitation (carried over, not fixed here)

Validation of the underlying model against `livePose` showed **r = 0.985**
correlation but an **~8% yaw over-prediction** at `steer_ratio = 13.11`,
implying the effective *kinematic* ratio is nearer **14.1–14.3** — 13.11 is a
control-tuned value from liveParameters, not a pure geometric one. Position
drift runs ~8–12% of path length over 3–5 minute open-loop windows: bounded,
not runaway, and consistent with the yaw bias compounding.

This was **deliberately left uncorrected**. Use `steer_ratio:=14.2` to try the
fitted value; if it visibly improves closure on a known loop, that's the
empirical odometry-specific ratio worth adopting — kept separate from the 13.11
used for steering control.

> Methodology note: if re-validating, use **20 Hz `livePose`** as ground truth,
> **not** 1 Hz GPS bearing. An earlier attempt using GPS bearing produced a
> misleading 30–40% under-prediction due to regression attenuation from bearing
> noise.

## ⚠️ Unverified assumptions — confirm before trusting live output

1. **JSON field names.** This node assumes `byd_cereal_server.py` emits
   `v_kmh`, `steer_deg`, and `yaw_rate`. The server's State dict is known to
   carry `v_kmh` and `steer_deg`; **`yaw_rate` must be confirmed present.** If
   it isn't, add it server-side (one line reading `cs.yawRate`, matching what
   `byd_route_to_csv.py` already does). **Done 2026-08-24** — the server now emits
   it; the node logs a one-time warning if it ever goes missing, and the measured
   track then holds heading rather than silently falling back.
2. **Wire format.** Assumes newline-delimited JSON objects over TCP.
3. **Port.** Assumes 5556. The project notes 5555/5556 as the CAN/cereal server
   ports and 5557 as the manual-steer writer — confirm which is which.

Check quickly with:
```bash
ssh kommu@<ip> "timeout 2 nc localhost 5556 | head -3"
```

## Scope

- Read-only. Subscribes to a device stream; transmits nothing, touches no CAN,
  never opens the Panda.
- Independent of the steering tool and the longitudinal work. No shared state,
  no shared files.
- Open-loop dead reckoning only — no GPS/map fusion, no loop closure. Drift is
  expected and quantified above.
