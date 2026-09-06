# Odom_ROS — live odometry visualisation for the BYD Dolphin

Laptop-side live dead-reckoning odometry, rendered in RViz2. The Kommu device
runs **no ROS**; this bridges to it over the device's existing
`byd_cereal_server.py` TCP/JSON stream (port 5556). The ROS2 node consumes that
stream, integrates a bicycle model, and publishes `nav_msgs/Odometry` +
`nav_msgs/Path` + TF.

Full background, the measurement evidence behind the corrections, and the known
limitations are in the main **`CONTEXT.md` → `## ODOMETRY / ROS2 VISUALIZATION`**
section. This README is the setup-and-run guide only; it does not repeat that.

```
DEVICE (no ROS)                    LAPTOP (ROS2 Jazzy)
byd_cereal_server.py --TCP/JSON--> byd_odom_ros (odom_node.py)
  (port 5556)                        --> /byd/odom_corrected, /byd/path_corrected
                                     --> /byd/odom_measured,  /byd/path_measured
                                     --> TF: odom -> base_link_{corrected,measured}
                                     --> RViz2
```

---

## Contents

```
laptop-side/
  byd_odom_ros/                 the ROS2 package (ament_python)
  byd_drive.sh                  one-command launcher (ensures device server, then launches)
  byd_ensure_cereal_server.sh   device-side pre-flight: check / restore / start / verify
device-side/
  byd_cereal_server.py          the TCP/JSON bridge that runs ON the Kommu device
```

`byd_drive.sh` calls `byd_ensure_cereal_server.sh` **from its own directory**
(`$SCRIPT_DIR`), so keep those two files together wherever you put them.

---

## One-time laptop setup

**1. ROS2 Jazzy.** Install `ros-jazzy-desktop` — **not** `-base`, which omits
RViz2 — plus colcon. Follow the official ROS2 docs for Ubuntu 24.04:

```bash
sudo apt install ros-jazzy-desktop python3-colcon-common-extensions
```

**2. Build the package into a colcon workspace.**

```bash
mkdir -p ~/ros2_ws/src
cp -r laptop-side/byd_odom_ros ~/ros2_ws/src/
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select byd_odom_ros
```

**3. Put the two scripts somewhere convenient** (together — see above).

> ⚠️ **`byd_drive.sh` hardcodes `$HOME/ros2_ws/install/setup.bash`.** If you use
> a different workspace path, edit that `source` line (near the top of the
> script) or the launch step will fail with "package not found". The
> `/opt/ros/jazzy/setup.bash` line assumes a standard Jazzy install.

---

## One-time device setup

Requires SSH access to the Kommu device (`kommu@<device-ip>`). Without it you
can still read and edit all the code here, but you cannot run a live test —
the ROS node has nothing to connect to.

The server must exist in **two** places:

| Path | Why |
|---|---|
| `/data/openpilot/byd_cereal_server.py` | where it actually runs from |
| `/data/kommu_tools/byd_cereal_server.py` | master copy — survives the updater tree-swap |

That second copy is not optional housekeeping. When the device updater
finalises an update, the next boot **replaces `/data/openpilot` wholesale** and
every untracked file in it is lost. `/data/kommu_tools/` sits outside that blast
radius, and `byd_ensure_cereal_server.sh` restores from it automatically. See
CONTEXT.md's OPERATIONAL GOTCHAS for the full mechanism.

```bash
IP=<device-ip>
ssh kommu@$IP "mkdir -p /data/kommu_tools"
scp device-side/byd_cereal_server.py kommu@$IP:/data/kommu_tools/
scp device-side/byd_cereal_server.py kommu@$IP:/data/openpilot/
```

Start it (normally you never need this by hand — `byd_drive.sh` does it):

```bash
ssh kommu@$IP "cd /data/openpilot && screen -dmS cereal bash -c \
  'PYTHONPATH=/data/kommu_tools/pylibs:/data/openpilot \
   /usr/local/venv/bin/python3 -u byd_cereal_server.py > /tmp/cereal_server.log 2>&1'"
```

The venv path and `PYTHONPATH` both matter — the system Python cannot import
`cereal` (missing `capnp`). Verify:

```bash
ssh kommu@$IP "timeout 2 nc localhost 5556 | head -1"
```

You should get one line of JSON containing `yaw_rate` and `angle_offset_deg`.

---

## Running, every session

### Preferred — one command, self-healing

```bash
./byd_drive.sh <device-ip>
```

**The IP is a positional argument, not `host:=<ip>`.** Passing `host:=1.2.3.4`
would be taken literally as the hostname and fail at the SSH step. Extra
arguments after the IP pass straight through to `ros2 launch`:

```bash
./byd_drive.sh                          # defaults to 172.20.10.3
./byd_drive.sh 192.168.1.42             # different device IP
./byd_drive.sh 192.168.1.42 rviz:=false # headless, node only
./byd_drive.sh 192.168.1.42 corrected_steer_ratio:=14.5
```

This checks the device server, restores it from `/data/kommu_tools/` if the
updater wiped it, starts it if it isn't running, verifies the stream actually
responds, then sources both ROS overlays and launches. It is idempotent — safe
to run repeatedly, and it will not start a second server.

### Manual fallback

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch byd_odom_ros odom_rviz.launch.py host:=<device-ip>
```

Note the launch file **does** use `host:=` — that's ROS launch syntax, distinct
from `byd_drive.sh`'s positional argument. Easy to mix up.

This route does **not** self-heal the device server. If the node sits printing
`device stream lost (...); retrying in 2s`, the server is down or missing — run:

```bash
./byd_ensure_cereal_server.sh <device-ip>
```

which also takes the IP positionally.

### Finding the device IP

It is dynamic and changes per network (home WiFi vs phone hotspot). Get it from
the KommuAI app, or scan: `nmap -sn 192.168.1.0/24` (adjust the subnet).

---

## What's live, what isn't

| Track | Topics | RViz | Status |
|---|---|---|---|
| **corrected** | `/byd/{odom,path}_corrected` | orange | **active, use this** — steer-derived, `steer_ratio=14.2`, steering centre-offset removed |
| measured | `/byd/{odom,path}_measured` | green | publishes at 50 Hz but is **structurally flat** — see below |
| kinematic | — | (disabled) | **deprecated**, commented out in source, does not publish |

**The measured track draws a straight line and that is expected.** It integrates
the car's own `yawRate`, which is declared in the cereal schema but never
assigned anywhere in the BYD car port — it reads `0.0` forever. Integrating zero
yaw means the heading never changes. This is a gap in the car port, not a bug
here. Do not spend time debugging the ROS package over it.

**Do not re-enable the kinematic track** in `byd_drive.sh` or any launch config.
It is the original uncorrected model (`steer_ratio=13.11`, no offset removal)
that these two corrections replaced. The code is commented out rather than
deleted so the old behaviour can be reproduced for comparison — there is a
block comment in `odom_node.py`'s `__init__` explaining exactly why.

---

## Known open items

Detail for all of these is in CONTEXT.md; listed here so nobody rediscovers them
the hard way.

- **No yaw-rate signal.** As above. Closing it needs either a real yaw-rate CAN
  signal (not yet found on this car) or a differential wheel-speed estimate.
- **Wheel-speed differential is viable but unattempted.** All four wheel speeds
  *are* decoded from `WHEEL_SPEED` (ID 496), but the generic
  `parse_wheel_speeds()` averages them into `vEgoRaw` and never fills
  `carState.wheelSpeeds` — so they're discarded before reaching cereal. Needs
  ~4 lines in `carstate.py`, plus the Dolphin's rear track width (~1.53 m,
  **unverified**), and is noise-sensitive at low speed.
- **`corrected_steer_ratio = 14.2` is a midpoint estimate, not a fitted value.**
  It comes from a prior 14.1–14.3 range and was not re-fit. A sweep
  (`corrected_steer_ratio:=14.0 / 14.5` over the same repeated loop) would
  tighten it.
- **Path publishing is amortised, not fixed.** Republishing the whole `Path`
  costs O(number of poses). `--path-publish-hz` (default 10) decouples it from
  the 50 Hz tick, a ~4.4x reduction — but the underlying scaling remains, and
  individual publish ticks still spike. Fine for drives of minutes; not for
  indefinite unattended logging.
