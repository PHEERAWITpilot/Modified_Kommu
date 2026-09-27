# Modified_Kommu

Modifications and instrumentation for the Kommu / bukapilot BYD Dolphin port.

| Folder | What it holds |
|---|---|
| `Odom_ROS/` | Laptop-side dead-reckoning odometry in RViz2, as dated snapshots (`29_08/`, `06_09/`). Start with the newest snapshot's README. |
| `RecorderN-N/` | On-device end-to-end dataset recorder (road camera + CAN steering, wheel speed and yaw, one rosbag2 per session, never subscribes to `carState`), its laptop wrapper (`byd_record_session.sh`, or `byd_drive.sh --with-recorder` to run it together with the odom node), and `track_a_core.py`, the shared Track A math that `odom_node.py` now imports. Snapshot of 2026-09-24, `--with-recorder` added 2026-09-27; see its README. |
| `ManualSteering60deg/` | Manual desired-angle substitution at the ±45°/±60° tier, on 70° Panda firmware. |
| `LongitudinalCapture_20260819/` | ACC_CMD (0x32E) injection harness and the evidence that injection alongside the live factory ACC ECU is non-viable. |

## Related repositories

**RTK** — the ZED-F9R / NTRIP-VRS positioning stack (`start_rtk.sh`,
`rtk_overlay_node.py`, `rtk_status.py`, the EKF config and the track tooling)
lives in its own repository at **github.com/PHEERAWITpilot/RTK**, because it has
its own commit history.

A working copy may sit at `RTK/` in this tree. It is **deliberately
gitignored** — nesting one git repo inside another makes git record only a
*gitlink*, a bare pointer to a commit this repo does not contain, so `RTK/`
would clone as an empty directory. Clone it separately instead:

```bash
git clone git@github.com:PHEERAWITpilot/RTK.git
```
