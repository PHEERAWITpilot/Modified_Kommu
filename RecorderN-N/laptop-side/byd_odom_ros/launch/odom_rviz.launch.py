"""
Launch the BYD Dolphin dual-track odometry node together with RViz2.

Publishes TWO independent paths simultaneously for comparison:
  green = measured  (car's own yaw_rate, via cereal)
  blue  = kinematic (derived from steer_deg + speed, bicycle model)

Usage:
    ros2 launch byd_odom_ros odom_rviz.launch.py
    ros2 launch byd_odom_ros odom_rviz.launch.py host:=192.168.1.42
    ros2 launch byd_odom_ros odom_rviz.launch.py steer_ratio:=14.2
    ros2 launch byd_odom_ros odom_rviz.launch.py rviz:=false
    ros2 launch byd_odom_ros odom_rviz.launch.py rviz_delay_s:=5.0

RViz is NOT started together with the node. It opens rviz_delay_s seconds after
the node's FIRST /byd/odom_corrected message, so the EKF session-zero window has
been collecting stationary samples for that long first. RViz opening is the
driver's "ready to drive" signal. If no message arrives within
odom_wait_timeout_s, RViz opens anyway with a warning.

The device IP is DYNAMIC — do not rely on the default. Find it via the
KommuAI app or `nmap -sn <subnet>/24`.

⚠️ The MEASURED (green) track requires byd_cereal_server.py to emit a
"yaw_rate" field. If it doesn't yet, the measured track will sit flat at the
origin (a one-time warning prints in the odom_node log) while the kinematic
(blue) track still moves normally — that is not a bug, it's a visible signal
that the server-side field is still missing.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo,
                            RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("byd_odom_ros")
    default_rviz = os.path.join(pkg_share, "rviz", "byd_odom.rviz")

    args = [
        DeclareLaunchArgument("host", default_value="172.20.10.2",
                              description="Kommu device IP (DYNAMIC — verify per network)"),
        DeclareLaunchArgument("port", default_value="5556",
                              description="byd_cereal_server.py TCP port on the device"),
        DeclareLaunchArgument("rate", default_value="50.0",
                              description="odometry publish rate, Hz"),
        DeclareLaunchArgument("path_publish_hz", default_value="10.0",
                              description="Path republish rate, Hz. Poses are still appended "
                                          "every tick; only the republish is throttled, since a "
                                          "Path carries its whole history and costs O(n) to send."),
        DeclareLaunchArgument("wheelbase", default_value="2.70",
                              description="Dolphin wheelbase, m (NOT the SEAL 2.92 placeholder)"),
        DeclareLaunchArgument("corrected_steer_ratio", default_value="14.2",
                              description="steer ratio for the CORRECTED track (13.11 over-predicts "
                                          "yaw ~8%; fitted estimate 14.1-14.3). Sweep this to close "
                                          "the loop, e.g. corrected_steer_ratio:=14.5"),
        DeclareLaunchArgument("steer_ratio", default_value="13.11",
                              description="kinematic-track steer ratio (NOT the SEAL 16.0 placeholder). "
                                          "Known ~8%% yaw over-prediction at 13.11; "
                                          "effective kinematic ratio nearer 14.1-14.3 — try both, "
                                          "compare against the green (measured) track directly."),
        DeclareLaunchArgument("gear_mode", default_value="reverse-only",
                              description="how gear maps to the sign of integrated speed, for ALL "
                                          "tracks. 'signed': D +1, R -1, P/N/unknown 0. "
                                          "'reverse-only' (DEFAULT): R -1, everything else +1, so an "
                                          "unforwarded gear never freezes the path. 'off': gear "
                                          "ignored, reverse folds the path back on itself. "
                                          "e.g. gear_mode:=reverse-only"),
        DeclareLaunchArgument("rviz", default_value="true",
                              description="also launch RViz2"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz),
        DeclareLaunchArgument("rviz_delay_s", default_value="5.0",
                              description="seconds between the node's first "
                                          "/byd/odom_corrected message and RViz opening"),
        DeclareLaunchArgument("odom_wait_timeout_s", default_value="60",
                              description="give up waiting for the first odom message "
                                          "after this many seconds and open RViz anyway"),
    ]

    odom_node = Node(
        package="byd_odom_ros",
        executable="odom_node",
        name="byd_odom_node",
        output="screen",
        arguments=[
            "--host", LaunchConfiguration("host"),
            "--port", LaunchConfiguration("port"),
            "--rate", LaunchConfiguration("rate"),
            "--path-publish-hz", LaunchConfiguration("path_publish_hz"),
            "--wheelbase", LaunchConfiguration("wheelbase"),
            "--steer-ratio", LaunchConfiguration("steer_ratio"),
            "--corrected-steer-ratio", LaunchConfiguration("corrected_steer_ratio"),
            "--gear-mode", LaunchConfiguration("gear_mode"),
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    # Wait for the node to actually PUBLISH, rather than a bare sleep: startup
    # time varies (device stream connect, first sample). `ros2 topic echo --once`
    # exits on the first message; `timeout` bounds the wait. Only started when
    # RViz is wanted at all.
    wait_for_odom = ExecuteProcess(
        cmd=["timeout", LaunchConfiguration("odom_wait_timeout_s"),
             "ros2", "topic", "echo", "--once",
             "/byd/odom_corrected", "nav_msgs/msg/Odometry"],
        name="wait_for_odom",
        output="log",
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    def _open_rviz_after_wait(event, context):
        if event.returncode == 0:
            delay = float(LaunchConfiguration("rviz_delay_s").perform(context))
            msg = ("[byd_drive] session-zero collection complete "
                   "\u2014 RViz opening, ready to drive")
        else:
            delay = 0.0
            msg = ("[byd_drive] no /byd/odom_corrected message within "
                   "odom_wait_timeout_s \u2014 opening RViz anyway; the node is "
                   "not publishing yet, check the device stream")
        return [TimerAction(period=delay, actions=[LogInfo(msg=msg), rviz_node])]

    open_rviz = RegisterEventHandler(
        OnProcessExit(target_action=wait_for_odom, on_exit=_open_rviz_after_wait))

    return LaunchDescription(args + [odom_node, wait_for_odom, open_rviz])
