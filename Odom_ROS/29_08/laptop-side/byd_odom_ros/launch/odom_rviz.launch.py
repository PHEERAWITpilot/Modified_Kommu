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
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("byd_odom_ros")
    default_rviz = os.path.join(pkg_share, "rviz", "byd_odom.rviz")

    args = [
        DeclareLaunchArgument("host", default_value="172.20.10.3",
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
        DeclareLaunchArgument("rviz", default_value="true",
                              description="also launch RViz2"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz),
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

    return LaunchDescription(args + [odom_node, rviz_node])
