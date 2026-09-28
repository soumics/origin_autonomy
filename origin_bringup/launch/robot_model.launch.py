# Copyright 2026 Soumic Sarkar
# SPDX-License-Identifier: Apache-2.0
"""Robot model and TF for the real Origin One (the PC side of the Zenoh bridge).

- origin_one_description's robot_state_publisher: /robot/robot_description and the body /
  sensor mount frames (base_link -> os_sensor, camera_link, ...)
- the sensor frames that the Gazebo launch also publishes (camera optical frames, os_lidar)
- tf_relay: the robot's /robot/tf -> /tf (odom -> base_link), without its own map -> odom;
  odom -> base_link from the odometry topic if the robot's TF does not come through
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# (x, y, z, yaw, pitch, roll, parent, child), same as origin_one_gazebo's simulation launch
SENSOR_FRAMES = [
    (0, 0, 0, 0, 0, 0, "camera_link", "camera_color_frame"),
    (0, 0, 0, -1.5707, 0, -1.5707, "camera_color_frame", "camera_color_optical_frame"),
    (0, 0, 0, 0, 0, 0, "camera_link", "camera_depth_frame"),
    (0, 0, 0, -1.5707, 0, -1.5707, "camera_color_frame", "camera_depth_optical_frame"),
    (0.006253, -0.011775, 0.007645, 0, 0, 0, "os_sensor", "os_imu"),
    (0, 0, 0.03618, 0, 0, 0, "os_sensor", "os_lidar"),
]


def generate_launch_description():
    description = os.path.join(get_package_share_directory("origin_one_description"),
                               "launch", "origin_one_description.launch.py")
    nodes = [DeclareLaunchArgument("odom_topic", default_value="/robot/odom"),
             IncludeLaunchDescription(PythonLaunchDescriptionSource(description))]
    for x, y, z, yaw, pitch, roll, parent, child in SENSOR_FRAMES:
        nodes.append(Node(
            package="tf2_ros", executable="static_transform_publisher", output="log",
            name=f"static_{child}",
            arguments=["--x", str(x), "--y", str(y), "--z", str(z), "--yaw", str(yaw),
                       "--pitch", str(pitch), "--roll", str(roll),
                       "--frame-id", parent, "--child-frame-id", child]))
    nodes.append(Node(package="origin_bringup", executable="tf_relay.py", name="tf_relay",
                      output="screen",
                      remappings=[("odom", LaunchConfiguration("odom_topic"))]))
    return LaunchDescription(nodes)
