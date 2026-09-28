# Copyright 2026 Soumic Sarkar
# SPDX-License-Identifier: Apache-2.0
"""YOLO detection / tracking / counting on the Origin One camera (own process).

    ros2 launch origin_yolo_perception perception.launch.py use_sim_time:=true sim:=true

Topics follow the Origin One's /robot/... names (the Avular simulation mirrors the real robot).
sim:=true adds the optical frame override (the simulated images are stamped camera_link);
on the robot the RealSense images are already in the optical frame. If the robot's driver uses
other names, remap image / depth / camera_info / cloud.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    pkg = get_package_share_directory("origin_yolo_perception")
    sim = LaunchConfiguration("sim").perform(context).lower() in ("true", "1")
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() in ("true", "1")
    cam = "/robot/camera"
    topics = {"image": f"{cam}/color/image_raw", "depth": f"{cam}/depth/image_raw",
              "camera_info": f"{cam}/color/camera_info", "cloud": "/robot/lidar/points"}
    optical = "camera_color_optical_frame" if sim else ""
    overrides = {
        "use_sim_time": use_sim_time,
        "optical_frame": optical,
        "tracker": os.path.join(pkg, "config", "bytetrack_origin.yaml"),
    }
    model = LaunchConfiguration("model").perform(context)
    if model:
        overrides["model"] = model
    return [Node(
        package="origin_yolo_perception", executable="yolo_node", name="yolo_perception",
        namespace="perception", output="screen",
        parameters=[os.path.join(pkg, "config", "perception.yaml"), overrides],
        remappings=list(topics.items()),
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("sim", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("model", default_value="",
                              description="Override the YOLO weights (e.g. yolo11s.pt)"),
        OpaqueFunction(function=launch_setup),
    ])
