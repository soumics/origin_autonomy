#!/usr/bin/env bash
# Entry point of the origin_autonomy image: set up the ROS environment, then run the command
# (default: the full bringup). A development workspace at /root/ros2_ws is overlaid if built.
source /opt/origin_ws/entrypoint_env.sh
exec "$@"
