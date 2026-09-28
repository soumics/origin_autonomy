#!/usr/bin/env bash
# Entry point of the origin_autonomy image: source ROS and the built workspace, then run the
# command (default: the full bringup). A development workspace mounted at /root/ros2_ws is
# overlaid on top if it has been built.
set -e
source /opt/ros/jazzy/setup.bash
source /opt/origin_ws/install/setup.bash
if [ -f /root/ros2_ws/install/setup.bash ]; then
  source /root/ros2_ws/install/setup.bash
fi
exec "$@"
