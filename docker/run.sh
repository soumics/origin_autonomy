#!/usr/bin/env bash
# Run the origin_autonomy image.
#
#   docker/run.sh                          # everything: Gazebo, SLAM, exploration, YOLO, RViz
#   docker/run.sh mode:=navigate           # extra bringup.launch.py arguments
#   docker/run.sh dev                      # persistent dev container "avular_jazzy" with this
#                                          # workspace mounted at /root/ros2_ws
#
# Maps (RTAB-Map databases) persist in ~/.origin_autonomy (mounted at /root/.ros).
set -euo pipefail
IMAGE="${IMAGE:-origin_autonomy:jazzy}"
DATA="${ORIGIN_DATA:-$HOME/.origin_autonomy}"
mkdir -p "$DATA"
xhost +local: >/dev/null

COMMON=(--gpus=all --net=host --ipc=host --privileged
        -e DISPLAY="$DISPLAY"
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw
        -v "$DATA:/root/.ros")
if [ -n "${XAUTHORITY:-}" ]; then
  COMMON+=(-e XAUTHORITY="$XAUTHORITY" -v "$XAUTHORITY:$XAUTHORITY:ro")
fi

if [ "${1:-}" = "dev" ]; then
  WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
  docker run -d --name avular_jazzy "${COMMON[@]}" \
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/root/ros2_ws/src/origin_autonomy/config/fastdds.xml \
    -v "$WS:/root/ros2_ws" -w /root/ros2_ws "$IMAGE" sleep infinity
  echo "Dev container started: docker exec -it avular_jazzy bash"
else
  docker run --rm -it "${COMMON[@]}" "$IMAGE" \
    ros2 launch origin_bringup bringup.launch.py "$@"
fi
