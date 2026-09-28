#!/usr/bin/env bash
# Run the origin_autonomy image.
#
#   docker/run.sh                          # everything: Gazebo, SLAM, exploration, YOLO, RViz
#   docker/run.sh mode:=navigate           # extra bringup.launch.py arguments
#   docker/run.sh dev [name] [workspace]   # persistent dev container (default name
#                                          # "avular_jazzy", default workspace: this one)
#                                          # with the workspace mounted at /root/ros2_ws
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
  NAME="${2:-avular_jazzy}"
  WS="$(cd "${3:-$(dirname "${BASH_SOURCE[0]}")/../../..}" && pwd)"
  # Use the workspace's own Fast DDS profile if it has one, else the image's.
  DDS=()
  if [ -f "$WS/src/origin_autonomy/config/fastdds.xml" ]; then
    DDS=(-e FASTRTPS_DEFAULT_PROFILES_FILE=/root/ros2_ws/src/origin_autonomy/config/fastdds.xml)
  fi
  docker run -d --name "$NAME" "${COMMON[@]}" "${DDS[@]}" \
    -v "$WS:/root/ros2_ws" -w /root/ros2_ws "$IMAGE" sleep infinity
  echo "Dev container '$NAME' started with $WS at /root/ros2_ws: docker exec -it $NAME bash"
else
  docker run --rm -it "${COMMON[@]}" "$IMAGE" \
    ros2 launch origin_bringup bringup.launch.py "$@"
fi
