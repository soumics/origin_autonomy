#!/usr/bin/env bash
# Start the persistent dev container `avular_jazzy` with the workspace mounted at /root/ros2_ws.
# Open a shell in it with:  docker exec -it avular_jazzy bash
set -euo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
IMAGE="${IMAGE:-origin_autonomy:jazzy}"

xhost +local: >/dev/null
docker run -d --name avular_jazzy \
  -e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 -e XAUTHORITY="$XAUTHORITY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v "$XAUTHORITY:$XAUTHORITY:ro" \
  -v "$WS:/root/ros2_ws" \
  --net=host --ipc=host --privileged --gpus=all \
  "$IMAGE" sleep infinity
