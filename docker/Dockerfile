# Origin One autonomy stack (ROS 2 Jazzy + Gazebo Harmonic), self-contained.
#
# Build (no local checkout needed, the repositories are cloned from GitHub):
#   docker build -t origin_autonomy:jazzy \
#     https://github.com/soumics/origin_autonomy.git#ros2-jazzy:docker
# Run everything (finds the Origin One through the Zenoh bridge if this PC is on its network,
# else Gazebo; SLAM, exploration, YOLO perception, RViz):
#   xhost +local:
#   docker run --rm -it --gpus=all --net=host --ipc=host \
#     -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v ~/.origin_autonomy:/root/.ros \
#     origin_autonomy:jazzy
# Launch arguments can be appended, e.g.
#   ... origin_autonomy:jazzy ros2 launch origin_bringup bringup.launch.py mode:=navigate
# (docker/run.sh wraps these commands.)
FROM osrf/ros:jazzy-desktop-full

# The base image lags the ROS apt repo. Upgrade first so newly installed binaries (e.g. rtabmap)
# match the installed libraries (otherwise: "undefined symbol ... diagnostic_updater").
RUN apt-get update \
 && DEBIAN_FRONTEND=noninteractive apt-get upgrade -y -o Dpkg::Options::="--force-confold" \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y \
      ros-jazzy-rtabmap-ros \
      ros-jazzy-navigation2 \
      ros-jazzy-nav2-bringup \
      python3-pip \
      git-lfs \
 && rm -rf /var/lib/apt/lists/*

# YOLO perception: PyTorch with CUDA 12.8 (first release line supporting Blackwell GPUs such as the
# RTX 50 series, compute capability 12.0) and Ultralytics. numpy < 2 and opencv-python 4.11 keep
# ROS Jazzy's Python packages (built against numpy 1.26) working.
RUN pip install --break-system-packages --no-cache-dir \
      torch==2.11.0 torchvision==0.26.0 --index-url https://download.pytorch.org/whl/cu128 \
 && pip install --break-system-packages --no-cache-dir \
      ultralytics==8.4.164 "numpy<2" opencv-python==4.11.0.86 "lap>=0.5.12"

# Model weights baked in, so nothing is downloaded at run time (e.g. on the robot).
ENV ORIGIN_YOLO_MODELS=/opt/models
RUN mkdir -p /opt/models && cd /opt/models \
 && python3 -c "from ultralytics import YOLO; [YOLO(m) for m in ('yolo11s.pt', 'yolo26s.pt')]"

# Gazebo Fuel models used by origin_office_people.sdf, cached so the world loads offline.
RUN . /opt/ros/jazzy/setup.sh \
 && for m in "OpenRobotics/models/OfficeChairBlack" "OpenRobotics/models/Dining Chair" \
             "OpenRobotics/models/Dining Table" "OpenRobotics/models/Sofa" \
             "OpenRobotics/models/Refrigerator" "OpenRobotics/models/Standing person" \
             "OpenRobotics/models/Casual female" "OpenRobotics/models/Male visitor" \
             "Mingfei/models/actor"; do \
      gz fuel download -u "https://fuel.gazebosim.org/1.0/$m" || exit 1; \
    done

# Zenoh bridge: Avular's documented way to reach the Origin One from another computer
# (software_development/ros2/ros2_connection). Version 1.0.3 is required ("newer versions may
# cause connectivity issues").
RUN echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" \
      > /etc/apt/sources.list.d/zenoh.list \
 && apt-get update \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y zenoh-bridge-ros2dds=1.0.3 \
      ros-jazzy-rmw-cyclonedds-cpp \
 && rm -rf /var/lib/apt/lists/*

# Workspace: the two Avular forks and origin_autonomy, branch ros2-jazzy, from GitHub.
# The ADDs bust the build cache whenever one of the branches gets a new commit.
ARG GITHUB_USER=soumics
ARG BRANCH=ros2-jazzy
ADD https://api.github.com/repos/${GITHUB_USER}/avular_origin_description/git/refs/heads/${BRANCH} /tmp/ref_description.json
ADD https://api.github.com/repos/${GITHUB_USER}/avular_origin_simulation/git/refs/heads/${BRANCH} /tmp/ref_simulation.json
ADD https://api.github.com/repos/${GITHUB_USER}/origin_autonomy/git/refs/heads/${BRANCH} /tmp/ref_autonomy.json
WORKDIR /opt/origin_ws/src
RUN git lfs install --system \
 && for r in avular_origin_description avular_origin_simulation origin_autonomy; do \
      git clone --depth 1 -b ${BRANCH} https://github.com/${GITHUB_USER}/$r.git || exit 1; \
    done

# origin_msgs (control mode service types) from Avular's public example repository.
ADD https://api.github.com/repos/avular-robotics/example_origin_ros/git/refs/heads/main /tmp/ref_avular_examples.json
RUN mkdir -p /opt/avular_ws/src && cd /opt/avular_ws/src \
 && git clone --depth 1 https://github.com/avular-robotics/example_origin_ros.git \
 && cd /opt/avular_ws && . /opt/ros/jazzy/setup.sh \
 && colcon build --packages-select origin_msgs && rm -rf build log

WORKDIR /opt/origin_ws
RUN apt-get update && rosdep update --rosdistro jazzy \
 && rosdep install --from-paths src --ignore-src -y --rosdistro jazzy \
      --skip-keys "cmd_vel_controller ament_copyright_avular" \
 && rm -rf /var/lib/apt/lists/* \
 && . /opt/ros/jazzy/setup.sh \
 && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
 && rm -rf build log

# Fast DDS profile: large shared-memory segment so ~0.7 MB Ouster clouds reach best-effort
# subscribers (RTAB-Map, Nav2, RViz) without loss. See config/fastdds.xml.
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/opt/origin_ws/src/origin_autonomy/config/fastdds.xml
# Discovery on this machine only: immune to Wi-Fi/IP changes. The real robot is reached through
# the local Zenoh bridge (also on this machine), so this holds for the robot as well.
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
# Avular: the external computer must use a different ROS_DOMAIN_ID than the robot.
ENV ROS_DOMAIN_ID=42

# Render Gazebo and RViz on the NVIDIA GPU (also on hybrid-graphics laptops via PRIME offload)
ENV NVIDIA_DRIVER_CAPABILITIES=all \
    NVIDIA_VISIBLE_DEVICES=all \
    __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json \
    __NV_PRIME_RENDER_OFFLOAD=1 \
    __GLX_VENDOR_LIBRARY_NAME=nvidia \
    QT_X11_NO_MITSHM=1

RUN cp /opt/origin_ws/src/origin_autonomy/docker/entrypoint_env.sh /opt/origin_ws/ \
 && install -m 755 /opt/origin_ws/src/origin_autonomy/docker/origin-enable-driving \
      /opt/origin_ws/src/origin_autonomy/docker/origin-stop-driving \
      /opt/origin_ws/src/origin_autonomy/docker/origin-check-robot /usr/local/bin/ \
 && echo 'source /opt/origin_ws/entrypoint_env.sh' >> /root/.bashrc
WORKDIR /root
ENTRYPOINT ["/opt/origin_ws/src/origin_autonomy/docker/entrypoint.sh"]
CMD ["ros2", "launch", "origin_bringup", "bringup.launch.py"]
