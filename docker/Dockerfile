# Development image for the Origin One autonomy stack (ROS 2 Jazzy + Gazebo Harmonic).
#
#   docker build -t origin_autonomy:jazzy src/origin_autonomy/docker
#   src/origin_autonomy/docker/run.sh
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
 && python3 -c "from ultralytics import YOLO; [YOLO(m) for m in ('yolo11s.pt', 'yolo26s.pt')]" \
 && ls -l /opt/models

# Fast DDS profile: large shared-memory segment so ~0.7 MB Ouster clouds reach best-effort
# subscribers (RTAB-Map, Nav2, RViz) without loss. See config/fastdds.xml.
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/root/ros2_ws/src/origin_autonomy/config/fastdds.xml

# Render Gazebo and RViz on the NVIDIA GPU (also on hybrid-graphics laptops via PRIME offload)
ENV NVIDIA_DRIVER_CAPABILITIES=all \
    NVIDIA_VISIBLE_DEVICES=all \
    __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json \
    __NV_PRIME_RENDER_OFFLOAD=1 \
    __GLX_VENDOR_LIBRARY_NAME=nvidia

WORKDIR /root/ros2_ws
RUN echo 'source /opt/ros/jazzy/setup.bash; [ -f /root/ros2_ws/install/setup.bash ] && source /root/ros2_ws/install/setup.bash' >> /root/.bashrc
