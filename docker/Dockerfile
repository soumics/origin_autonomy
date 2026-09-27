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
 && rm -rf /var/lib/apt/lists/*

# Render Gazebo and RViz on the NVIDIA GPU (also on hybrid-graphics laptops via PRIME offload)
ENV NVIDIA_DRIVER_CAPABILITIES=all \
    NVIDIA_VISIBLE_DEVICES=all \
    __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json \
    __NV_PRIME_RENDER_OFFLOAD=1 \
    __GLX_VENDOR_LIBRARY_NAME=nvidia

WORKDIR /root/ros2_ws
RUN echo 'source /opt/ros/jazzy/setup.bash; [ -f /root/ros2_ws/install/setup.bash ] && source /root/ros2_ws/install/setup.bash' >> /root/.bashrc
