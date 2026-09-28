# Environment of the origin_autonomy image (sourced by the entry point and the helpers).
source /opt/ros/jazzy/setup.bash
source /opt/avular_ws/install/setup.bash      # origin_msgs (Avular's public interfaces)
source /opt/origin_ws/install/setup.bash
if [ -f /root/ros2_ws/install/setup.bash ]; then
  # Optional development overlay; a broken or half-built overlay must not stop the container.
  source /root/ros2_ws/install/setup.bash || echo "entrypoint: ignoring /root/ros2_ws overlay"
fi
