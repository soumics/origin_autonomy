# origin_autonomy

Autonomy stack for the Avular Origin One on ROS 2 Jazzy: 3D lidar mapping and localization,
frontier exploration with Nav2, and YOLO camera perception. It is developed in Gazebo Harmonic
first and targets the real robot afterwards.

The stack runs on a separate PC with ROS 2 Jazzy. The robot keeps its stock Avular (Humble)
software, including its onboard `cmd_vel_controller`, and the two talk over the network with
standard ROS messages.

## Packages

| Package | Purpose |
|---|---|
| `origin_lidar_localization` | 3D SLAM and localization from the Ouster point cloud (RTAB-Map) |
| `origin_frontier_explore` | Frontier exploration and Nav2 bringup |
| `origin_yolo_perception` | YOLO detection, tracking and counting on the camera stream |
| `origin_bringup` | Top-level launch file, RViz config and simulation worlds |

## Dependencies

- [avular_origin_description](https://github.com/soumics/avular_origin_description) and
  [avular_origin_simulation](https://github.com/soumics/avular_origin_simulation), branch
  `ros2-jazzy`, in the same workspace
- Install ROS dependencies from the workspace root:

  ```bash
  rosdep install --from-paths src --ignore-src -y --skip-keys "cmd_vel_controller ament_copyright_avular"
  ```

## License

Apache-2.0. The Avular packages this depends on are under their own licences.
