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
| `origin_frontier_explore` | Own C++ frontier explorer; navigation backend Nav2 or custom (A* + DWA) |
| `origin_yolo_perception` | YOLO detection, tracking and counting on the camera stream |
| `origin_bringup` | Top-level launch file, RViz config and simulation worlds |

## Quick start (simulation)

```bash
# Development container (from the workspace root, once)
docker build -t origin_autonomy:jazzy src/origin_autonomy/docker
src/origin_autonomy/docker/run.sh
docker exec -it avular_jazzy bash

# Inside the container
colcon build --symlink-install && source install/setup.bash
ros2 launch origin_bringup sim.launch.py                                   # Gazebo, origin_office world
ros2 launch origin_frontier_explore explore.launch.py use_sim_time:=true rviz:=true backend:=nav2
#   explores the unknown world autonomously while RTAB-Map builds the 3D map (backend:=custom for
#   the own A* + DWA navigator); then localize and navigate in the saved map, see
#   origin_frontier_explore/README.md
```

Perception test (moving people, identical chairs; Gazebo shows the lidar beams):

```bash
ros2 launch origin_bringup sim.launch.py world:=origin_office_people.sdf
ros2 launch origin_yolo_perception perception.launch.py use_sim_time:=true sim:=true
ros2 launch origin_frontier_explore explore.launch.py use_sim_time:=true rviz:=true
```

Worlds in `origin_bringup/worlds`:

- `origin_office.sdf`: 20 × 14 m, five-room indoor test world with obstacles of different
  heights. It uses no online models, so it loads offline.
- `origin_office_people.sdf`: the same floor plan with 3 identical walking people, 3 standing
  people, 5 identical office chairs, 2 dining chairs, a dining table, a sofa and a
  refrigerator. The models are downloaded from Gazebo Fuel on first use.

`config/fastdds.xml` (set by the Docker image) is required: without it, best-effort
subscribers lose about 40% of the Ouster clouds.

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
