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

## Quick start: one command, everything in Docker

The image clones this repository and the two Avular forks (branch `ros2-jazzy`) from GitHub,
builds them and starts the whole stack:

```bash
docker build -t origin_autonomy:jazzy https://github.com/soumics/origin_autonomy.git#ros2-jazzy:docker
xhost +local:
docker run --rm -it --gpus=all --net=host --ipc=host -e DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix -v ~/.origin_autonomy:/root/.ros origin_autonomy:jazzy
```

With a local checkout, `docker/run.sh` wraps this (`docker/run.sh mode:=navigate`,
`docker/run.sh dev` for a development container with the workspace mounted).

`bringup.launch.py` (the default command) starts in order:

1. **Gazebo** with `origin_office_people.sdf`: 3 walking people, standing people, identical
   chairs and furniture. The Ouster beams are drawn in the Gazebo GUI.
2. **RTAB-Map** 3D SLAM.
3. **Navigation** with the frontier explorer (`backend:=nav2` or `custom`).
4. **YOLO perception.**
5. **RViz:** robot, Ouster, map, costmaps, paths, frontiers, YOLO image with counts, 3D
   object markers.

| Argument | Default | |
|---|---|---|
| `mode` | `explore` | `explore`: no map, explore autonomously and build it; `navigate`: localize in the saved map, goals from RViz "2D Goal Pose" |
| `backend` | `nav2` | `nav2` or `custom` (own A* + DWA) |
| `sim` | `true` | `false`: real robot (no Gazebo, wall clock) |
| `world` | `origin_office_people.sdf` | or `origin_office.sdf` (no people, plain obstacles) |
| `database_path` | `~/.ros/origin_rtabmap.db` | map, persisted in `~/.origin_autonomy` on the host |
| `perception`, `rviz`, `lidar_rays`, `headless` | `true`, `true`, `true`, `false` | switch parts on or off |
| `yolo_model` | (`yolo11s.pt`) | e.g. `yolo26s.pt` |

Typical session: run once with the default `mode:=explore`. The robot explores until no
frontiers are left and returns to its start; then Ctrl-C saves the map. Next,
`mode:=navigate` localizes in that map, and goals are sent with RViz "2D Goal Pose".

ROS discovery is limited to this machine (`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`). For
the real robot, where PC and robot talk over the network, add
`-e ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`.

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
