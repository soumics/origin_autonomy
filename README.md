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

`bringup.launch.py` (the default command) decides by itself what to run:

1. **Robot or simulation (`sim:=auto`).** For about 8 s it looks for a running Origin One on
   the network (`detect_origin_one.py`).
   - **Found** (a lidar point cloud that is not a simulation): the stack runs on the robot. The
     lidar, camera, depth, odometry and velocity topics are taken from what the robot
     publishes, and velocity goes to `/robot/cmd_vel_user` when the Origin One's
     `cmd_vel_controller` is there.
   - **Not found:** it prints that the Origin One has not been started or is not on the same
     network as this PC, and runs the Gazebo simulation instead. `sim:=false` keeps waiting
     for the robot; `sim:=true` skips the check.
2. **Explore or navigate (`mode:=auto`).**
   - **No saved map:** the robot explores autonomously (frontiers, obstacle avoidance) while
     RTAB-Map builds the 3D map. When no frontiers are left, it returns to its start,
     `mission_manager` saves the map (`database_path`, plus a 2D `.pgm`/`.yaml` copy), and
     RTAB-Map switches to **localization automatically**.
   - **A saved map exists:** it starts directly in localization.
   - In both cases you then send goals with RViz "2D Goal Pose".
3. **YOLO perception and RViz:** robot, Ouster, map, costmaps, paths, frontiers, YOLO image
   with counts, 3D object markers.

| Argument | Default | |
|---|---|---|
| `sim` | `auto` | `auto`: robot if found, else Gazebo; `true`: Gazebo; `false`: wait for the robot |
| `mode` | `auto` | `auto`: localize if a map exists, else explore; `explore` (old map kept as `.bak`); `navigate` |
| `backend` | `nav2` | `nav2` or `custom` (own A* + DWA) |
| `world` | `origin_office_people.sdf` | or `origin_office.sdf` (no people, plain obstacles) |
| `database_path` | `~/.ros/origin_rtabmap.db` | map, persisted in `~/.origin_autonomy` on the host |
| `*_topic` | detected | `lidar_topic`, `image_topic`, `depth_topic`, `camera_info_topic`, `odom_topic`, `cmd_vel_topic` overrides |
| `perception`, `rviz`, `lidar_rays`, `headless` | `true`, `true`, `true`, `false` | switch parts on or off |
| `yolo_model` | (`yolo11s.pt`) | e.g. `yolo26s.pt` |

In simulation, ROS discovery stays on this machine (`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`),
so Wi-Fi changes cannot cut nodes off. When the robot is detected, the bringup switches every
node to `SUBNET` itself. The robot and the PC must use the same `ROS_DOMAIN_ID` (pass
`-e ROS_DOMAIN_ID=<n>` if the robot does not use 0).

Real robot, still open (Phase 7): the Origin One only executes `/robot/cmd_vel_user` in its
"user" control mode, which is set with `origin_msgs/srv/SetControlMode`; that step is not
automated yet. Keep a hand on the e-stop for the first runs.

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
