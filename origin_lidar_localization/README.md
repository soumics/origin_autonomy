# origin_lidar_localization

3D lidar SLAM and localization for the Origin One with [RTAB-Map](https://github.com/introlab/rtabmap_ros),
fed by the Ouster point cloud (`/robot/lidar/points`).

## Usage

```bash
# 1. Build a map. Starts a new database, deleting the old one at database_path.
ros2 launch origin_lidar_localization slam.launch.py use_sim_time:=true rviz:=true \
    database_path:=/root/ros2_ws/maps/origin_office.db

# Drive the robot around. Stop with Ctrl-C; the map is saved to the database on exit.
# Optional 2D copy for other tools:
ros2 run nav2_map_server map_saver_cli -f maps/origin_office_2d --ros-args -p use_sim_time:=true

# 2. Localize in that map
ros2 launch origin_lidar_localization slam.launch.py use_sim_time:=true rviz:=true \
    localization:=true start_at_origin:=false database_path:=/root/ros2_ws/maps/origin_office.db
```

In localization mode RTAB-Map restores the last pose stored in the database. If the robot was
moved since then, set its pose with **2D Pose Estimate** in RViz (`/initialpose`); the pose is
then refined against the map. `start_at_origin:=true` instead assumes the robot starts where
mapping started.

Launch arguments: `localization`, `database_path` (default `~/.ros/origin_rtabmap.db`), `lidar_topic`,
`deskewing` (real Ouster only; Gazebo clouds have no per-point time), `start_at_origin`, `use_sim_time`, `rviz`.

## Outputs

| Topic / TF | Content |
|---|---|
| `/map` | 2D occupancy grid (0.05 m), projected from the 3D map; used by Nav2 and exploration |
| `/rtabmap/cloud_obstacles`, `/rtabmap/cloud_map` | 3D point-cloud map (obstacles only / with ground) |
| `/rtabmap/octomap_*` | OctoMap, including `octomap_global_frontier_space` |
| `/rtabmap/icp_odom` | Lidar odometry |
| TF `map → icp_odom → odom → base_link` | `map → icp_odom`: graph optimization; `icp_odom → odom`: ICP correction of wheel odometry; `odom → base_link`: wheel odometry from the robot |

The wheel odometry is only a motion guess for ICP. On skid-steer it drifts badly when turning.
In a 5-minute Gazebo run it was off by 7.8 m and 2 rad, while the SLAM pose stayed within a few
millimetres of ground truth.

## Notes

- Lidar-only RTAB-Map has no bag-of-words place recognition, so loop closures come from
  proximity detection (scan overlap near earlier poses). Localization therefore needs a
  roughly correct starting pose: the pose stored in the database, `start_at_origin`, or
  2D Pose Estimate.
- Parameters are in `config/rtabmap_lidar3d.yaml`, based on `rtabmap_examples/lidar3d.launch.py`.
  Returns closer than 0.8 m are dropped, because the Ouster sees the robot's own rear body and
  GNSS antenna.
- RTAB-Map nodes are not lifecycle-managed. Nav2 consumes `/map` and TF directly, so no
  `map_server` or AMCL is needed.
