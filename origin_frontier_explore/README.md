# origin_frontier_explore

Autonomous frontier exploration for the Origin One, written in C++. The robot starts with no map.
RTAB-Map (`origin_lidar_localization`) builds the map while the robot explores, and closes
loops when it revisits places. The saved map is then used to localize and navigate.

## Nodes

| Node | Purpose |
|---|---|
| `frontier_explorer` | Finds frontiers on the live `/map` and sends the best one as a `NavigateToPose` goal. Handles goal replacement, blacklisting and completion, and returns to the start when done |
| `custom_navigator` | Custom navigation backend, a `NavigateToPose` server: A* on `/map` with inflation plus a Dynamic Window Approach local controller on the lidar |
| `cloud_self_filter` | Removes lidar returns off the robot's own body and antenna (about 15% of Ouster points) → `/robot/lidar/points_filtered` |
| `wait_for_transform.py` | Starts Nav2 only once RTAB-Map publishes `map → base_link` |

The algorithms (`grid`, `frontier_search`, `grid_planner`, `dwa_controller`) are plain C++ with no
ROS dependency, and are unit-tested in `test/test_algorithms.cpp`.

## Usage

```bash
# Simulation (from origin_bringup)
ros2 launch origin_bringup sim.launch.py

# 1. Explore and build the map (RTAB-Map mapping + navigation backend + explorer)
ros2 launch origin_frontier_explore explore.launch.py use_sim_time:=true rviz:=true \
    backend:=nav2 database_path:=/root/ros2_ws/maps/office.db       # or backend:=custom
# Explorer status: /frontier_explorer/status (exploring | returning | done).
# Ctrl-C when done; RTAB-Map writes the database on exit.

# 2. Localize and navigate in the saved map
ros2 launch origin_lidar_localization slam.launch.py use_sim_time:=true localization:=true \
    start_at_origin:=false database_path:=/root/ros2_ws/maps/office.db
ros2 launch origin_frontier_explore navigation.launch.py use_sim_time:=true backend:=nav2
# Send goals with RViz "2D Goal Pose" or the /navigate_to_pose action.
```

Services: `/frontier_explorer/start` and `/frontier_explorer/stop` (`std_srvs/Trigger`).

## How it works

**Frontier search** (`frontier_search.cpp`)
1. Cells at least `robot_clearance` (0.40 m) from obstacles are traversable.
2. Dijkstra from the robot over traversable cells gives the travel distance to each cell.
3. A reachable cell next to unknown space is a frontier cell; cells outside the map array count
   as unknown.
4. Frontier cells are grouped into 8-connected clusters, dropping clusters under 10 cells.
5. Each cluster's goal is its cell closest to the centroid.
6. Score = length − travel distance; the best score wins.

**Explorer** (`frontier_explorer_node.cpp`)
- Re-evaluates the frontiers every second.
- A goal is replaced when its frontier has been seen, meaning no frontier cell remains within
  0.6 m of it.
- Goals that fail, or where the robot moves less than 0.3 m in 30 s, are blacklisted.
- A goal rejected by a server that is not yet active is simply retried.
- After 3 empty checks it returns to the start pose and stops.
- **Look-around:** the lidar sees 360° but the camera only about 60° ahead, so the robot spins
  once in place at the start and after every 4 m travelled (`look_around_distance`, 0 = off).
  This lets YOLO perception see every area that exploration covers.

**Custom navigator** (`custom_navigator_node.cpp`)
- **Global plan:** A* on `/map`, replanned every second. Cells within 0.40 m of an obstacle are
  lethal, with a cost gradient out to 1.0 m. Unknown cells are allowed at 3× cost, and the
  planner can leave the inflation zone if the robot starts inside it.
- **Local control:** a Dynamic Window Approach on lidar points 0.08–0.8 m high, remembered for
  2 s in `icp_odom` to cover the near-ground blind zone.
  - The collision check uses the rectangular footprint plus a 6 cm margin.
  - An arc is admissible if the robot can brake before the first collision point
    (v²/2a plus one control period).
  - If the robot already starts inside the margin, it may not approach the obstacle further.
- **Recovery:** it backs up, or rotates if the space behind is blocked. It aborts after 4
  recoveries or 3 failed plans.

**Nav2 backend** (`config/nav2_params.yaml`)
- MPPI controller (DiffDrive model) and NavFn A* planner (`allow_unknown`).
- 3D voxel layers on the filtered cloud (0.08–0.8 m) with the Origin One footprint.
- Collision monitor with a fixed footprint polygon, in front of `/robot/cmd_vel`.
- The local costmap and behaviours use `icp_odom`: skid-steer wheel odometry drifts heavily
  when turning.

## Results in Gazebo (`origin_office`, 20 × 14 m, five rooms)

| | custom backend | Nav2 backend |
|---|---|---|
| Exploration time (no map → no frontiers) | 111 s + 34 s return | 123–190 s + 46 s return |
| Goals / blacklisted / recoveries | 18 / 0 / 0 | 21–28 / 0 / 0 |
| Mapped free space | 262.8 m² (all rooms) | 263.7–264.7 m² |
| SLAM pose vs ground truth at the end | 1 mm, 0.001 rad | 1 mm, 0.001 rad |
| Navigation in the saved map (3 goals across rooms) | 3/3, 29–57 s, ≤ 5 mm localization error | 3/3, 36–60 s, ≤ 1 mm |
