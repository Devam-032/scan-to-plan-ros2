# TurtleBot3 Probabilistic Occupancy Grid Mapping + A* Path Planning

A from-scratch implementation of probabilistic occupancy grid mapping, A* path planning, and Stanley path following in ROS 2 Humble (C++), built on top of TurtleBot3 Gazebo simulation — without Nav2.

---

## Demo

![Demo](assets/demo.gif)

> Grey = unknown | White = free space | Black = occupied walls | Green line = A\* planned path

---

## What This Project Does

The node subscribes to a TurtleBot3's LiDAR scan and odometry, builds a probabilistic 2D occupancy grid of the environment in real time, plans an A* path from the robot's current position to a user-defined goal, and follows it using a Stanley controller — all published and visualised in RViz2.

```
/scan  (LaserScan) ──┐
                     ├──► Log-odds grid ──► Gradient inflation ──► A* planner ──► /planned_path
/odom  (Odometry)  ──┤                                                                  │
                     └──► Stanley controller ◄──────────────────────────────────────────┘
                                  │
                                  ▼
                             /cmd_vel
```

---

## Background and Learning Path

Built as a learning exercise progressing through the following concepts, each implemented from scratch:

### 1. Occupancy Grid Theory
Based on Cyrill Stachniss's occupancy grid mapping lectures (University of Bonn). Covers the probabilistic foundation of how sensor data updates cell belief states using Bayesian inference.

### 2. LaserScan to Cartesian Conversion
The TurtleBot3's 2D LiDAR publishes polar data `(range, angle)`. Each beam is converted to a Cartesian `(x, y)` world-frame point:
```
x = posx + r * cos(theta + yaw)
y = posy + r * sin(theta + yaw)
```

### 3. Bresenham Ray Tracing
Rather than only marking beam endpoints as occupied, each beam is traced through the grid using Bresenham's line algorithm. Every cell the beam passes through receives a **free space update** and only the endpoint receives an **occupied update** — enabling the map to distinguish free space from unknown space.

### 4. Log-Odds Probabilistic Updates
Each cell stores a log-odds value `l = log(p / (1 - p))` rather than raw probability:
- `L_OCC = +3.0` — strong evidence of obstacle (beam endpoint)
- `L_FREE = -0.10` — weak evidence of free space (beam passes through)

Log-odds values are clamped to `[-10, +20]`. Three-state output:
```
l > 3.0   → 100  (occupied — black)
l < -1.0  → 0    (free — white)
otherwise → -1   (unknown — grey)
```

### 5. BFS → Dijkstra → A*
Path planning built in three stages from scratch:

**BFS** — flood-fill using `std::queue`. Finds shortest path in step count, equal cost per move.

**Dijkstra** — upgrades BFS with `std::priority_queue`. Straight moves cost `1.0`, diagonal moves cost `√2 ≈ 1.414`.

**A\*** — upgrades Dijkstra with heuristic `h(n)` = Euclidean distance to goal. Always expands lowest `f = g + h`.

### 6. Gradient Cost Inflation
Instead of binary lethal/free inflation, cells near obstacles receive intermediate costs:
```
Distance 0  (on obstacle)  → 100  (lethal — A* never enters)
Distance 1-3               →  80  (very expensive)
Distance 4-7               →  50  (expensive)
Distance 7-10              →  20  (slightly penalised)
Beyond radius              →   0  (free)
```
A* naturally avoids walls when space allows but can still navigate tight passages when necessary.

### 7. RDP Path Smoothing
Ramer-Douglas-Peucker algorithm removes unnecessary intermediate waypoints that lie close to the straight line between their neighbours — eliminating the jagged staircase pattern typical of grid-based A* paths.

### 8. Stanley Controller
Lateral path following using heading error and signed crosstrack error:
```
steering = heading_error + atan(k * crosstrack / (speed + epsilon))
```
Softening term `epsilon` prevents division-by-zero instability at low speeds.

### 9. Speed Scaling on Curves
Curvature at each path segment computed from three consecutive waypoints. Speed is inversely proportional to curvature — robot slows on tight corners and moves faster on straight sections:
```
speed = clamp(MAX_SPEED / (1 + k_curv * curvature), MIN_SPEED, MAX_SPEED)
```

### 10. Dynamic Goal via RViz2
Subscribes to `/goal_pose` (`geometry_msgs/PoseStamped`) — click **2D Nav Goal** in RViz2 to set a new goal in real time. A* replans immediately.

---

## Package Structure

```
turtlebot3_tuts/
├── src/
│   └── scan_to_cartesian.cpp    ← main node (all components in one file)
├── launch/
│   └── mapping_astar.launch.py  ← launches Gazebo + node + RViz2
├── .rviz2/
│   └── mapping.rviz             ← RViz2 config
├── assets/
│   └── demo.gif                 ← demo recording
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## Dependencies

- ROS 2 Humble
- TurtleBot3 packages (`turtlebot3`, `turtlebot3_gazebo`, `turtlebot3_msgs`)
- `nav_msgs`, `sensor_msgs`, `geometry_msgs`, `tf2_ros`, `tf2_geometry_msgs`

Install TurtleBot3:
```bash
sudo apt install ros-humble-turtlebot3 ros-humble-turtlebot3-gazebo
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

---

## Build

```bash
cd ~/your_ws
colcon build --packages-select turtlebot3_tuts --symlink-install
source install/setup.bash
```

---

## Run

```bash
# Default goal at (2.0, 2.0)
ros2 launch turtlebot3_tuts mapping_astar.launch.py

# Custom goal at launch
ros2 launch turtlebot3_tuts mapping_astar.launch.py goal_x:=1.5 goal_y:=-1.0
```

Change goal while running via parameter:
```bash
ros2 param set /scan_to_cartesian goal_x 2.5
ros2 param set /scan_to_cartesian goal_y 0.5
```

Or click **2D Nav Goal** in RViz2 — path replans immediately.

Drive the robot manually:
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

---

## RViz2 Setup

| Display | Topic | Type |
|---|---|---|
| Occupancy Grid | `/occ_grid_Devam` | Map |
| Planned Path | `/planned_path` | Path |
| Cartesian Points | `/scan_cartesian` | Float64MultiArray |

Set **Fixed Frame** to `odom`.

---

## Tuning Parameters

| Parameter | Default | Effect |
|---|---|---|
| `INFLATION_CELLS_` | 10 | Lethal obstacle radius in cells |
| `COST_PENALTY_WEIGHT_` | 2.0 | Strength of gradient cost near walls |
| `RDP_EPSILON_` | 0.1 m | Path smoothing aggressiveness |
| `STANLEY_K_` | 1.0 | Crosstrack correction gain |
| `STANLEY_EPS_` | 0.01 | Speed softening term |
| `MAX_SPEED_` | 0.15 m/s | Speed on straight sections |
| `MIN_SPEED_` | 0.04 m/s | Speed on tight corners |
| `CURV_GAIN_` | 5.0 | How aggressively speed is reduced on curves |
| `GOAL_TOLERANCE_` | 0.15 m | Distance at which goal is considered reached |
| `WAYPOINT_RADIUS_` | 0.10 m | Distance to advance to next waypoint |

---

## Grid Parameters

| Parameter | Value | Notes |
|---|---|---|
| Resolution | 0.02 m/cell | ~1cm precision at 1m range |
| Size | 500 × 500 cells | 10m × 10m coverage |
| Origin | (-5.0, -5.0) | Centred on robot start position |
| Free trace max | 3.4 m | Max beam length for free-space update |

---

## Known Limitations

- **Odometry drift** — pure dead-reckoning accumulates heading error over time, causing slight wall smearing during repeated rotation. Proper correction requires AMCL or SLAM.
- **Fixed grid origin** — the grid is anchored at the robot's starting position. If the robot travels more than ~5m from start, points fall outside the grid boundary.
- **Replanning on every scan** — A* replans at every LiDAR callback (~10Hz). On complex maps this can cause brief path flickering. A replanning cooldown timer would help.
- **No reverse motion** — Stanley controller only drives forward. Situations requiring reversing (e.g. dead ends) will cause the robot to stall.

---

## What's Next

This TurtleBot3 project served as the perception and planning foundation for a larger autonomous vehicle project in CARLA 0.9.14:

- [ ] Port occupancy grid to 3D LiDAR PointCloud2 (`lidar_grid_node.cpp` in CARLA)
- [ ] Implement Hybrid A\* with Reeds-Shepp curves for car-kinematic planning
- [ ] Full autonomous parking pipeline in CARLA

---

## Related Project

**CARLA Hybrid A\* Parking Planner** — autonomous parking in CARLA 0.9.14 using 3D LiDAR, Hybrid A\*, and full vehicle kinematics. *(In progress)*