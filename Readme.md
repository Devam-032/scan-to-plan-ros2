# TurtleBot3 Probabilistic Occupancy Grid Mapping + A* Path Planning

A from-scratch implementation of probabilistic occupancy grid mapping and A* path planning in ROS 2 Humble (C++), built on top of TurtleBot3 Gazebo simulation.

---

## Demo

![Demo](assets/demo.gif)

> Grey = unknown, White = free space, Black = occupied walls, Green line = A\* planned path

---

## What This Project Does

The node subscribes to a TurtleBot3's LiDAR scan and odometry, builds a probabilistic 2D occupancy grid of the environment in real time, and plans an A* path from the robot's current position to a user-defined goal — all published and visualised in RViz2.

```
/scan (LaserScan)  ──┐
                     ├──► Occupancy Grid (log-odds) ──► /occ_grid_Devam
/odom (Odometry)  ──┘
                     └──► A* Planner ──────────────────► /planned_path
```

---

## Background and Learning Path

This project was built as a learning exercise progressing through the following concepts, each implemented from scratch without using existing navigation libraries:

### 1. Occupancy Grid Theory
Based on Cyrill Stachniss's occupancy grid mapping lectures (University of Bonn). Covers the probabilistic foundation of how sensor data updates cell belief states.

### 2. LaserScan to Cartesian Conversion
The TurtleBot3's 2D LiDAR publishes polar data `(range, angle)`. Each beam is converted to a Cartesian `(x, y)` world-frame point using:
```
x = posx + r * cos(theta + yaw)
y = posy + r * sin(theta + yaw)
```

### 3. Bresenham Ray Tracing
Rather than only marking beam endpoints as occupied, each beam is traced through the grid using Bresenham's line algorithm. Every cell the beam passes through receives a **free space update** and only the endpoint receives an **occupied update**. This is what enables the map to distinguish free space from unknown space.

### 4. Log-Odds Probabilistic Updates
Instead of binary 0/100 marking, each cell stores a log-odds value:
```
l = log(p / (1 - p))
```
Two constants control the update:
- `L_OCC = +3.0` — strong evidence of obstacle (beam endpoint)
- `L_FREE = -0.10` — weak evidence of free space (beam passes through)

Log-odds values are clamped to `[-10, +20]` to bound confidence. The three-state output maps to RViz2's map colour scheme:
```
l > 3.0   → 100  (occupied — black)
l < -1.0  → 0    (free — white)
otherwise → -1   (unknown — grey)
```

### 5. BFS → Dijkstra → A*
Path planning was built up in three stages, each implemented from scratch:

**BFS** — flood-fill exploration using `std::queue`. Finds shortest path in step count but treats all moves as equal cost.

**Dijkstra** — upgrades BFS by replacing `std::queue` with `std::priority_queue`. Straight moves cost `1.0`, diagonal moves cost `√2 ≈ 1.414`.

**A\*** — upgrades Dijkstra by adding a heuristic `h(n)` — Euclidean distance to goal. The planner always expands the cell with lowest `f = g + h`, steering search toward the goal and dramatically reducing explored cells on large grids.

---

## Package Structure

```
turtlebot3_tuts/
├── src/
│   └── scan_to_cartesian.cpp    ← main node (mapping + planning)
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
- `nav_msgs`, `sensor_msgs`, `geometry_msgs`, `tf2_geometry_msgs`

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

# Custom goal
ros2 launch turtlebot3_tuts mapping_astar.launch.py goal_x:=1.5 goal_y:=-1.0
```

Change goal while running (path replans automatically):
```bash
ros2 param set /scan_to_cartesian goal_x 2.5
ros2 param set /scan_to_cartesian goal_y 0.5
```

Drive the robot with teleop:
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

## Grid Parameters

| Parameter | Value | Notes |
|---|---|---|
| Resolution | 0.02 m/cell | ~1cm precision at 1m range |
| Size | 500 × 500 cells | 10m × 10m coverage |
| Origin | (-5.0, -5.0) | Centred on robot start position |
| Free trace max | 3.4 m | Max beam length for free-space update |

---

## Known Limitations

- **Odometry drift** — pure dead-reckoning accumulates heading error over time, causing slight wall smearing when the robot rotates repeatedly. Proper correction requires AMCL or SLAM.
- **Fixed grid origin** — the grid is anchored at the robot's starting position. If the robot travels more than ~5m from start, points fall outside the grid boundary.
- **No path follower** — the A* path is published as `nav_msgs/Path` but a controller (e.g. Pure Pursuit) to actually follow it is not yet implemented.

---

## What's Next

- [ ] Pure Pursuit controller to follow the A* path
- [ ] Inflation layer for safer path clearance from walls
- [ ] Dynamic goal via RViz2 `2D Nav Goal` click
- [ ] Port perception pipeline to CARLA 0.9.14 with 3D LiDAR (`lidar_grid_node.cpp`)