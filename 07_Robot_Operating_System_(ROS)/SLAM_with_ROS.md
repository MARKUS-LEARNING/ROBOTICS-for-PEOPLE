---
title: SLAM with ROS
description: Covers the implementation of Simultaneous Localization and Mapping (SLAM) in ROS 2, including SLAM Toolbox, RTABMap, and LiDAR-based approaches, with sensor requirements, TF configuration, and integration with the Nav2 navigation stack.
tags:
  - SLAM
  - ROS2
  - localization
  - mapping
  - mobile-robot
  - LIDAR
  - robotics-software
  - navigation
  - sensor-fusion
  - occupancy-grid
layout: default
category: robotics
author: Jordan_Smith_&_le_Chat
date: 2025-05-02
permalink: /slam_with_ros/
related:
  - "[[Robot_Operating_System_(ROS)]]"
  - "[[ROS_2_Overview]]"
  - "[[Nav2_Navigation]]"
  - "[[TF_and_Topic_Architecture]]"
  - "[[URDF]]"
  - "[[LIDAR]]"
  - "[[Camera_Systems]]"
  - "[[IMU_Sensors]]"
  - "[[Odometry]]"
  - "[[Sensor_Fusion]]"
  - "[[Mapping]]"
  - "[[Localization]]"
  - "[[RViz_Tutorial]]"
  - "[[rosbag2]]"
---

# SLAM with ROS

**Simultaneous Localization and Mapping (SLAM)** is the problem of building a map of an unknown environment while simultaneously tracking the robot's position within it. In ROS 2, SLAM is implemented as a set of nodes that consume sensor data and publish map and pose information to the rest of the system.

SLAM is a foundational capability for autonomous mobile robots — it enables navigation in environments where no pre-built map exists, and provides the map that navigation stacks like [[Nav2_Navigation|Nav2]] use for path planning.

---

## The SLAM Problem

At each timestep $t$, the robot maintains a **belief** over its pose $\mathbf{x}_t$ and the map $\mathbf{m}$, given all observations $\mathbf{z}_{1:t}$ and odometry $\mathbf{u}_{1:t}$:

$$p(\mathbf{x}_t, \mathbf{m} \mid \mathbf{z}_{1:t}, \mathbf{u}_{1:t})$$

SLAM algorithms differ in how they represent and update this belief:

| Approach | Map Representation | Algorithm Family |
|---|---|---|
| Filter-based | Feature map (landmarks) | EKF-SLAM, Particle filter |
| Graph-based | Pose graph with constraints | GraphSLAM, g2o, iSAM |
| Occupancy grid | 2D or 3D grid | GMapping, Cartographer |
| Topological | Node-edge graph | TopoMap |

Modern ROS 2 SLAM packages primarily use **graph-based SLAM** on occupancy grids, which offers better scalability and loop closure handling than filter-based methods.

---

## Required TF Tree

All ROS SLAM packages require a properly configured [[TF_and_Topic_Architecture|TF tree]]. The SLAM node consumes sensor data and **publishes** the `map → odom` transform:

```
map ──(published by SLAM)──► odom ──(published by wheel odometry)──► base_link
                                                                            │
                                                                    ┌───────┴────────┐
                                                                 laser_link     camera_link
                                                         (published by robot_state_publisher from URDF)
```

The SLAM algorithm computes the global pose of the robot in the map frame by aligning sensor data (e.g., LIDAR scans) to the accumulated map and corrects for odometry drift via the `map → odom` transform.

---

## SLAM Toolbox (2D LiDAR SLAM)

**SLAM Toolbox** is the default SLAM package for ROS 2 and is deeply integrated with [[Nav2_Navigation|Nav2]]. It provides:

* **Online SLAM**: Build a map in real time while driving.
* **Lifelong SLAM**: Continue building and updating a map across multiple sessions.
* **Localization-only mode**: Use a pre-built map and localize within it (replacing AMCL).
* **Loop closure**: Detect previously visited places and correct accumulated drift.
* **Serialization**: Save and load maps in a compact binary format.

### Installation and Launch

```bash
sudo apt install ros-$ROS_DISTRO-slam-toolbox

# Online async SLAM (map while navigating)
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=true \
  slam_params_file:=/opt/ros/$ROS_DISTRO/share/slam_toolbox/config/mapper_params_online_async.yaml
```

### Required Topics

| Topic | Message Type | Direction | Description |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | Input | 2D LIDAR scan |
| `/tf` | TF | Input | `odom → base_link` and `base_link → laser_link` |
| `/map` | `nav_msgs/OccupancyGrid` | Output | Built occupancy grid map |
| `/slam_toolbox/pose` | `geometry_msgs/PoseWithCovarianceStamped` | Output | Current estimated pose |
| `/tf` | TF | Output | `map → odom` transform |

### Key Parameters

```yaml
# mapper_params_online_async.yaml (excerpt)
slam_toolbox:
  ros__parameters:
    # Solver
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI

    # Map update
    resolution: 0.05         # meters per grid cell
    max_laser_range: 20.0    # maximum range to use from LIDAR
    minimum_travel_distance: 0.5  # meters between scans added to map
    minimum_travel_heading: 0.5   # radians between scans added to map

    # Loop closure
    loop_search_maximum_distance: 4.0
    do_loop_closing: true
    loop_match_minimum_chain_size: 10

    # Frame names
    map_frame: map
    base_frame: base_footprint
    odom_frame: odom
    scan_topic: /scan
```

### Saving and Loading Maps

```bash
# Save the current map (while SLAM is running)
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'my_map'}}"

# Serialize the SLAM session (save map + all poses for lifelong SLAM)
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: {data: '/home/user/maps/my_session'}}"

# Load a serialized session (for lifelong mapping or localization)
ros2 service call /slam_toolbox/deserialize_map slam_toolbox/srv/DeserializePoseGraph \
  "{filename: {data: '/home/user/maps/my_session'}, initial_pose: ..., match_type: 1}"

# Export a standard ROS map (for Nav2)
ros2 run nav2_map_server map_saver_cli -f /home/user/maps/my_map
```

---

## Cartographer (2D/3D LiDAR SLAM)

**Cartographer** (Google) supports both 2D and 3D SLAM using a scan matching approach with submap-based loop closure. It is particularly strong for 3D environments using 3D LiDAR (Velodyne, Ouster).

```bash
sudo apt install ros-$ROS_DISTRO-cartographer ros-$ROS_DISTRO-cartographer-ros

# 2D SLAM
ros2 launch cartographer_ros demo_revo_lds.launch.py

# 3D SLAM with 3D LiDAR
ros2 launch cartographer_ros demo_backpack_3d.launch.py
```

Cartographer configuration is split into:
* **`.lua` configuration file**: Algorithm parameters (voxel filter size, scan matching, loop closure).
* **`.launch.py`**: Node launch with remappings for sensor topics.

---

## RTABMap (Visual/LiDAR SLAM)

**RTABMap** (Real-Time Appearance-Based Mapping) supports **visual SLAM** (RGB-D, stereo cameras) and **LiDAR SLAM**, as well as sensor fusion combinations. It uses a memory management system to enable large-scale mapping and provides 3D point cloud maps in addition to 2D occupancy grids.

```bash
sudo apt install ros-$ROS_DISTRO-rtabmap-ros

# RGB-D SLAM (RealSense D435 example)
ros2 launch rtabmap_launch rtabmap.launch.py \
  rgb_topic:=/camera/color/image_raw \
  depth_topic:=/camera/depth/image_rect_raw \
  camera_info_topic:=/camera/color/camera_info \
  frame_id:=base_link

# LiDAR + camera fusion
ros2 launch rtabmap_launch rtabmap.launch.py \
  lidar3d:=true \
  lidar3d_topic:=/velodyne_points \
  rgb_topic:=/camera/color/image_raw \
  depth_topic:=/camera/depth/image_rect_raw
```

RTABMap outputs:
* `/rtabmap/map` — 3D point cloud of the environment.
* `/rtabmap/grid_map` — 2D occupancy grid for Nav2.
* `/tf` — `map → odom` transform.

---

## LiDAR Odometry: LOAM / LIO-SAM / KISS-ICP

For 3D outdoor environments or when wheel odometry is unavailable or unreliable (e.g., legged robots), **LiDAR odometry** algorithms estimate odometry from scan-to-scan point cloud alignment:

| Package | Approach | Characteristics |
|---|---|---|
| **LOAM** | Feature-based | Pioneering, heavy, targets planar/edge features |
| **LeGO-LOAM** | Lightweight LOAM variant | Ground-segmented, designed for ground vehicles |
| **LIO-SAM** | LiDAR + IMU tightly coupled | High accuracy, requires IMU |
| **KISS-ICP** | ICP with adaptive threshold | Simple, robust, real-time on all platforms |

KISS-ICP example (ROS 2):

```bash
sudo apt install ros-$ROS_DISTRO-kiss-icp

ros2 launch kiss_icp odometry.launch.py \
  topic:=/velodyne_points \
  visualize:=true
```

KISS-ICP outputs odometry on `/kiss/odometry` and the `odom → base_link` TF — which can then be consumed by SLAM Toolbox or another loop-closure system.

---

## Integration with Nav2

SLAM Toolbox integrates directly with Nav2 in **slam mode**:

```bash
# Launch Nav2 with SLAM (no pre-built map needed)
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  slam:=True

# Or launch SLAM Toolbox alongside Nav2 separately
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true &
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true
```

The SLAM node publishes `/map` and the `map → odom` TF, which Nav2's costmaps and planner automatically consume. Once mapping is complete, save the map with `map_saver_cli` and switch to localization-only mode for future runs.

---

## Visualizing SLAM in RViz

Add the following displays to [[RViz_Tutorial|RViz]] to monitor SLAM:

```
Displays:
├── Map          → Topic: /map             (occupancy grid)
├── TF           → (entire TF tree)
├── LaserScan    → Topic: /scan            (real-time sensor)
├── RobotModel   → (from URDF)
└── Path         → Topic: /slam_toolbox/graph  (pose graph, optional)
```

Fixed Frame: `map`

---

## SLAM with rosbag2

A common development workflow uses [[rosbag2]] to decouple data collection from algorithm development:

```bash
# 1. Record sensor data on the robot
ros2 bag record /scan /odom /tf /tf_static -o slam_session_001

# 2. Replay at desk and run SLAM
ros2 bag play slam_session_001/ --clock --rate 0.5 &
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true

# 3. Tune SLAM parameters and repeat from step 2 — no robot needed
```

---

## Dataview Plugin Features

```dataview
LIST FROM #SLAM OR #mapping WHERE contains(file.outlinks, [[SLAM_with_ROS]])
```

```dataview
TABLE title, description FROM #ROS WHERE contains(file.tags, "SLAM")
```
