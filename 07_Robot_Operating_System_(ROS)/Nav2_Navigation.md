---
title: Nav2 Navigation Stack
description: "Nav2 is ROS 2's navigation system: it takes a map, a pose estimate and a goal, and drives a mobile robot there safely. Covers costmaps, planners (NavFn, Smac, Theta*), controllers (DWB, Regulated Pure Pursuit, MPPI, Graceful), behavior trees, recoveries, the newer servers (docking, routes, following, coverage, collision monitor), configuration, and how learned perception plugs in."
aliases:
  - Nav2
  - Navigation2
tags:
  - nav2
  - navigation
  - ros2
  - mobile-robot
  - path-planning
  - costmap
  - behavior-tree
  - robotics-software
type: tool
level: intermediate
subfield:
  - software
  - planning
  - mobile-robots
status: reviewed
author: Jordan_Smith
date: 2025-04-28
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[ROS_2_Overview]]"
  - "[[TF_and_Topic_Architecture]]"
  - "[[Path_Planning]]"
related:
  - "[[SLAM_with_ROS]]"
  - "[[Localization]]"
  - "[[RViz_Tutorial]]"
  - "[[Gazebo_Simulator]]"
  - "[[Model_Predictive_Control_(MPC)]]"
key_takeaways:
  - "Nav2 splits navigation into servers: a planner finds a path on the global costmap, a controller tracks it on the local costmap, and a behavior tree decides what to do, including recoveries."
  - "Choose plugins by robot: Smac Hybrid-A* or State Lattice for car-like and legged bases, MPPI or Regulated Pure Pursuit for most differential-drive robots."
  - "Since 2024 Nav2 has grown task servers (docking, routes on a graph, dynamic object following, coverage) and a collision monitor that acts as a last safety layer."
sources:
  - "Macenski, S., Martín, F., White, R. and Ginés Clavero, J. (2020). The Marathon 2: A Navigation System. https://arxiv.org/abs/2003.00368"
  - "Macenski, S., Moore, T., Lu, D. V., Merzlyakov, A. and Ferguson, M. (2023). From the desks of ROS maintainers: A survey of modern & capable mobile robotics algorithms in the robot operating system 2. https://arxiv.org/abs/2307.15236"
  - "Open Navigation (2026). Nav2 documentation. https://docs.nav2.org/"
  - "ros-navigation (2026). navigation2. https://github.com/ros-navigation/navigation2"
  - "Williams, G., Aldrich, A. and Theodorou, E. (2015). Model Predictive Path Integral Control using Covariance Variable Importance Sampling. https://arxiv.org/abs/1509.01149"
---

# Nav2 — ROS 2 Navigation Stack

**Nav2** (Navigation2) is the successor to the ROS 1 Navigation Stack and the standard framework for autonomous mobile robot navigation in ROS 2. It provides a complete, modular pipeline for taking a robot from a current pose to a goal pose while avoiding obstacles, using a configurable system of planners, controllers, costmaps, behavior trees, and recovery behaviors.

Nav2 is used in ground robots, logistics AMRs, service robots, outdoor vehicles, and research platforms. Its behavior tree-based architecture makes it highly configurable without requiring code changes.

---

## System Architecture

Nav2 operates as an orchestrated collection of ROS 2 [[Custom_Packages_and_Nodes|servers]], each responsible for a specific function. The **BT Navigator** is the central coordinator, using a **Behavior Tree (BT)** to decide which servers to call and in what sequence.

```
               ┌──────────────────────────────────┐
               │          BT Navigator            │  ← drives the overall task
               │  (NavigateToPose Action Server)  │
               └────────────┬─────────────────────┘
                            │ orchestrates via Behavior Tree
          ┌─────────────────┼──────────────────────────┐
          ▼                 ▼                          ▼
  ┌───────────────┐ ┌───────────────┐      ┌────────────────────┐
  │ Planner Server│ │Controller     │      │ Recovery Server    │
  │ (global plan) │ │ Server (local │      │ (spin, backup,     │
  │               │ │  control)     │      │  wait, clear maps) │
  └──────┬────────┘ └──────┬────────┘      └────────────────────┘
         │                 │
  ┌──────▼──────┐   ┌──────▼──────┐
  │ Global      │   │ Local       │
  │ Costmap     │   │ Costmap     │
  └─────────────┘   └─────────────┘
         ▲                 ▲
         └────── Sensors (LIDAR, depth cameras, sonar) ──────┘
```

---

## Core Components

### 1. Costmaps

A **costmap** is a 2D grid map where each cell encodes the cost (difficulty or danger) of the robot occupying that location. Nav2 uses two costmaps:

* **Global Costmap**: Built from the static map (from SLAM or a pre-built map). Used by the global planner to find a high-level path.
* **Local Costmap**: A smaller, rolling window around the robot, updated in real time from sensor data. Used by the local controller to avoid dynamic obstacles.

Costmap layers are composable plugins:

| Layer | Purpose |
|---|---|
| `StaticLayer` | Inflates the pre-built occupancy grid map |
| `InflationLayer` | Adds cost around obstacles proportional to proximity |
| `ObstacleLayer` | Marks/clears cells based on 2D LIDAR |
| `VoxelLayer` | Marks/clears cells based on 3D point clouds |

The cost value $c$ in the inflation layer decays with distance $d$ from an obstacle:

$$c(d) = \text{INSCRIBED\_COST} \cdot e^{-\beta (d - r_{\text{inscribed}})}$$

where $r_{\text{inscribed}}$ is the robot's inscribed radius and $\beta$ is the decay factor.

### 2. Global Planner (Planner Server)

Computes a collision-free path from the robot's current pose to the goal pose using the global costmap. Common plugins:

* **NavFn** (Dijkstra/A*): Classic grid-based planner, robust and well-tested.
* **Smac Planner 2D**: Improved A* with kinematic constraints.
* **Smac Planner Hybrid-A\***: Kinodynamically feasible paths for Ackermann (car-like) robots.
* **Smac State Lattice**: Feasible paths from precomputed motion primitives, for any drive type including legged and omnidirectional bases.
* **Theta\***: Any-angle paths that are shorter than grid-constrained A*.

The planner produces a `nav_msgs/Path` that is passed to the controller.

### 3. Local Controller (Controller Server)

Tracks the global path in real time, reacting to dynamic obstacles detected in the local costmap and generating velocity commands (`geometry_msgs/Twist`) on `/cmd_vel`. Common plugins:

* **DWB (Dynamic Window Approach)**: Samples feasible velocity trajectories, scores them, and selects the best. Successor to DWA from ROS 1.
* **RPP (Regulated Pure Pursuit)**: Follows the path by steering toward a look-ahead point; velocity regulated near obstacles.
* **MPPI (Model Predictive Path Integral)**: Sampling-based [[Model_Predictive_Control_(MPC)|MPC]]. It rolls out thousands of noisy control sequences on the CPU (vectorised), scores them with plugin "critics" and averages the best. It is the most capable general-purpose controller in Nav2.
* **Graceful Controller**: Smooth, pose-following control law, good for docking approaches and final alignment.
* **Rotation Shim**: Rotates in place toward the path before handing over to another controller.

(TEB, the Timed Elastic Band planner popular in ROS 1, is not part of Nav2.)

### 4. Behavior Tree Navigator (BT Navigator)

The **BT Navigator** is the high-level task coordinator. It reads an XML behavior tree file and executes it to complete a navigation task. A simplified default BT looks like:

```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1.0">
          <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
        </RateController>
        <FollowPath path="{path}" controller_id="FollowPath"/>
      </PipelineSequence>
      <ReactiveFallback name="RecoveryFallback">
        <GoalUpdated/>
        <RoundRobin name="RecoveryActions">
          <Sequence name="ClearingActions">
            <ClearEntireCostmap name="ClearLocalCostmap" service_name="local_costmap/clear_entirely_local_costmap"/>
            <ClearEntireCostmap name="ClearGlobalCostmap" service_name="global_costmap/clear_entirely_global_costmap"/>
          </Sequence>
          <Spin spin_dist="1.57"/>
          <Wait wait_duration="5"/>
          <BackUp backup_dist="0.15" backup_speed="0.025"/>
        </RoundRobin>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>
```

### 5. Recovery Behaviors

When navigation fails (e.g., the robot is stuck, the planner cannot find a path), Nav2 executes recovery actions:

* **Spin**: Rotate in place to clear sensor occlusions or get unstuck.
* **Back Up**: Move backward a small distance.
* **Wait**: Pause and wait for dynamic obstacles to clear.
* **Clear Costmap**: Erase stale obstacle markings from the costmap.

### 6. Map Server and AMCL

* **Map Server**: Loads a pre-built `.pgm`/`.yaml` occupancy grid map and serves it as a latched ROS topic.
* **AMCL (Adaptive Monte Carlo Localization)**: Particle filter-based localization. Estimates robot pose within the provided static map using LIDAR scan matching against the map:

$$p(\mathbf{x}_t | \mathbf{z}_{1:t}, \mathbf{u}_{1:t}) \propto p(\mathbf{z}_t | \mathbf{x}_t) \cdot \int p(\mathbf{x}_t | \mathbf{u}_t, \mathbf{x}_{t-1}) \cdot p(\mathbf{x}_{t-1} | \mathbf{z}_{1:t-1}, \mathbf{u}_{1:t-1}) \, d\mathbf{x}_{t-1}$$

For fully autonomous mapping and localization, [[SLAM_with_ROS|SLAM Toolbox]] can replace both the Map Server and AMCL.

---

## Required TF Frames

Nav2 requires a properly configured [[TF_and_Topic_Architecture|TF tree]]:

```
map → odom → base_link → [sensor frames]
```

* `map → odom`: Published by AMCL or SLAM (accounts for global localization correction).
* `odom → base_link`: Published by wheel odometry or IMU. Continuous, drift-prone.
* `base_link → sensor_frames`: Published by `robot_state_publisher` from the [[URDF]].

---

## Launching Nav2

```bash
# Full navigation with a pre-built map (localization only)
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=true \
  map:=/path/to/map.yaml

# Navigation with simultaneous mapping (SLAM Toolbox)
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  slam:=true

# Send a navigation goal from command line
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}"
```

---

## Configuration

Nav2 is configured via YAML parameter files passed to nodes at launch:

```yaml
# nav2_params.yaml (excerpt)
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.5
      lookahead_dist: 0.6
      max_angular_decel: 3.2

planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: true

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      robot_radius: 0.22
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
```

---

## Waypoint Following and Multi-Goal Navigation

Nav2 supports navigating through a sequence of waypoints:

```python
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped

navigator = BasicNavigator()

# Define waypoints
waypoints = []
for x, y in [(1.0, 0.0), (2.0, 1.5), (0.5, 3.0)]:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.w = 1.0
    waypoints.append(pose)

navigator.followWaypoints(waypoints)

while not navigator.isTaskComplete():
    feedback = navigator.getFeedback()
    print(f'Executing waypoint {feedback.current_waypoint + 1}/{len(waypoints)}')
```

---

## Worked example: how many rollouts MPPI evaluates

MPPI's defaults are 1,000 sampled trajectories (`batch_size`) of 56 points (`time_steps`) spaced 0.05 s apart (`model_dt`), so each candidate looks 56 × 0.05 = 2.8 s ahead. At the 30 Hz controller rate used in the example configuration, MPPI simulates 1,000 × 56 = 56,000 motion steps per cycle, or about 1.7 million per second. Each step is a few multiply-adds for a differential-drive model, which is why the maintainers report 50+ Hz on a 4th-generation Intel i5. Doubling the batch to 2,000 smooths the result but roughly doubles the cost, so `batch_size` is the first parameter to adjust on small computers.

## Beyond point-to-point: task servers (2024–2026)

| Server | What it does |
|---|---|
| **Docking Server** | Detects a charging dock and performs the precise approach and contact |
| **Route Server** | Plans on a hand-made graph of lanes, such as warehouse aisles or sidewalks, instead of free space |
| **Following Server** | Follows a moving target such as a person or another robot |
| **Coverage Server** | Plans complete-coverage paths for mowing, cleaning and agriculture |
| **Collision Monitor** | Independent safety layer: slows or stops the robot when anything enters configured zones, whatever the planner says |

## Learned perception in Nav2

The costmap is where learning enters most navigation stacks today:
- segmentation or detection networks mark people, glass or drivable terrain as costmap layers;
- learned depth turns a single camera into an obstacle sensor.

The Nav2 documentation added tutorials on depth estimation and semantic segmentation in 2025–2026. The planners and controllers themselves mostly remain classical and verifiable, with learned components supplying *what is where*.

## Distributions

Nav2 is released for every ROS 2 distribution. In November 2026, Lyrical and Jazzy are in active support, Kilted and Humble are maintained, and Iron is end of life. Match the Nav2 documentation version to your distribution, because parameter names change between releases; each release has a migration guide.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Nav2_Navigation]])
```
