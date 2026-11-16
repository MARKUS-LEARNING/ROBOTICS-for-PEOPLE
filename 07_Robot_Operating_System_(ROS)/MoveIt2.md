---
title: MoveIt 2
description: "MoveIt 2 is ROS 2's motion-planning framework for robot arms: it plans collision-free trajectories, solves inverse kinematics, manages the planning scene and executes motions through ros2_control. Covers the architecture, SRDF, planners (OMPL, Pilz, STOMP, CHOMP), MoveItPy, Task Constructor, versions per ROS distribution in 2026, MoveIt Pro, and GPU motion generation (cuRobo)."
aliases:
  - MoveIt
  - MoveItPy
tags:
  - moveit
  - motion-planning
  - ros2
  - manipulation
  - kinematics
  - collision-detection
  - robotics-software
  - manipulator-arm
type: tool
level: intermediate
subfield:
  - software
  - planning
  - manipulation
status: reviewed
author: Jordan_Smith
date: 2025-04-28
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[ROS_2_Overview]]"
  - "[[URDF]]"
  - "[[Path_Planning]]"
related:
  - "[[ros2_control]]"
  - "[[Inverse_Kinematics]]"
  - "[[Manipulation]]"
  - "[[RViz_Tutorial]]"
  - "[[Vision-Language-Action_Models]]"
key_takeaways:
  - "MoveIt turns 'put the gripper here without hitting anything' into a time-parameterised joint trajectory, and sends it to a ros2_control trajectory controller."
  - "Versions follow ROS distributions: MoveIt 2.15 for Lyrical (recommended in 2026), 2.12 for Jazzy, 2.5 for Humble."
  - "Learned policies and MoveIt coexist: policies handle contact-rich, perception-driven skills; MoveIt handles collision-free transit, reachability and safety checks."
sources:
  - "Coleman, D., Sucan, I., Chitta, S. and Correll, N. (2014). Reducing the Barrier to Entry of Complex Robotic Software: a MoveIt! Case Study. https://arxiv.org/abs/1404.3785"
  - "PickNik Robotics (2026). MoveIt Motion Planning Framework. https://moveit.ai/"
  - "moveit (2026). moveit2: MoveIt for ROS 2. https://github.com/moveit/moveit2"
  - "Qualcomm (2026). Qualcomm to acquire PickNik to advance the future of open robotics. https://www.qualcomm.com/news/releases/2026/09/qualcomm-to-acquire-picknik-to-advance-the-future-of-open-roboti"
  - "Sundaralingam, B. et al. (2023). cuRobo: Parallelized Collision-Free Minimum-Jerk Robot Motion Generation. https://arxiv.org/abs/2310.17274"
---

# MoveIt 2

**MoveIt 2** is the standard motion planning framework for robot manipulation in ROS 2. It provides a complete pipeline for planning collision-free trajectories for robot arms, grippers, and other articulated mechanisms, from specifying a target pose through to executing the trajectory on real or simulated hardware via [[ros2_control]].

MoveIt 2 is used in industrial automation, research manipulation, surgical robotics, and service robots. It integrates with RViz for visualization, Gazebo for simulation, and supports a wide variety of kinematics solvers and motion planners as interchangeable plugins.

---

## Architecture Overview

```
┌──────────────────────────────────────────────────────────┐
│                 Move Group Node (Central Hub)             │
│                                                          │
│  ┌─────────────┐  ┌──────────────┐  ┌────────────────┐  │
│  │   Planning  │  │  Kinematics  │  │   Collision    │  │
│  │  Pipeline   │  │   Plugin     │  │   Checking     │  │
│  │ (OMPL, etc.)│  │ (KDL, IKFast)│  │   (FCL, etc.)  │  │
│  └──────┬──────┘  └──────┬───────┘  └────────────────┘  │
│         │                │                               │
│  ┌──────▼──────────────────────────────────────────────┐ │
│  │              Planning Scene Monitor                  │ │
│  │  (robot state, collision objects, octomap, TF)      │ │
│  └─────────────────────────────────────────────────────┘ │
└─────────────────────┬────────────────────────────────────┘
                      │ Action / Service interfaces
          ┌───────────▼───────────────────────────┐
          │    MoveGroupInterface (C++ / Python)   │
          │    MoveIt Task Constructor             │
          │    RViz MoveIt Plugin                  │
          └───────────────────────────────────────┘
```

---

## Configuration Files

### URDF

The robot's physical description is provided via [[URDF]] (or xacro). This defines the kinematic chain, link geometry, joint limits, and inertial properties.

### SRDF — Semantic Robot Description Format

The **SRDF** supplements the URDF with semantic information that MoveIt needs:

```xml
<?xml version="1.0"?>
<robot name="my_arm">

  <!-- Planning groups define which joints MoveIt plans for -->
  <group name="arm">
    <chain base_link="base_link" tip_link="tool0"/>
  </group>

  <group name="gripper">
    <joint name="finger_joint_1"/>
    <joint name="finger_joint_2"/>
  </group>

  <!-- Named joint states (preset configurations) -->
  <group_state name="home" group="arm">
    <joint name="shoulder_joint" value="0"/>
    <joint name="elbow_joint" value="0"/>
    <joint name="wrist_joint" value="0"/>
  </group_state>

  <group_state name="ready" group="arm">
    <joint name="shoulder_joint" value="0"/>
    <joint name="elbow_joint" value="-1.57"/>
    <joint name="wrist_joint" value="0"/>
  </group_state>

  <!-- End effectors -->
  <end_effector name="hand" parent_link="tool0" group="gripper"/>

  <!-- Self-collision exclusions (pairs always in collision in valid configs) -->
  <disable_collisions link1="base_link" link2="shoulder_link" reason="Adjacent"/>
  <disable_collisions link1="shoulder_link" link2="upper_arm_link" reason="Adjacent"/>
  <disable_collisions link1="upper_arm_link" link2="forearm_link" reason="Adjacent"/>

</robot>
```

The **MoveIt Setup Assistant** (a GUI tool) generates the SRDF automatically by analyzing the URDF and running collision matrix computation.

---

## Planning Pipeline

MoveIt's planning pipeline is a sequence of pluggable stages:

```
Goal specification → [Pre-processing] → Planner → [Post-processing] → Trajectory
```

### Motion Planners (via OMPL)

MoveIt integrates with the **Open Motion Planning Library (OMPL)** as its primary planning backend. Common planners:

| Planner | Type | Best For |
|---|---|---|
| RRTConnect | Bidirectional RRT | Fast, general-purpose |
| RRT* | Asymptotically optimal RRT | Higher quality paths (slower) |
| PRM / PRM* | Probabilistic Roadmap | Repeated queries in static environments |
| STOMP | Stochastic Trajectory Optimization | Smooth, constraint-satisfying trajectories |
| CHOMP | Gradient-based optimization | Fast trajectory refinement |
| PILZ | Deterministic Cartesian planners | Industrial LIN/CIRC/PTP motions |

### Kinematics Solvers

| Solver | Type | Notes |
|---|---|---|
| KDL | Numerical (Jacobian-based) | Default, works for any chain |
| IKFast | Analytical | Precomputed, extremely fast, robot-specific |
| TracIK | Numerical | Faster and more robust than KDL |
| BioIK | Bio-inspired optimization | Handles redundant DOF, constraints |

### Collision Checking

MoveIt uses the **Planning Scene** to track:
* Robot links (from URDF geometry).
* Static world objects (boxes, cylinders, meshes added programmatically).
* Octomap (3D voxel map from point cloud sensors).

Collision checking uses the **Flexible Collision Library (FCL)** for continuous collision detection between mesh objects.

---

## MoveGroupInterface (Python)

The `MoveGroupInterface` is the primary API for commanding MoveIt programmatically:

```python
import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from geometry_msgs.msg import Pose
import math

rclpy.init()

# Initialize MoveItPy — loads robot model, planning scene, etc.
moveit = MoveItPy(node_name="moveit_py_example")
arm = moveit.get_planning_component("arm")

# ----- 1. Plan to a named joint configuration -----
arm.set_start_state_to_current_state()
arm.set_goal_state(configuration_name="home")
plan_result = arm.plan()
if plan_result:
    moveit.execute(plan_result.trajectory, controllers=["joint_trajectory_controller"])

# ----- 2. Plan to a Cartesian (end-effector) pose -----
from moveit.core.robot_state import RobotState
from moveit_msgs.msg import Constraints

target_pose = Pose()
target_pose.position.x = 0.4
target_pose.position.y = 0.1
target_pose.position.z = 0.5
target_pose.orientation.w = 1.0  # identity rotation

arm.set_start_state_to_current_state()
arm.set_goal_state(pose_stamped_msg=target_pose, pose_link="tool0")

plan_result = arm.plan()
if plan_result:
    moveit.execute(plan_result.trajectory, controllers=["joint_trajectory_controller"])

rclpy.shutdown()
```

---

## MoveIt Task Constructor (MTC)

For complex multi-step manipulation tasks (pick-and-place, assembly), the **MoveIt Task Constructor** provides a higher-level planning framework that composes a task as a sequence of **stages**:

```
Task: Pick an object and place it
  ├── Stage: Move to pre-grasp pose (freespace motion)
  ├── Stage: Approach object (Cartesian, linear)
  ├── Stage: Close gripper (gripper action)
  ├── Stage: Attach object to gripper (scene modification)
  ├── Stage: Retreat from grasp (Cartesian, linear)
  ├── Stage: Move to pre-place pose (freespace motion)
  ├── Stage: Place object (Cartesian, linear)
  ├── Stage: Open gripper (gripper action)
  └── Stage: Detach object from gripper
```

MTC handles the interface between stages automatically, propagating state forward and backward to find feasible solutions for the whole task.

---

## Adding Collision Objects to the Planning Scene

```python
from moveit.core.planning_scene_monitor import PlanningSceneMonitor
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

# Add a box obstacle to the planning scene
collision_object = CollisionObject()
collision_object.header.frame_id = "base_link"
collision_object.id = "table"

box = SolidPrimitive()
box.type = SolidPrimitive.BOX
box.dimensions = [0.8, 1.2, 0.05]  # x, y, z

box_pose = Pose()
box_pose.position.x = 0.5
box_pose.position.z = 0.4

collision_object.primitives = [box]
collision_object.primitive_poses = [box_pose]
collision_object.operation = CollisionObject.ADD

# Apply to planning scene via PlanningSceneInterface
from moveit_py.planning_scene_interface import PlanningSceneInterface
psi = PlanningSceneInterface()
psi.apply_collision_object(collision_object)
```

---

## Cartesian Path Planning

For tasks requiring the end effector to follow a straight line (e.g., welding, painting, surface scanning):

```python
# Plan a Cartesian path through a sequence of waypoints
waypoints = []

pose = Pose()
pose.position.x = 0.3
pose.position.z = 0.5
pose.orientation.w = 1.0

for dx in [0.0, 0.1, 0.2, 0.3]:
    pose.position.x += dx
    waypoints.append(copy.deepcopy(pose))

# fraction: proportion of path successfully planned (1.0 = fully planned)
(plan, fraction) = move_group.compute_cartesian_path(
    waypoints,
    eef_step=0.01,        # 1 cm resolution
    jump_threshold=0.0    # disable jump threshold
)
```

The Cartesian path is parameterized such that the end effector moves linearly through joint space, constraining it to follow a straight-line Cartesian trajectory.

---

## Launching MoveIt 2

```bash
# A UR5e with mock hardware (no robot needed): start the driver, then MoveIt with RViz
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=0.0.0.0 use_mock_hardware:=true launch_rviz:=false
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true

# Your own robot: generate a config package with the Setup Assistant, then run its demo
ros2 launch moveit_setup_assistant setup_assistant.launch.py
ros2 launch my_robot_moveit_config demo.launch.py
```

---

## Relationship with ros2_control

MoveIt 2 and [[ros2_control]] work together as the standard manipulation stack:

* MoveIt 2 computes a **trajectory** (sequence of joint positions over time).
* The trajectory is sent as a `FollowJointTrajectory` action goal to the **JointTrajectoryController** (from `ros2_controllers`).
* The JointTrajectoryController interpolates the trajectory and sends position/velocity commands to the hardware via ros2_control interfaces.
* The hardware interface translates commands to motor signals and reads back encoder data.

---

## Versions and support (November 2026)

| ROS 2 distribution | MoveIt | Status |
|---|---|---|
| Lyrical (LTS) | 2.15 | Latest stable, recommended |
| Kilted | 2.14 | Maintained |
| Jazzy (LTS) | 2.12 | Maintained |
| Humble (LTS) | 2.5 | Maintained |
| Noetic (ROS 1) | 1.1 | End of life |

**MoveIt Pro** is PickNik Robotics' commercially supported product built on MoveIt, adding behavior-tree task authoring, a web interface and teleoperation tools. In September 2026 Qualcomm announced it would acquire PickNik; the open-source MoveIt project continues under the same maintainers' stewardship.

## Planning on GPUs and alongside learned policies

- **GPU motion generation.** NVIDIA's cuRobo (2023) runs many trajectory optimisations in parallel on a GPU, producing collision-free, minimum-jerk motions in tens of milliseconds. It is packaged for ROS as Isaac ROS cuMotion, which includes a MoveIt planner plugin. Fast planning makes reactive replanning practical in cluttered scenes.
- **Division of labour with learned policies.** A typical 2026 manipulation stack uses MoveIt (or cuRobo) to move the arm safely to a pre-grasp pose, then hands control to a learned policy ([[Diffusion_Policy]] or a [[Vision-Language-Action_Models|VLA]]) for the contact-rich part, then returns to MoveIt for transit. MoveIt's planning scene still provides the collision checks and joint limits that learned policies do not guarantee.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[MoveIt2]])
```
