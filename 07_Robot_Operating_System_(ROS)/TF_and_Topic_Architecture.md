---
title: TF and Topic Architecture
description: "How data and geometry flow through a ROS 2 system: topics carry time-stamped messages tagged with a frame, and tf2 keeps the tree of coordinate frames (map, odom, base_link, sensors) so any node can express any measurement in any frame at any time. Covers the REP 105 frame conventions, static and dynamic transforms, a worked transform example, common failure modes and the inspection tools."
aliases:
  - TF
  - tf2
  - Transform Tree
tags:
  - ros
  - ros2
  - tf
  - topics
  - architecture
  - robotics-software
  - spatial-math
  - software-stack
type: concept
level: intermediate
subfield:
  - software
  - estimation-mapping
status: reviewed
author: Jordan_Smith
date: 2025-04-28
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[ROS_2_Overview]]"
  - "[[Homogeneous_Transformation]]"
related:
  - "[[URDF]]"
  - "[[RViz_Tutorial]]"
  - "[[SLAM_with_ROS]]"
  - "[[Nav2_Navigation]]"
  - "[[Localization]]"
key_takeaways:
  - "Every sensor message carries a header with a timestamp and a frame_id; tf2 turns 'where was this measured' into 'where is this in the frame I care about'."
  - "Mobile robots follow REP 105: map → odom → base_link → sensor frames. Localization owns map→odom, odometry owns odom→base_link, robot_state_publisher owns the rest."
  - "Most TF bugs are one of four things: two nodes publishing the same transform, a missing link in the tree, timestamps from different clocks, or asking for a time the buffer does not hold."
sources:
  - "Foote, T. (2013). tf: The transform library. https://ieeexplore.ieee.org/document/6556373"
  - "Meeussen, W. (2010). REP 105: Coordinate Frames for Mobile Platforms. https://www.ros.org/reps/rep-0105.html"
  - "Foote, T. and Purvis, M. (2010). REP 103: Standard Units of Measure and Coordinate Conventions. https://www.ros.org/reps/rep-0103.html"
  - "Open Robotics (2026). geometry2: tf2 for ROS 2. https://github.com/ros2/geometry2"
  - "Open Robotics (2026). ROS 2 documentation source: Introduction to tf2. https://github.com/ros2/ros2_documentation/blob/rolling/source/ROS-Framework/interfaces/Working-with-interfaces/Introduction-To-Tf2/Introduction-To-Tf2.rst"
---

# TF and Topic Architecture

A ROS system moves two kinds of information: **data** (scans, images, poses, commands) on [[ROS_2_Overview|topics]], and **geometry**, meaning where every part of the robot and the world is relative to every other part, through **tf2**. Each relies on the other: every sensor message says *when* and *in which frame* it was measured, and tf2 answers *where that frame was at that time*.

> **Etymology.** *TF* is short for *transform*, from Latin *transformare*, "to change in shape". The second-generation library, written by Tully Foote, is called **tf2**; ROS 2 uses only tf2.

## Topics, briefly

A **topic** is a named, typed stream: publishers send messages, subscribers receive them, and neither knows about the other. Most sensor and state messages begin with a `std_msgs/Header`:

```text
header:
  stamp: {sec: 1763200000, nanosec: 120000000}   # when the measurement was valid
  frame_id: "laser_link"                          # the frame it is expressed in
```

That header is the contract between topics and tf2: data that lacks a correct `stamp` and `frame_id` cannot be placed in the world.

## The transform tree

tf2 keeps a **tree** of coordinate frames. Each edge is a time-stamped rigid transform (translation and rotation) from a parent frame to a child frame. Any node can ask for the transform between *any* two frames at *any* time the buffer holds (10 s by default); tf2 chains and interpolates the edges.

**REP 105**, the standard for mobile platforms:

```
map ──(localization, e.g. AMCL / SLAM)──> odom ──(wheel or visual odometry)──> base_link ──(robot_state_publisher)──> laser_link, camera_link, ...
```

| Frame | Meaning | Property |
|---|---|---|
| `map` | Fixed world frame | Globally accurate, but may jump when localization corrects |
| `odom` | Odometry's world frame | Smooth and continuous, but drifts |
| `base_link` | Attached to the robot body | Moves with the robot |
| sensor frames | Where each sensor is mounted | Usually fixed (static) relative to `base_link` |

Each frame has exactly one parent, so each transform must have exactly **one** publisher. The split between `map` and `odom` exists so that controllers can use the smooth `odom` frame while planners use the accurate `map` frame.

**REP 103** fixes the conventions: SI units, and right-handed axes with x forward, y left, z up for bodies. Camera *optical* frames use z forward, x right, y down, which is why cameras have a separate `*_optical_frame`.

### Static and dynamic transforms

- **Dynamic** transforms (odometry, joints) go on `/tf` at the rate they change.
- **Static** transforms (sensor mounts) go once on `/tf_static`, which uses `transient_local` durability so late-starting nodes still receive them.

Publish a static mount from the command line:

```bash
ros2 run tf2_ros static_transform_publisher --x 0.2 --z 0.1 --yaw 0 --frame-id base_link --child-frame-id laser_link
```

## Worked example: a lidar point into the odometry frame

A lidar is mounted 0.2 m forward and 0.1 m up from `base_link`, facing forward. It sees an obstacle 1.0 m straight ahead: $p_{laser} = (1.0, 0, 0)$.

1. **laser → base_link** (pure translation): $p_{base} = (1.0 + 0.2,\ 0,\ 0 + 0.1) = (1.2,\ 0,\ 0.1)$.
2. **base_link → odom**: the robot is at $(2.0, 1.0)$ in `odom`, heading $\theta = 90°$ (facing +y). Rotate, then translate:
$$\begin{bmatrix}x\\y\end{bmatrix}_{odom} = \begin{bmatrix}\cos\theta & -\sin\theta\\ \sin\theta & \cos\theta\end{bmatrix}\begin{bmatrix}1.2\\0\end{bmatrix} + \begin{bmatrix}2.0\\1.0\end{bmatrix} = \begin{bmatrix}0\\1.2\end{bmatrix} + \begin{bmatrix}2.0\\1.0\end{bmatrix} = \begin{bmatrix}2.0\\2.2\end{bmatrix}$$

So the obstacle is at $(2.0, 2.2, 0.1)$ in `odom`: 1.2 m ahead of the robot's centre, in the +y direction it is facing. In code, one call does both steps:

```python
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # registers PointStamped conversions

self.buffer = Buffer(); self.listener = TransformListener(self.buffer, self)
p_odom = self.buffer.transform(point_stamped_in_laser, 'odom', timeout=rclpy.duration.Duration(seconds=0.1))
```

`transform` uses the point's own timestamp, so a scan taken 50 ms ago is placed where the robot *was* 50 ms ago, not where it is now.

## When it goes wrong

| Symptom | Usual cause | Check |
|---|---|---|
| "Lookup would require extrapolation into the future" | Asking for data newer than the latest transform; clocks differ | `use_sim_time` set the same everywhere; ask for the message's stamp, or wait with a timeout |
| Robot "teleports" or flickers in RViz | Two nodes publishing the same transform (for example the EKF and the driver both publishing odom→base_link) | `ros2 run tf2_ros tf2_monitor` |
| "Frame X does not exist" | A broken chain: missing static transform or URDF link | `ros2 run tf2_tools view_frames` (writes a PDF of the tree) |
| Data off by a fixed rotation | Optical frame confused with body frame | REP 103 conventions |

## Inspection tools

- `ros2 topic list | echo | hz | bw | info -v` show topics, rates, bandwidth and QoS.
- `ros2 run tf2_ros tf2_echo map base_link` prints one transform over time.
- `ros2 run tf2_tools view_frames` draws the whole tree.
- `rqt_graph` draws the node and topic graph.
- [[RViz_Tutorial|RViz]]'s TF display shows frames in 3D; Foxglove offers the same in a browser.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[TF_and_Topic_Architecture]])
```
