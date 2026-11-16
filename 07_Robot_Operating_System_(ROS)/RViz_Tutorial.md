---
title: RViz Tutorial
description: "RViz is ROS 2's 3D visualiser: it shows the robot model, coordinate frames, sensor data, maps, plans and custom markers in one scene, so you can see what the robot believes. Covers setting the fixed frame, the displays you will use most, a debugging recipe, markers from your own nodes, and the 2026 alternatives (Foxglove, Rerun) for remote and recorded data."
aliases:
  - RViz
  - RViz2
tags:
  - rviz
  - ros
  - visualization
  - debugging
  - robotics-software
  - tf
  - software-stack
type: tutorial
level: beginner
subfield:
  - software
status: reviewed
author: Jordan_Smith
date: 2025-04-28
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[ROS_2_Overview]]"
  - "[[TF_and_Topic_Architecture]]"
related:
  - "[[URDF]]"
  - "[[Nav2_Navigation]]"
  - "[[SLAM_with_ROS]]"
  - "[[rosbag2]]"
  - "[[MoveIt2]]"
key_takeaways:
  - "RViz draws what the robot believes, not what is true: every display is data from a topic placed in the scene through tf2."
  - "Set the Fixed Frame first (map, odom or base_link). Most 'nothing shows up' problems are a wrong fixed frame, a missing transform or a QoS mismatch."
  - "For remote robots, browsers and recorded MCAP data, Foxglove and Rerun complement RViz; RViz remains the tool integrated with MoveIt and Nav2 interaction."
sources:
  - "Open Robotics (2026). rviz: ROS 3D visualisation tool for ROS 2. https://github.com/ros2/rviz"
  - "Open Robotics (2026). ROS 2 Lyrical Luth release notes (RViz does not yet draw URDF capsules). https://github.com/ros2/ros2_documentation/blob/rolling/source/Releases/Release-Lyrical-Luth.rst"
  - "Foxglove (2026). Foxglove documentation. https://docs.foxglove.dev/docs"
  - "Rerun (2026). Rerun: an SDK for logging and visualising multimodal data. https://github.com/rerun-io/rerun"
  - "Open Robotics (2026). rviz for ROS 1 (original authors D. Hershberger, D. Gossow and J. Faust). https://github.com/ros-visualization/rviz"
---

# RViz Tutorial

**RViz** (ROS visualisation; the ROS 2 program is `rviz2`) is a 3D viewer for everything a ROS system knows: the robot model and its joint angles, coordinate frames, laser scans, point clouds, camera images, maps, planned paths and custom shapes your own nodes publish. It does not simulate anything. It shows what the robot *believes*, which makes it the fastest way to find out why the robot is wrong.

> **Etymology.** *RViz* is *ROS* + *viz*, short for *visualisation*, from Latin *visus*, "sight". It was created at Willow Garage around 2009, by Dave Hershberger, David Gossow and Josh Faust.

## Start it

```bash
ros2 run rviz2 rviz2                         # empty scene
ros2 run rviz2 rviz2 -d my_robot/config/view.rviz   # load a saved layout
```

Most robot packages start RViz from a [[Launch_Files|launch file]] with a saved `.rviz` configuration.

## The one setting that matters first: Fixed Frame

Everything in the scene is transformed into the **Fixed Frame** (Global Options):
- `map`, for navigation and SLAM: the world stays still and the robot moves.
- `odom`, for odometry checks: smooth, but drifts.
- `base_link`, for sensor mounting checks: the robot stays still and the world moves.

If the chosen frame is not connected to a display's frame through tf2, that display shows an error and draws nothing. See [[TF_and_Topic_Architecture]].

## Displays you will use most

| Display | Message type | Use |
|---|---|---|
| **RobotModel** | `robot_description` + TF | Is the [[URDF]] right; do joints move correctly |
| **TF** | `/tf`, `/tf_static` | Frame tree, axes (red x, green y, blue z) |
| **LaserScan** | `sensor_msgs/LaserScan` | 2D lidar |
| **PointCloud2** | `sensor_msgs/PointCloud2` | 3D lidar, depth cameras |
| **Image** / **Camera** | `sensor_msgs/Image` | Raw image, or image overlaid on the 3D scene |
| **Map** | `nav_msgs/OccupancyGrid` | SLAM maps, costmaps |
| **Path**, **Pose**, **Odometry** | `nav_msgs`, `geometry_msgs` | Plans and pose estimates |
| **Marker** / **MarkerArray** | `visualization_msgs` | Anything your node wants to draw |

The toolbar's **2D Pose Estimate** and **Nav2 Goal** tools publish an initial pose and a navigation goal; the MoveIt **MotionPlanning** panel plans arm motions interactively (see [[MoveIt2]]).

## Debugging recipe: "my lidar data is in the wrong place"

1. Set Fixed Frame to `base_link`; add **TF** and **RobotModel**. Is `laser_link` where the real lidar is mounted? If not, fix the URDF or static transform.
2. Add **LaserScan**. Nothing drawn? Check `ros2 topic info -v /scan`: the driver usually publishes best-effort, and the display's reliability policy must match.
3. Walls drawn behind the robot? The scan's `frame_id` or the mount's yaw is wrong (a 180° error is common).
4. Switch Fixed Frame to `odom` and drive slowly. Walls should stay still; if they smear when turning, check wheel odometry and timestamps.

## Draw your own data with markers

```python
from visualization_msgs.msg import Marker
m = Marker()
m.header.frame_id = 'base_link'; m.header.stamp = self.get_clock().now().to_msg()
m.ns, m.id, m.type, m.action = 'goal', 0, Marker.SPHERE, Marker.ADD
m.pose.position.x = 1.5; m.pose.orientation.w = 1.0
m.scale.x = m.scale.y = m.scale.z = 0.2
m.color.r, m.color.a = 1.0, 1.0            # alpha 0 would make it invisible
self.marker_pub.publish(m)
```

Markers are the quickest way to see what a planner, a detector or a learned policy is "thinking": draw candidate grasps, predicted trajectories or attention points in the same scene as the sensor data.

## RViz and the alternatives in 2026

| Tool | Strength | Typical use |
|---|---|---|
| **RViz** | Native, interactive, integrated with MoveIt and Nav2 tools | Development on the robot's network |
| **Foxglove** | Browser or desktop; opens MCAP recordings; connects to live robots over a WebSocket bridge | Remote robots, team debugging, recorded data |
| **Rerun** | Logs and replays multimodal data (images, 3D, time series) from any Python, C++ or Rust program | Machine-learning and perception pipelines, including non-ROS code |

They are complementary: many teams use RViz on the bench, Foxglove for fleet and bag review, and Rerun inside training and evaluation scripts.

**Known gap.** URDF 1.2's new capsule geometry (Lyrical, 2026) was not yet drawn by RViz's RobotModel display at release.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[RViz_Tutorial]])
```
