---
title: Turtlesim Projects
description: "Turtlesim is ROS 2's tiny 2D simulator for learning topics, services, actions, parameters and launch files. Covers its interfaces as of ROS 2 Lyrical (including the move of its message types to turtlesim_msgs from Kilted), the commands to explore them, a computed circle-drawing example, and a ladder of projects from teleoperation to a turtle-chasing controller with TF."
aliases:
  - Turtlesim
  - ROS 2 Turtlesim
tags:
  - ros2
  - education
type: tutorial
level: beginner
subfield:
  - software
  - creative-education
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Robot_Operating_System_(ROS)]]"
related:
  - "[[ROS_2_Communication]]"
  - "[[Actions]]"
  - "[[Launch_Files]]"
  - "[[TF_and_Topic_Architecture]]"
  - "[[Labs_Projects_and_Tutorials]]"
key_takeaways:
  - "Turtlesim exercises every core ROS 2 concept (nodes, topics, services, actions, parameters, launch) without any hardware or heavy simulator."
  - "From Kilted (2025) its interface types live in the turtlesim_msgs package (e.g. turtlesim_msgs/msg/Pose); Jazzy and older still use turtlesim/msg/Pose, so older tutorials need that one change."
  - "Commanding 2.0 linear and 1.0 rad/s angular velocity draws a circle of radius 2.0 in about 6.28 s, a quick check that you understand the kinematics."
sources:
  - "Open Robotics (2026). ros_tutorials: turtlesim source (rolling branch). https://github.com/ros/ros_tutorials"
  - "Open Robotics (2026). ROS 2 documentation source: Introducing turtlesim. https://github.com/ros2/ros2_documentation/blob/rolling/source/Get-Started/Introducing-Turtlesim/Introducing-Turtlesim.rst"
  - "Open Robotics (2026). ROS 2 documentation source: Understanding topics. https://github.com/ros2/ros2_documentation/blob/rolling/source/ROS-Framework/interfaces/topics/Understanding-ROS2-Topics/Understanding-ROS2-Topics.rst"
---

# Turtlesim Projects

**Turtlesim** is a minimal 2D simulator shipped with ROS: turtles in a window that move when you send velocity commands and draw as they go. It is how most people learn ROS 2, because every core concept can be tried in minutes.

> **Etymology.** Named after the turtle robots of the Logo programming language (Seymour Papert, 1960s), which drew lines as they moved.

## Its interfaces (ROS 2 Lyrical)

| Kind | Name | Type |
|---|---|---|
| Topic (in) | `/turtle1/cmd_vel` | `geometry_msgs/msg/Twist` |
| Topic (out) | `/turtle1/pose` | `turtlesim_msgs/msg/Pose` |
| Topic (out) | `/turtle1/color_sensor` | `turtlesim_msgs/msg/Color` |
| Services | `/spawn`, `/kill`, `/clear`, `/reset` | `turtlesim_msgs/srv/...` (`std_srvs/srv/Empty` for clear and reset) |
| Services (per turtle) | `/turtle1/set_pen`, `/turtle1/teleport_absolute`, `/turtle1/teleport_relative` | `turtlesim_msgs/srv/...` |
| Action | `/turtle1/rotate_absolute` | `turtlesim_msgs/action/RotateAbsolute` |
| Parameters | `background_r`, `background_g`, `background_b` | integers 0–255 |

**Version note.** In Kilted and Lyrical the types are in **`turtlesim_msgs`**; in Jazzy and earlier they are in `turtlesim` (for example `turtlesim/msg/Pose`). Adjust imports when following older tutorials.

## Explore it

```bash
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key          # in a second terminal
ros2 topic echo /turtle1/pose
ros2 service call /spawn turtlesim_msgs/srv/Spawn "{x: 2.0, y: 2.0, theta: 0.0, name: 'turtle2'}"
ros2 action send_goal /turtle1/rotate_absolute turtlesim_msgs/action/RotateAbsolute "{theta: 1.57}" --feedback
ros2 param set /turtlesim background_r 150
rqt_graph
```

## Worked example: drawing a circle

Publishing a constant `Twist` with linear $v = 2.0$ (units per second) and angular $\omega = 1.0$ rad/s makes the turtle drive a circle of radius $r = v/\omega = 2.0$. It completes one lap in $2\pi/\omega = 6.28$ s. Predict, then check with:

```bash
ros2 topic pub --rate 10 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}, angular: {z: 1.0}}"
```

The window is about 11 units wide with turtle1 starting near its centre, so a radius-2 circle fits comfortably.

## Project ladder

1. **Draw shapes**: a square (drive, stop, turn 90° with the action, repeat) and a spiral (increase $v$ over time).
2. **Write a node**: a Python node that publishes `cmd_vel` and subscribes to `pose` (see [[Python_ROS_Nodes]]).
3. **Go to goal**: a proportional controller on distance and heading error that drives to a clicked or given point.
4. **Services**: spawn several turtles, set pen colours, clear the screen.
5. **Launch and namespaces**: start two simulators in different namespaces with a [[Launch_Files|launch file]]; remap one teleop to control both.
6. **TF follower**: publish each turtle's pose as a TF frame and make turtle2 chase turtle1 using a transform lookup (see [[TF_and_Topic_Architecture]]).
7. **Record and replay**: record a session with [[rosbag2]] and replay it to reproduce the drawing.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Turtlesim_Projects]])
```
