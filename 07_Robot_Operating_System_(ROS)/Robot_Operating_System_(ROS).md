---
title: Robot Operating System (ROS)
description: "ROS is the open-source software framework most robots are built on: a way to split a robot's software into small programs (nodes) that exchange typed messages, plus tools, drivers and libraries for perception, navigation, manipulation and simulation. Covers the core ideas, ROS 1 versus ROS 2, which distribution to use in 2026 (Lyrical Luth), and where ROS fits next to AI and learned policies."
aliases:
  - ROS
  - ROS (Robot Operating System)
tags:
  - ros
  - ros2
  - middleware
  - robotics-software
  - software-stack
type: overview
level: beginner
subfield:
  - software
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Robots]]"
related:
  - "[[ROS_2_Overview]]"
  - "[[Gazebo_Simulator]]"
  - "[[ROS_AI_Integration]]"
  - "[[Nav2_Navigation]]"
  - "[[MoveIt2]]"
  - "[[ros2_control]]"
key_takeaways:
  - "ROS is not an operating system: it is middleware plus an ecosystem. Programs called nodes exchange typed messages over topics, services and actions."
  - "Use ROS 2. ROS 1 reached end of life with Noetic in May 2025. For new work in late 2026, Lyrical Luth (LTS, May 2026 to May 2031) on Ubuntu 26.04 is the recommended distribution; Jazzy (to 2029) and Humble (to May 2027) remain supported."
  - "ROS is the plumbing around learned policies, not a competitor to them: drivers, transforms, logging, simulation and safety layers still run on ROS when a VLA or RL policy makes the decisions."
sources:
  - "Macenski, S., Foote, T., Gerkey, B., Lalancette, C. and Woodall, W. (2022). Robot Operating System 2: Design, Architecture, and Uses In The Wild. https://arxiv.org/abs/2211.07752"
  - "Open Robotics (2026). ROS 2 Lyrical Luth release notes. https://github.com/ros2/ros2_documentation/blob/rolling/source/Releases/Release-Lyrical-Luth.rst"
  - "Open Robotics (2026). Lyrical Luth supported platforms. https://github.com/ros2/ros2_documentation/blob/rolling/source/Releases/lyrical/supported-platforms.rst"
  - "Open Robotics (2025). ROS 2 Kilted Kaiju release notes. https://github.com/ros2/ros2_documentation/blob/rolling/source/Releases/Release-Kilted-Kaiju.rst"
  - "Gerkey, B. (2015). Why ROS 2? ROS 2 design article. https://design.ros2.org/articles/why_ros2.html"
  - "Open Robotics (2017). REP 2000: ROS 2 Releases and Target Platforms. https://www.ros.org/reps/rep-2000.html"
---

# Robot Operating System (ROS)

The **Robot Operating System (ROS)** is the open-source software framework that most research robots, and a large share of commercial ones, are built on. Despite the name it is not an operating system: it runs on top of Linux (and, with fewer packages, Windows and macOS). It gives you three things:

1. **Plumbing**: a way to split a robot's software into many small programs that find each other and exchange data.
2. **Tools**: for building, launching, recording, replaying, visualising and debugging those programs.
3. **An ecosystem**: thousands of packages, including drivers for sensors and motors, [[Nav2_Navigation|navigation]], [[MoveIt2|manipulation planning]], [[SLAM_with_ROS|mapping]], [[ros2_control|control]] and [[Gazebo_Simulator|simulation]].

> **Etymology.** *Robot* comes from Czech *robota*, "forced labour", coined for Karel Čapek's 1920 play *R.U.R.* (see [[Robots]]). The *operating system* in the name is aspirational: in 2007, when Willow Garage and Stanford started the project, the idea was that ROS would do for robots what an operating system does for a computer, hiding the hardware behind common interfaces so software could be shared.

## The core idea: a graph of programs

A ROS application is a **graph**: the nodes are programs, and the edges are named channels of data.

| Concept | What it is | Analogy |
|---|---|---|
| **Node** | One program with one job (read the lidar, estimate pose, plan a path) | A worker at a station |
| **Topic** | A named stream of messages; many publishers, many subscribers | A radio channel |
| **Message** | A typed data structure (`sensor_msgs/msg/Image`, `geometry_msgs/msg/Twist`) | A standard form |
| **Service** | A request and a reply, for short calls | Asking a question |
| **Action** | A long-running goal with feedback, which can be cancelled (see [[Actions]]) | Placing an order and tracking delivery |
| **Parameter** | A configuration value a node exposes (gains, frame names, rates) | A dial on the station |
| **TF** | The tree of coordinate frames and how they move (see [[TF_and_Topic_Architecture]]) | Everyone agreeing where "here" is |

Because nodes only agree on message types and names, you can replace a simulated camera with a real one, or one planner with another, without touching the rest of the system.

### Worked example: a delivery robot's graph

```
/lidar_driver --scan--> /slam_toolbox --map, map->odom--> /nav2 (planner + controller) --cmd_vel--> /base_controller --> motors
/wheel_odometry --odom, odom->base_link-->-------------------^
/camera_driver --image--> /person_detector --detections--> /nav2 costmap
```

Seven nodes, about six topics, two TF transforms. Count the data: a 2D lidar at 10 Hz with 1,080 ranges sends about 1,080 × 4 bytes × 10 = 43 kB/s; a 640×480 RGB camera at 30 Hz sends 640 × 480 × 3 × 30 = 27.6 MB/s. That roughly 640-fold difference is why images usually stay on one machine (or move as compressed or zero-copy messages) while small topics travel freely over the network.

## ROS 1 and ROS 2

ROS 1 (2007–2025) assumed one robot, a trusted network and a central "master" process. ROS 2 was redesigned on standard middleware (DDS, and from 2025 also Zenoh) for multiple robots, unreliable networks, real-time control, security and products. **ROS 1 reached end of life with Noetic in May 2025**; all new work should use ROS 2. See [[ROS_2_Overview]] for the details.

## Which distribution to use (November 2026)

ROS 2 ships a new **distribution** every May. Even-year releases are long-term support (LTS, 5 years); odd-year releases are supported for about 18 months.

| Distribution | Released | Supported until | Main platform | Gazebo |
|---|---|---|---|---|
| **Lyrical Luth** (LTS) | 22 May 2026 | May 2031 | Ubuntu 26.04, Windows 11, RHEL 10 | Jetty |
| Kilted Kaiju | May 2025 | December 2026 | Ubuntu 24.04 | Ionic |
| **Jazzy Jalisco** (LTS) | May 2024 | May 2029 | Ubuntu 24.04 | Harmonic |
| Humble Hawksbill (LTS) | May 2022 | May 2027 | Ubuntu 22.04 | Fortress |
| Rolling Ridley | continuous | — | latest | latest |

**Rule of thumb.** Start new projects on **Lyrical**. Stay on **Jazzy** if your hardware vendor's drivers only support it. Plan to leave **Humble** before May 2027.

## What changed in 2025–2026

- **Zenoh became a Tier 1 middleware** in Kilted (2025): `rmw_zenoh_cpp` is a fully supported alternative to DDS, attractive over Wi-Fi and the internet because it needs no multicast discovery. Fast DDS stays the default in Lyrical.
- **Lyrical Luth (2026)** added, among other things:
  - an `asyncio`-based `AsyncNode` for Python, using less CPU than the default executor;
  - zero-copy publishing with `rosidl::Buffer`;
  - URDF 1.2 with quaternions, capsule shapes and acceleration and jerk limits (see [[URDF]]);
  - remote control of bag recording and playback (see [[rosbag2]]).
- **Ecosystem consolidation.** Gazebo Classic reached end of life in January 2025, and modern Gazebo (formerly Ignition) is the default simulator. PickNik, the company behind MoveIt, announced in 2026 that it is now part of Qualcomm; MoveIt itself remains open source.

## ROS and AI in 2026

Learned policies have changed what sits at the *top* of a robot's software stack, not the plumbing underneath. A [[Vision-Language-Action_Models|vision-language-action model]] or an [[Reinforcement_Learning_for_Robots|RL policy]] typically runs as one ROS node (or behind one). It consumes camera and joint-state topics and publishes joint or end-effector targets to [[ros2_control]] controllers, while ROS still handles:
- drivers and time synchronisation;
- TF and robot models;
- data logging for training (`rosbag2` → datasets);
- [[Gazebo_Simulator|simulation]];
- safety monitors.

See [[ROS_AI_Integration]].

## Where to go next

[[ROS_2_Overview]] → [[Custom_Packages_and_Nodes]] → [[Launch_Files]] → [[TF_and_Topic_Architecture]] → [[URDF]] → [[RViz_Tutorial]] → [[Gazebo_Simulator]] → [[ros2_control]] → [[Nav2_Navigation]] / [[MoveIt2]]

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Robot_Operating_System_(ROS)]])
```
