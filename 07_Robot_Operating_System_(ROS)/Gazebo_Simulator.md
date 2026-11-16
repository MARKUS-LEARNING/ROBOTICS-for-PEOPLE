---
title: Gazebo Simulator
description: "Gazebo is the open-source robot simulator most used with ROS 2. Covers the modern Gazebo (formerly Ignition) and the end of Gazebo Classic, which Gazebo release goes with which ROS 2 distribution (Jetty with Lyrical, Harmonic with Jazzy), the ros_gz bridge, SDF and URDF models, and how to launch a simulated robot."
aliases:
  - Gazebo
  - Gazebo Sim
  - Ignition Gazebo
tags:
  - gazebo
  - simulation
  - ros2
  - robotics-software
  - urdf
  - sdf
type: tool
level: intermediate
subfield:
  - software
status: reviewed
author: Jordan_Smith
date: 2025-04-28
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Robot_Operating_System_(ROS)]]"
  - "[[Robot_Simulation]]"
related:
  - "[[URDF]]"
  - "[[ros2_control]]"
  - "[[RViz_Tutorial]]"
  - "[[Launch_Files]]"
  - "[[Physics_Engines]]"
  - "[[Simulators_and_IDEs]]"
key_takeaways:
  - "'Gazebo' now means the modern simulator (formerly Ignition); Gazebo Classic reached end of life on 29 January 2025."
  - "Match Gazebo to your ROS 2 distribution: Jetty with Lyrical (recommended for new projects), Harmonic with Jazzy, Fortress with Humble."
  - "ROS 2 talks to Gazebo through ros_gz: a bridge for topics, a launcher, and spawning tools; gz_ros2_control connects controllers."
sources:
  - "Open Robotics (2026). Gazebo releases. https://gazebosim.org/docs/latest/releases/"
  - "Open Robotics (2026). Installing Gazebo with ROS. https://gazebosim.org/docs/latest/ros_installation/"
  - "Open Robotics (2025). Gazebo Classic. https://classic.gazebosim.org/"
  - "ROS 2 Release Team (2026). ROS 2 Lyrical Luth Released! https://discourse.openrobotics.org/t/ros-2-lyrical-luth-released/55021"
  - "Open Robotics (2026). ros_gz: integration between ROS and Gazebo. https://github.com/gazebosim/ros_gz"
---

# Gazebo Simulator

**Gazebo** is an open-source 3D robot simulator built to work with ROS. It simulates robots and their surroundings (rigid-body physics, contact, sensors such as cameras, LIDAR and IMUs) so that the same ROS 2 software can run against a simulated robot before it runs on a real one. For ROS development it is the default choice; for large-scale robot learning, GPU simulators such as [[Isaac_Lab]] and [[MuJoCo]] Playground are more common (see [[Robot_Simulation]]).

> **Etymology.** A *gazebo* is a garden pavilion with open sides and a view all round. The word appeared in 18th-century English, probably as a mock-Latin joke: *gaze* + *-ebo*, "I shall gaze". Andrew Howard and Nate Koenig started the simulator at the University of Southern California in 2002 as a 3D, outdoor companion to the 2D *Stage* simulator; the name suits a place from which you watch your robots.

## Two Gazebos, and which one is current

| | Gazebo Classic | Gazebo (modern) |
|---|---|---|
| Versions | 1 to 11 | Codenamed releases: Fortress, Harmonic, Ionic, Jetty, … |
| Former name | "Gazebo" | "Ignition Gazebo" until 2022 |
| Status | **End of life on 29 January 2025** | Actively developed |
| ROS integration | `gazebo_ros_pkgs` | `ros_gz` |
| Command | `gazebo`, `gzserver`, `gzclient` | `gz sim` |

Tutorials that mention `gazebo_ros`, `gzserver` or `spawn_entity.py` refer to Gazebo Classic. New projects should use the modern Gazebo.

## Matching Gazebo to ROS 2

From Gazebo's official compatibility table (November 2026):

| ROS 2 distribution | Recommended Gazebo | Gazebo support ends |
|---|---|---|
| **Lyrical Luth** (LTS, May 2026) | **Jetty** (LTS) | May 2031 |
| Kilted Kaiju | Ionic | December 2026 |
| Jazzy Jalisco (LTS) | Harmonic (LTS) | May 2029 |
| Humble Hawksbill (LTS) | Fortress (LTS) | May 2027 |

Gazebo's documentation recommends **Ubuntu 26.04 (Resolute), ROS 2 Lyrical and Gazebo Jetty** for new users, and Ubuntu 24.04, Jazzy and Harmonic for those who cannot upgrade yet. Gazebo's next codenamed release is planned for March 2027, two months before the next ROS 2 release.

## How a robot gets into Gazebo

Gazebo's native format is **SDF** (Simulation Description Format), which describes whole worlds: lighting, terrain, models, sensors and plugins. ROS robots are usually described in **[[URDF]]**; Gazebo converts URDF to SDF on load. Simulation-only details (sensor plugins, friction, the `ros2_control` hardware interface) go in URDF `<gazebo>` tags or in xacro macros.

## Connecting ROS 2: ros_gz

Install the integration for your distribution (it pulls in the matching Gazebo):

```bash
sudo apt install ros-${ROS_DISTRO}-ros-gz
```

The pieces of `ros_gz`:

| Package | What it does |
|---|---|
| `ros_gz_sim` | Launch Gazebo from ROS (`gz_sim.launch.py`) and spawn models (`create`) |
| `ros_gz_bridge` | Bridge Gazebo topics to ROS 2 topics, e.g. camera images, laser scans, `/cmd_vel`, `/clock` |
| `ros_gz_image` | Efficient image transport for cameras |
| `gz_ros2_control` (separate repo) | Run [[ros2_control]] controllers against simulated joints |

A typical launch, sketched:

```bash
# start Gazebo with a world
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="-r empty.sdf"
# publish the robot description, then spawn it from the topic
ros2 run ros_gz_sim create -topic robot_description -name my_robot -z 0.1
# bridge the topics you need (Gazebo type in brackets after @)
ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
    /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
```

Run ROS nodes with `use_sim_time:=true` so they follow Gazebo's clock instead of the wall clock. In real projects, put the bridge mappings in a YAML file and everything in one [[Launch_Files|launch file]].

## Physics and performance

Modern Gazebo uses pluggable physics engines: DART by default, with Bullet and a lightweight engine (TPE) for simple kinematic cases (see [[Physics_Engines]]). It runs one world on the CPU at or faster than real time for typical mobile robots and arms. It is not designed to run thousands of parallel worlds for reinforcement learning; that is the job of [[GPU-Parallel_Simulation]].

## Worked example: why use_sim_time matters

A navigation stack's controller runs at 20 Hz and times out if it hears nothing from odometry for 0.5 s. If the simulation runs at half real time (real-time factor 0.5) but ROS nodes use wall-clock time, 0.5 s of wall time is only 0.25 s of simulated time. The controller will see odometry arriving at half the expected rate and may time out on a perfectly healthy simulation. With `use_sim_time:=true`, every node measures time on Gazebo's clock and the timeouts mean the same thing in simulation as on the robot.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Gazebo_Simulator]])
```
