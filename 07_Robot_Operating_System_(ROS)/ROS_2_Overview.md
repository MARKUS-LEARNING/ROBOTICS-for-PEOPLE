---
title: ROS 2 Overview
description: "How ROS 2 works under the hood and how it differs from ROS 1: the RMW middleware layer (DDS or Zenoh), Quality of Service, executors, composition, the build and launch tools, security and real-time support, with a worked QoS example and the 2025–2026 changes (Zenoh Tier 1, events executors, AsyncNode)."
aliases:
  - ROS2
  - ROS 2
tags:
  - ros
  - ros2
  - middleware
  - robotics-software
  - dds
  - qos
  - real-time
  - colcon
  - software-stack
type: concept
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
related:
  - "[[Actions]]"
  - "[[Launch_Files]]"
  - "[[Custom_Packages_and_Nodes]]"
  - "[[TF_and_Topic_Architecture]]"
  - "[[ros2_control]]"
key_takeaways:
  - "ROS 2 sits on a pluggable middleware layer (RMW). Fast DDS is the default in Lyrical; Cyclone DDS, Connext and, since Kilted, Zenoh are the other Tier 1 options."
  - "Quality of Service settings (reliability, durability, history, deadline) decide how each topic behaves on a lossy network; a publisher and subscriber whose QoS is incompatible silently never connect."
  - "Executors decide how callbacks run. The events executors (rclcpp, and Lyrical's EventsCBGExecutor) and rclpy's AsyncNode cut CPU use compared with the classic executors."
sources:
  - "Macenski, S., Foote, T., Gerkey, B., Lalancette, C. and Woodall, W. (2022). Robot Operating System 2: Design, Architecture, and Uses In The Wild. https://arxiv.org/abs/2211.07752"
  - "Gerkey, B. (2015). Why ROS 2? ROS 2 design article. https://design.ros2.org/articles/why_ros2.html"
  - "Open Robotics (2026). ROS 2 Lyrical Luth release notes. https://github.com/ros2/ros2_documentation/blob/rolling/source/Releases/Release-Lyrical-Luth.rst"
  - "Open Robotics (2025). ROS 2 Kilted Kaiju release notes. https://github.com/ros2/ros2_documentation/blob/rolling/source/Releases/Release-Kilted-Kaiju.rst"
  - "Open Robotics (2026). rmw_zenoh: a ROS 2 middleware based on Zenoh. https://github.com/ros2/rmw_zenoh"
  - "Object Management Group (2015). Data Distribution Service (DDS) 1.4. https://www.omg.org/spec/DDS/1.4/About-DDS"
---

# ROS 2 Overview

**ROS 2** is the second generation of the [[Robot_Operating_System_(ROS)|Robot Operating System]]. It keeps ROS 1's central idea, a graph of small programs exchanging typed messages, but rebuilds everything underneath for what ROS 1 was never designed for: several robots, unreliable wireless networks, real-time control loops, embedded processors, security, and products that must be maintained for years. ROS 1 reached end of life in May 2025, so ROS 2 is now simply "ROS".

> **Etymology.** *Middleware*, the word at the heart of ROS 2, dates from a 1968 NATO software-engineering conference: the software in the *middle*, between applications and the operating system. *DDS* stands for *Data Distribution Service*, an Object Management Group standard first published in 2004 for military, aerospace and industrial systems.

## Why a redesign

| ROS 1 assumption | Problem in practice | ROS 2 answer |
|---|---|---|
| A central `roscore` master | Single point of failure; awkward for many robots | Decentralised discovery (DDS) or a router (Zenoh) |
| Reliable, wired network | Wi-Fi drops packets; TCP stalls | Per-topic Quality of Service |
| Linux workstation | No real-time guarantees, no microcontrollers | Real-time-friendly design; micro-ROS for MCUs |
| Trusted lab network | No authentication or encryption | SROS2 security (DDS-Security) |
| Research prototypes | Little attention to lifecycle or long-term support | Managed (lifecycle) nodes; 5-year LTS releases |

## The layer cake

```
your nodes (rclcpp / rclpy)
        │
rcl  (common C core: names, parameters, graph, actions)
        │
rmw  (ROS middleware interface)  ← pick an implementation at run time
        │
Fast DDS · Cyclone DDS · Connext DDS · Zenoh
```

Switch middleware with one environment variable, for example `RMW_IMPLEMENTATION=rmw_zenoh_cpp`. In Lyrical, four implementations are Tier 1 (fully supported):
- Fast DDS (the default);
- Cyclone DDS;
- Connext (all architectures except arm64);
- Zenoh (Tier 1 since Kilted in 2025).

**DDS** discovers peers by multicast on the local network, which is automatic on a wired LAN but noisy and unreliable on Wi-Fi and impossible across the internet. **Zenoh** uses a router (`zenohd`) for discovery and copes better with those networks.

## Quality of Service (QoS)

Every publisher and subscriber carries a QoS profile:

| Policy | Options | Typical use |
|---|---|---|
| **Reliability** | `reliable` / `best_effort` | Commands reliable; high-rate sensor data best-effort |
| **Durability** | `volatile` / `transient_local` | `transient_local` lets late joiners get the last message (maps, `/tf_static`, robot description) |
| **History / depth** | keep last *N* / keep all | Queue size |
| **Deadline, lifespan, liveliness** | durations | Detect a silent sensor or stale data |

**Compatibility rule.** A subscriber may ask for *less* than a publisher offers, never more. A `reliable` subscriber does not connect to a `best_effort` publisher, and a `transient_local` subscriber does not connect to a `volatile` publisher. ROS reports an "incompatible QoS" event, but the symptom is usually just *no data*. Check with `ros2 topic info -v /topic`.

### Worked example: a camera over Wi-Fi

A 640×480 RGB camera at 30 Hz produces 640 × 480 × 3 = 921,600 bytes per frame, or about 27.6 MB/s, which is more than many Wi-Fi links can sustain.
- **With `reliable` QoS**, every lost fragment is retransmitted. Queues fill, latency grows to seconds, and the robot reacts to old images.
- **With `best_effort`** and a depth of 1, lost frames are simply dropped, and the subscriber always gets the newest image it can.

That is why the predefined `SensorDataQoS` profile is best-effort with a small queue. Even better, send compressed images (`image_transport`) or keep the image on the same machine and use zero-copy transport.

## Executors and composition

A node's callbacks (subscriptions, timers, services) run inside an **executor**:

- **Single- and multi-threaded executors**: the classic ones.
- **Events executor** (rclcpp; experimental in rclpy from Kilted). It reacts to a queue of events instead of repeatedly scanning every entity, which uses less CPU.
- **EventsCBGExecutor** (rclcpp, Lyrical). It adds multiple threads and multiple time sources; the release notes report 10–15% less CPU than the single- and multi-threaded executors.
- **`AsyncNode`** (rclpy, Lyrical, experimental). Runs an `asyncio` event loop, so callbacks can `await` service calls and timers.

**Composition** loads several nodes as components into one process, so messages between them can pass by pointer (intra-process, zero-copy) instead of through the network stack. This matters for images and point clouds.

## Tools you use every day

| Task | ROS 2 tool |
|---|---|
| Build a workspace | `colcon build` (packages use `ament_cmake` or `ament_python`) |
| Start many nodes with parameters | `ros2 launch` with Python, XML or YAML launch files (see [[Launch_Files]]) |
| Inspect the graph | `ros2 node`, `ros2 topic`, `ros2 service`, `ros2 action`, `ros2 param`, `rqt_graph` |
| Diagnose setup problems | `ros2 doctor --report` |
| Record and replay data | `ros2 bag` (MCAP format by default; see [[rosbag2]]) |
| Visualise | [[RViz_Tutorial|RViz]]; Foxglove and Rerun are popular web and desktop alternatives |

## Real-time, embedded and secure

- **Real time.** ROS 2 itself does not make Linux real-time, but its design (preallocated memory, controllable executors, real-time-safe [[ros2_control]] loops) works with a `PREEMPT_RT` kernel, which is part of mainline Linux since 6.12. Control loops at 1 kHz are routine.
- **Microcontrollers.** micro-ROS runs ROS 2 nodes on microcontrollers over a serial or UDP link to an agent on the main computer.
- **Security.** SROS2 adds authentication, encryption and access control through DDS-Security.

## Migrating from ROS 1

Most concepts map directly:
- `rospy` → `rclpy`, `roscpp` → `rclcpp`, `catkin` → `colcon`/`ament`.
- `rostopic` → `ros2 topic`, and the other `ros*` commands likewise become `ros2` verbs.
- `.launch` XML → Python, XML or YAML launch files.

The work lies in parameters (which are per-node in ROS 2), QoS choices, and drivers. `ros1_bridge` still helps a gradual migration, but with Noetic past end of life, plan to finish it.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[ROS_2_Overview]])
```
