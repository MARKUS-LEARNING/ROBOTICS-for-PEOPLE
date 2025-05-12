---
title: ROS 2 Overview
description: Provides an overview of ROS 2, the second generation of the Robot Operating System, highlighting its key features, improvements over ROS 1, and core concepts.
tags:
  - ROS
  - ROS2
  - middleware
  - robotics-software
  - DDS
  - QoS
  - real-time
  - colcon
  - software-stack
layout: default
category: robotics
author: Jordan_Smith_&_le_Chat
date: 2025-04-28
permalink: /ros_2_overview/
related:
  - "[[Robot_Operating_System_(ROS)]]"
  - "[[DDS]]"
  - "[[Quality_of_Service_(QoS)]]"
  - "[[Nodes]]"
  - "[[Topics]]"
  - "[[Services]]"
  - "[[Actions]]"
  - "[[Colcon]]"
  - "[[Launch_Files]]"
  - "[[Real-Time_Systems]]"
---

# ROS 2 Overview

**ROS 2** is the second generation of the **[[ROS (Robot Operating System)]]**, representing a significant redesign of the original ROS (ROS 1) framework. It aims to address limitations encountered in ROS 1 and provide features needed for modern robotics applications, particularly in areas like multi-robot systems, real-time control, embedded systems, and commercial/production environments. While retaining the core philosophy of modularity and distributed communication, ROS 2 introduces substantial changes to its underlying architecture and tools.

---

## Key Features and Differences from ROS 1

ROS 2 was developed to overcome specific shortcomings of ROS 1 and introduce new capabilities:

* **Communication Middleware ([[DDS]]):**
    * **ROS 1:** Uses a custom TCP/UDP-based communication system (TCPROS/UDPROS) requiring a central [[ROS (Robot Operating System)|ROS Master]] for node discovery.
    * **ROS 2:** Built on top of the industry-standard **[[DDS|Data Distribution Service]]** (or an abstraction layer called RMW - ROS Middleware Interface - allowing different DDS implementations like Fast DDS, Cyclone DDS, etc.). This provides:
        * **Decentralized Discovery:** Eliminates the single point of failure of the ROS Master. [[Nodes]] discover each other automatically using DDS mechanisms.
        * **Improved Performance:** DDS implementations are often optimized for low-latency, high-throughput communication.
        * **[[Quality of Service (QoS)|Quality of Service]] Policies:** Granular control over communication reliability.

* **[[Quality of Service (QoS)|Quality of Service (QoS)]]:**
    * **ROS 1:** Primarily offered reliable (TCP) or best-effort (UDP) communication.
    * **ROS 2:** Exposes underlying DDS QoS policies, allowing developers to configure aspects like:
        * **Reliability:** Best-effort or reliable delivery.
        * **Durability:** How long data persists for late-joining nodes (volatile or transient local).
        * **History:** Keep last N messages or all messages within resource limits.
        * **Deadline, Lifespan, Liveliness:** Policies related to timing and node/topic availability.
    This enables tailoring communication for different needs (e.g., reliable service calls vs. best-effort sensor streams, real-time control data).

* **[[Nodes]] and Composition:**
    * The concept of nodes as executable processes remains.
    * ROS 2 introduces **component nodes** and **composition**, allowing multiple nodes (written as components/libraries) to be run within a single operating system process. This reduces overhead (inter-process communication becomes intra-process) and allows for more efficient deployment, especially on resource-constrained systems.

* **Client Libraries:**
    * **ROS 1:** `roscpp` (C++) and `rospy` (Python).
    * **ROS 2:** **`rclcpp`** (C++) and **`rclpy`** (Python). These libraries offer a more modern, object-oriented API and expose ROS 2 features like QoS and node composition.

* **Build System ([[Colcon]]):**
    * **ROS 1:** Primarily used **Catkin**, an extension of CMake.
    * **ROS 2:** Uses **Colcon** as the standard build tool. Colcon is a meta-build tool that orchestrates underlying build systems like CMake (for C++) and Python's setuptools. It works with the **Ament** build system framework, which provides CMake macros and Python packages to simplify package development.

* **[[Launch Files]]:**
    * **ROS 1:** Used XML-based `.launch` files.
    * **ROS 2:** Uses **Python-based `.launch.py` files**. This provides significantly more flexibility, allowing programmatic logic, conditionals, loops, and easier integration with external tools within the launch description. XML and YAML formats are also supported for simpler cases.

* **Command Line Interface (CLI):**
    * **ROS 1:** Used multiple distinct commands (e.g., `rostopic`, `rosservice`, `rosnode`, `rosparam`, `roslaunch`).
    * **ROS 2:** Uses a unified `ros2` command with various subcommands (verbs), e.g., `ros2 topic list`, `ros2 node info`, `ros2 launch <pkg> <launch_file>`, `ros2 run <pkg> <executable>`.

* **Real-Time and Embedded Support:**
    * The architecture (DDS, client libraries) is designed to better support real-time performance requirements and deployment on embedded systems.

* **Cross-Platform Support:**
    * ROS 2 officially supports Linux, macOS, and Windows, broadening its accessibility.

---

## Core Concepts in ROS 2

Many core concepts from ROS 1 persist, albeit with implementation differences:

* **Packages:** The fundamental unit of software organization, containing nodes, libraries, configuration files, launch files, and interface definitions. Uses `package.xml` (format 2 or 3) and build files (`CMakeLists.txt`, `setup.py`).
* **Nodes:** Individual executable processes performing specific computations.
* **Topics:** Asynchronous, publish/subscribe communication channels for streaming data. Uses defined [[Messages|message types]]. Governed by QoS settings.
* **Services:** Synchronous, request/reply communication for specific queries or commands. Uses defined service types (.srv). Governed by QoS settings.
* **Actions:** Asynchronous, long-running, goal-oriented tasks with feedback. Uses defined action types (.action). Governed by QoS settings.
* **Parameters:** Runtime configurable parameters associated with nodes.
* **Launch System:** Mechanism for starting and configuring multiple nodes simultaneously.

---

## Benefits and Status

ROS 2 offers several advantages over ROS 1, including improved performance, enhanced reliability (no single Master failure point), support for real-time applications, better multi-robot system capabilities, fine-grained communication control via QoS, and official support for more platforms.

ROS 2 is the focus of active development by Open Robotics (formerly OSRF) and the global ROS community. While ROS 1 (specifically the Noetic Ninjemys distribution) is still widely used and benefits from a larger existing package ecosystem and long-term support, ROS 2 is generally recommended for new robotics projects, especially those requiring real-time capabilities, high reliability, or targeting commercial deployment. Migration guides and tools exist to help transition projects from ROS 1 to ROS 2.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #robotics OR #control-systems WHERE contains(file.outlinks, [[Robot_Operating_System_(ROS)]])