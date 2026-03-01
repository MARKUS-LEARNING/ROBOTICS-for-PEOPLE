---
title: Simulators and IDEs
description: A comprehensive reference for simulation tools, visualization environments, and Integrated Development Environments (IDEs) used in robotics research and development, covering physics simulators, cloud tools, and editors.
tags:
  - simulator
  - IDE
  - software
  - tools
  - development
  - Gazebo
  - MuJoCo
  - ROS
  - VSCode
  - simulation
  - coding
  - references
layout: default
category: robotics
author: Jordan_Smith_&_Claude
date: 2025-05-02
permalink: /simulators_and_ides/
related:
  - "[[Simulation]]"
  - "[[Gazebo_Simulator]]"
  - "[[RViz_Tutorial]]"
  - "[[Robot_Operating_System_(ROS)]]"
  - "[[ROS_2_Overview]]"
  - "[[rosbag2]]"
  - "[[C++]]"
  - "[[Python_for_Robotics]]"
  - "[[Reinforcement_Learning_(RL)]]"
  - "[[Sim2Real_Experiments]]"
  - "[[SLAM_with_ROS]]"
  - "[[Nav2_Navigation]]"
  - "[[MoveIt2]]"
---

# Simulators and IDEs in Robotics

This note provides a comprehensive reference for simulation environments and development tools used in robotics. Choosing the right simulator and IDE significantly impacts development speed, algorithm quality, and how well results transfer to real hardware.

See also: [[Resources_Index]], [[rosbag2]], [[Sim2Real_Experiments]]

---

## Physics Simulators

Physics simulators provide virtual environments with accurate dynamics, sensor modeling, and robot interaction — essential for algorithm development, [[Reinforcement_Learning_(RL)|RL]] training, and [[Sim2Real_Experiments|Sim2Real]] research without risking hardware.

### Gazebo / Gazebo Sim

The **de facto standard** simulator for ROS robotics. Two generations currently coexist:

* **Gazebo Classic** (versions 1–11): The widely-used original. Stable, large existing model/plugin library, but no longer actively developed.
* **Gazebo Sim** (formerly Ignition Gazebo, versions 6+): The modern successor. Modular architecture, improved performance, actively maintained, and the default for ROS 2 (Humble+).

**Strengths:** Deep [[ROS_2_Overview|ROS 2]] integration, large community, extensive sensor library ([[LIDAR]], [[Camera_Systems]], [[IMU_Sensors]], contact, GPS), multiple physics backends (ODE, Bullet, DART, Simbody).

**Best for:** ROS 2 development, mobile robot navigation, manipulation, sensor simulation.

Website: [gazebosim.org](https://gazebosim.org/)

---

### MuJoCo

**MuJoCo** (Multi-Joint dynamics with Contact) has become the dominant simulator for **robot learning and RL research** since DeepMind made it free and open-source in 2021. It is renowned for its extremely accurate and stable contact dynamics and fast simulation speed.

* **Strengths:** Best-in-class contact/friction physics, very fast (often 10–100× faster than Gazebo for contact-rich tasks), excellent Python API, stable numerical integration.
* **Weaknesses:** Less sensor variety than Gazebo, historically weaker ROS integration (though `mujoco_ros` packages exist).
* **Used by:** OpenAI, DeepMind, most academic RL manipulation research. Standard environments in `dm_control` and `Gymnasium`.

**Best for:** Robot learning, dexterous manipulation, contact-rich tasks, RL policy training.

Website: [mujoco.org](https://mujoco.org/)

---

### NVIDIA Isaac Sim

Built on the **NVIDIA Omniverse** platform with PhysX 5 physics. Designed for high-fidelity, photorealistic simulation with GPU acceleration.

* **Strengths:** Photorealistic rendering (ray tracing), domain randomization for [[Sim2Real_Experiments|Sim2Real]], synthetic dataset generation, GPU-accelerated physics, integrated **Isaac Lab** framework for RL training, full ROS 2 bridge.
* **Weaknesses:** Requires a capable NVIDIA GPU (RTX recommended), heavier setup than Gazebo, commercial ecosystem.
* **Isaac Lab:** A modular framework built on Isaac Sim for robot learning — supports training locomotion, manipulation, and dexterous hand policies.

**Best for:** High-fidelity Sim2Real, synthetic data generation for perception, GPU-accelerated RL at scale.

Website: [developer.nvidia.com/isaac-sim](https://developer.nvidia.com/isaac-sim)

---

### PyBullet

Python bindings for the **Bullet** physics engine. Lightweight and quick to prototype with — no GUI required in headless mode.

* **Strengths:** Pure Python API, easy to install (`pip install pybullet`), supports URDF and SDF, fast headless simulation for RL, used in many research codebases.
* **Weaknesses:** Less feature-rich than MuJoCo or Gazebo; officially unmaintained as Bullet development moved to the C++ API.

**Best for:** Quick prototyping, research scripts, headless RL training pipelines.

Website: [pybullet.org](https://pybullet.org/)

---

### Webots

Open-source, cross-platform desktop simulator from Cyberbotics. Particularly popular in **education**.

* **Strengths:** Beginner-friendly GUI, self-contained (no separate physics engine setup), large library of robot models, supports C++, Python, Java, MATLAB, ROS/ROS 2 bridge.
* **Weaknesses:** Less commonly used in production research than Gazebo or MuJoCo.

**Best for:** Education, rapid prototyping, courses.

Website: [cyberbotics.com](https://cyberbotics.com/)

---

### CoppeliaSim (formerly V-REP)

Versatile commercial/academic simulator with a distributed control architecture. Widely used in academia.

* **Strengths:** Extensive built-in model library, supports embedded Lua scripts and Python API, ROS interface, both free EDU and commercial licenses.
* **Weaknesses:** Less ROS-native than Gazebo; learning curve for scripting architecture.

Website: [coppeliarobotics.com](https://coppeliarobotics.com/)

---

### CARLA

Open-source simulator specifically designed for **autonomous driving** research. Built on Unreal Engine for photorealistic urban environments.

* **Strengths:** High-quality urban scenarios, rich sensor suite (cameras, LiDAR, radar, semantic segmentation, depth), traffic and pedestrian simulation, Python API, active research community.
* **Best for:** Autonomous driving, perception algorithm testing, HD map development.

Website: [carla.org](https://carla.org/)

---

### Other Notable Simulators

| Simulator | Focus | Notes |
|---|---|---|
| **Drake** (TRI) | Manipulation, trajectory optimization | Rigorous multibody physics; strong optimization integration |
| **Sapien** | Manipulation learning | Photorealistic; used in RLBench, ManiSkill benchmarks |
| **Genesis** | Universal physics (2024) | New; GPU-accelerated, aims to unify simulation paradigms |
| **AirSim** | Aerial/ground vehicles | Microsoft project; largely unmaintained since 2022 — prefer alternatives |
| **MATLAB/Simulink** | Control system design, model-based | Robotics System Toolbox; strong for classical control analysis |

---

## Visualization Tools

### RViz / RViz2

The standard **3D visualization** tool for ROS. Not a simulator — displays data published on ROS topics (sensor streams, robot models, maps, paths). See [[RViz_Tutorial]] for full coverage.

### Foxglove Studio

Modern, web-based visualization and analysis tool for ROS 2 and MCAP bag files. A powerful complement or alternative to RViz for post-hoc analysis.

* **Strengths:** Browser-based (no install) or desktop app, excellent timeline scrubbing for bag playback, supports custom panels (3D, plots, images, raw messages), Foxglove Bridge for live ROS 2 connection.
* **Best for:** Bag file analysis, debugging, team sharing of recordings.

Website: [foxglove.dev](https://foxglove.dev/)

### rqt

A Qt-based ROS GUI framework providing modular diagnostic and visualization plugins:

| Plugin | Purpose |
|---|---|
| `rqt_graph` | Visualize the ROS node/topic computation graph |
| `rqt_plot` | Real-time plotting of numeric topic fields |
| `rqt_image_view` | Display camera image streams |
| `rqt_console` | Browse and filter ROS log messages |
| `rqt_tf_tree` | Visualize the TF transform tree |
| `rqt_reconfigure` | Live parameter tuning for running nodes |
| `rqt_bag` | Inspect and play rosbag2 files |

```bash
ros2 run rqt_graph rqt_graph
ros2 run rqt_plot rqt_plot /joint_states/position[0]
```

---

## Integrated Development Environments (IDEs)

### Visual Studio Code (Recommended)

The most widely used editor in the ROS community. Free, lightweight, and highly extensible.

* **Key extensions for robotics:**
  * **ROS** (ms-iot.vscode-ros): Build, run, and debug ROS/ROS 2 nodes; browse topics/services; TF tree viewer.
  * **C/C++** (ms-vscode.cpptools): IntelliSense, debugging, CMake integration.
  * **CMake Tools**: Configure and build CMake projects directly.
  * **Python**: Full Python language support, debugging, virtual environments.
  * **Remote - SSH**: Develop directly on a robot's onboard computer over SSH.
  * **Docker**: Work inside containerized ROS environments.
* **Best for:** All ROS 2 development; highly recommended as the primary editor.

Website: [code.visualstudio.com](https://code.visualstudio.com/)

---

### CLion

Powerful commercial C++ IDE from JetBrains. Deep CMake integration and superior C++ code analysis make it a favorite for C++-heavy ROS projects.

* **Features:** Advanced refactoring, GDB/LLDB debugging, CMake-native workflow, ROS plugin available. Free licenses for students and academics.
* **Best for:** Large C++ ROS packages; teams preferring a full IDE over an editor.

Website: [jetbrains.com/clion](https://www.jetbrains.com/clion/)

---

### PyCharm

The leading Python IDE from JetBrains, ideal for Python-heavy ROS 2 work or ML pipeline development alongside ROS.

* **Features:** Excellent debugger, test runner, Jupyter notebook support, scientific tools integration (NumPy, Matplotlib). Free Community Edition available.
* **Best for:** `rclpy` node development, ML integration, data analysis scripts.

Website: [jetbrains.com/pycharm](https://www.jetbrains.com/pycharm/)

---

### Neovim / Vim

Terminal-based editors popular for remote robot development over SSH where GUI IDEs are impractical.

* **Setup:** With `nvim-lspconfig` + `clangd` (C++) and `pyright` (Python), provides full LSP-powered completions, go-to-definition, and diagnostics in the terminal.
* **Best for:** Onboard robot development via SSH, embedded systems, developers who prefer modal editing.

---

### Qt Creator

Cross-platform C++ IDE focused on the Qt framework. Useful for developing `rqt` plugins and robot GUIs, as well as general CMake C++ development.

Website: [qt.io/product/development-tools](https://www.qt.io/product/development-tools)

---

## Simulator Selection Guide

| Use Case | Recommended Simulator |
|---|---|
| ROS 2 mobile robot development | Gazebo Sim |
| Robot arm / manipulation | Gazebo Sim + MoveIt 2, or MuJoCo |
| RL policy training (contact-rich) | MuJoCo, Isaac Lab |
| RL policy training (GPU-scale) | Isaac Lab (NVIDIA) |
| Sim2Real with synthetic data | Isaac Sim |
| Autonomous driving | CARLA |
| Education / teaching | Webots, Gazebo Classic |
| Quick Python prototyping | PyBullet |
| Trajectory optimization / manipulation | Drake |

---

## Dataview Plugin Features

```dataview
LIST FROM #simulator OR #IDE WHERE contains(file.outlinks, [[Simulators_and_IDEs]])
```

```dataview
TABLE title, description FROM #tools WHERE contains(file.tags, "simulator")
```
