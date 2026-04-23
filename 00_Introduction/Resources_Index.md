---
title: Resources Index
description: Master index of curated external resources — books, papers, courses, simulators, hardware — cross-linked to the detailed lists in Chapter 13.
tags:
  - index
  - resources
  - tools
  - links
  - references
  - vault-meta
layout: default
category: robotics
author: Jordan_Smith
date: 2026-04-23
permalink: /resources_index/
related:
  - "[[MOOCs_and_Courses]]"
  - "[[Research_Papers_Index]]"
  - "[[Simulators_and_IDEs]]"
  - "[[Hardware_Shopping_List]]"
---

# Resources Index

A compact master index of the external resources this vault points to. The deeper lists live in [Chapter 13 — Tools, References, and Links](../13_Tools_References_and_Links/). This page is meant as a one-screen jumping-off point.

---

## Core Textbooks

The textbooks this vault draws from most often. Every robotics engineer should own or have read at least two of these.

**Robotics fundamentals**
- *Modern Robotics: Mechanics, Planning, and Control* — Kevin M. Lynch & Frank C. Park (2017). Free PDF + Coursera course. Best single book for kinematics, dynamics, and motion planning.
- *Robot Modeling and Control* — Mark W. Spong, Seth Hutchinson, M. Vidyasagar (2nd ed., 2020). Classical control-centric treatment.
- *Introduction to Autonomous Robots* — Correll, Hayes, Heckman, Roncone (open-source). Undergraduate-friendly.
- *Probabilistic Robotics* — Sebastian Thrun, Wolfram Burgard, Dieter Fox (2005). The reference for Bayesian filtering, SLAM, localization.

**Machine learning and AI**
- *Deep Learning* — Ian Goodfellow, Yoshua Bengio, Aaron Courville (2016). Free online.
- *Reinforcement Learning: An Introduction* — Richard S. Sutton, Andrew G. Barto (2nd ed., 2018). Free PDF.
- *Artificial Intelligence: A Modern Approach* — Stuart Russell, Peter Norvig (4th ed., 2020).

**Mathematics**
- *Linear Algebra Done Right* — Sheldon Axler (4th ed., 2024). Free online.
- *A Mathematical Introduction to Robotic Manipulation* — Murray, Li, Sastry (1994). Free PDF. Definitive on SE(3) and Lie groups for robotics.

See [[Research_Papers_Index]] for seminal papers.

---

## Influential Papers

A short list. The full, curated list lives in [[Research_Papers_Index]].

- *EKF-SLAM* — Smith, Self & Cheeseman (1986). Origin of probabilistic SLAM.
- *ORB-SLAM* — Mur-Artal, Montiel, Tardós (2015) and *ORB-SLAM2* (2017). Feature-based visual SLAM baseline.
- *Dynamic Window Approach* — Fox, Burgard, Thrun (1997). Local planner still used on mobile robots.
- *Deep Residual Learning for Image Recognition* — He et al. (2015). The ResNet paper behind most vision backbones.
- *Proximal Policy Optimization* — Schulman et al. (2017). Workhorse policy-gradient algorithm.
- *RT-2: Vision-Language-Action Models* — Google DeepMind (2023). The VLA paradigm.

---

## Online Courses and MOOCs

Full list at [[MOOCs_and_Courses]].

- [Modern Robotics Specialization — Coursera / Northwestern](https://www.coursera.org/specializations/modernrobotics) (Lynch & Park)
- [Robotics Specialization — Coursera / Penn](https://www.coursera.org/specializations/robotics)
- [MIT OCW 6.141 — Robotic Science and Systems I](https://ocw.mit.edu/courses/6-141-robotic-science-and-systems-i-fall-2020/)
- [Underactuated Robotics — MIT / Russ Tedrake](https://underactuated.mit.edu/) (free online textbook + lectures)
- [Self-Driving Cars Specialization — Coursera / Toronto](https://www.coursera.org/specializations/self-driving-cars)

---

## Simulators and Tools

Full list at [[Simulators_and_IDEs]].

- [Gazebo](https://gazebosim.org/) — standard ROS-integrated physics simulator.
- [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac/sim) — GPU-accelerated, photorealistic, RL-friendly.
- [MuJoCo](https://mujoco.org/) — fast rigid-body dynamics; now open-source (DeepMind).
- [Webots](https://cyberbotics.com/) — open-source cross-platform simulator.
- [RViz2](https://github.com/ros2/rviz) — the ROS 2 visualizer.
- [Foxglove Studio](https://foxglove.dev/) — modern rosbag2 visualization and debugging.

---

## Hardware Starter Platforms

Full list at [[Hardware_Shopping_List]].

- [TurtleBot 4](https://www.clearpathrobotics.com/turtlebot/) — official ROS 2 educational mobile robot.
- [NVIDIA Jetson Orin Nano / AGX Orin](https://developer.nvidia.com/embedded-computing) — edge AI compute.
- [OAK-D (OpenCV AI Kit)](https://docs.luxonis.com/) — stereo + depth + on-device NN.
- [Unitree Go2 / G1](https://www.unitree.com/) — quadruped and humanoid research platforms.
- [Raspberry Pi 5](https://www.raspberrypi.com/) — low-cost Linux compute for teaching.

---

## Obsidian Plugins (Recommended)

Plugins used by this vault.

- [Dataview](https://blacksmithgu.github.io/obsidian-dataview/) — powers [[Robotics_Vault_Dashboard]] and several indexes. Essential.
- [Canvas](https://help.obsidian.md/Canvas) — visual mind-maps, system diagrams.
- [Excalidraw](https://github.com/zsviczian/obsidian-excalidraw-plugin) — inline hand-drawn diagrams.
- [Advanced Tables](https://github.com/tgrosinger/advanced-tables-obsidian) — sane table editing.

---

## Research Labs and Organizations

A few starting points. Many more at [[Key_Figures_and_Labs]].

- [Open Robotics](https://www.openrobotics.org/) — maintainers of ROS 2 and Gazebo.
- [CMU Robotics Institute](https://www.ri.cmu.edu/) — one of the oldest robotics research institutes.
- [ETH Zurich ASL](https://asl.ethz.ch/) — autonomous systems, legged robotics, SLAM.
- [Stanford IPRL / SVL](https://iprl.stanford.edu/) — manipulation, imitation learning.
- [Berkeley AUTOLab / BAIR](https://autolab.berkeley.edu/) — grasping, dexterous manipulation.
- [MIT CSAIL](https://www.csail.mit.edu/) — broad robotics + AI research.

---

## Who Links Here

This Dataview query lists every note that references `Resources_Index`, so you can trace how resources propagate through the vault.

```dataview
TABLE file.folder AS "Chapter"
FROM [[Resources_Index]]
SORT file.folder ASC, file.name ASC
```
