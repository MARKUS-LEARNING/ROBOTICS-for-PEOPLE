---
title: Isaac Lab
description: "NVIDIA's open-source framework for robot learning in GPU-parallel simulation. Isaac Lab 3.0 (October 2026) runs one task API over PhysX and the open Newton engine, makes Isaac Sim optional, and covers RL, imitation learning and evaluation from one toolchain."
aliases:
  - Isaac Lab 3.0
  - Orbit
  - IsaacLab
tags:
  - simulation
  - reinforcement-learning
  - robot-learning
  - cuda
type: tool
level: intermediate
subfield:
  - software
  - robot-learning
status: reviewed
author: Jordan_Smith
date: 2026-11-15
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[GPU-Parallel_Simulation]]"
related:
  - "[[MuJoCo]]"
  - "[[Reinforcement_Learning_for_Robots]]"
  - "[[Sim-to-Real_Transfer]]"
  - "[[Simulators_and_IDEs]]"
  - "[[Physics_Engines]]"
key_takeaways:
  - "Isaac Lab trains robot policies across thousands of GPU-parallel environments, with ready-made tasks for locomotion, manipulation and dexterity."
  - "Version 3.0 (20 October 2026) decouples tasks from physics: the same task runs on PhysX or on Newton (MuJoCo-Warp, VBD and MPM solvers), with or without Isaac Sim."
  - "Its core is BSD-3-Clause; it grew out of the Orbit framework from ETH Zurich and NVIDIA."
sources:
  - "NVIDIA, Mittal, M. et al. (2025). Isaac Lab: A GPU-Accelerated Simulation Framework for Multi-Modal Robot Learning. https://arxiv.org/abs/2511.04831"
  - "Mittal, M. et al. (2023). Orbit: A Unified Simulation Framework for Interactive Robot Learning Environments. https://arxiv.org/abs/2301.04195"
  - "NVIDIA (2026). Isaac Lab 3.0 release notes. https://github.com/isaac-sim/IsaacLab/releases/tag/v3.0.0"
  - "Makoviychuk, V. et al. (2021). Isaac Gym: High Performance GPU-Based Physics Simulation For Robot Learning. https://arxiv.org/abs/2108.10470"
---

# Isaac Lab

**Isaac Lab** is NVIDIA's open-source framework for training robot policies in simulation. It runs thousands of copies of a task in parallel on a GPU (see [[GPU-Parallel_Simulation]]) and supplies ready-made robots, sensors, tasks and training scripts for [[Reinforcement_Learning_for_Robots|reinforcement learning]], imitation learning and evaluation.

> **Etymology.** NVIDIA's robotics platform is called **Isaac**, a name widely read as a nod to Isaac Asimov, whose 1940s–50s robot stories (and Three Laws of Robotics) shaped how people imagine robots. *Lab* signals a workbench for experiments. The framework began as **Orbit** (Mittal et al. 2023), a collaboration involving ETH Zurich and NVIDIA, and was renamed Isaac Lab in 2024.

## What it provides

| Piece | What it does |
|---|---|
| **Task API** | Defines observations, actions, rewards, terminations, randomisation and curricula as composable "manager" terms |
| **Assets** | Models of common arms, hands, quadrupeds and humanoids |
| **Sensors** | Cameras, ray-cast height scanners, contact and IMU sensors |
| **Learning libraries** | Integrations with RSL-RL and other RL libraries; imitation-learning and data-generation tools |
| **Physics backends** | PhysX, or Newton with several solvers (from 3.0) |

## Isaac Lab 3.0 (generally available 20 October 2026)

From the release notes:

- **One task API across backends.** The same task runs on different physics, rendering and visualisation backends.
- **Newton backend.** A Warp-native physics engine with several solver paths, including MuJoCo-Warp, VBD (for deformables such as cables) and MPM (for particles such as liquids).
- **OV libraries.** PhysX and RTX rendering available as standalone libraries (OVPhysX, OVRTX).
- **Isaac Sim is optional.** "Kit-less" workflows run without launching Isaac Sim; Isaac Sim remains the path when you need its full PhysX/RTX stack, ROS bridge, XR or streaming.
- **Targets**: Isaac Sim 6.2, Python 3.12, PyTorch 2.11 with CUDA 13.0, NVIDIA Warp 1.17, Newton 1.6.1 and RSL-RL 5.5.1. Installation uses `uv`.

## A typical workflow

1. Pick or write a task (for example, velocity-tracking locomotion for a quadruped).
2. Choose a backend and the number of parallel environments (thousands on one GPU).
3. Train with PPO through RSL-RL; watch rewards and videos.
4. Export the policy (for example to ONNX or TorchScript) and run it on the robot's computer under its low-level controller.
5. Compare real and simulated behaviour, and adjust randomisation and models; see [[Sim-to-Real_Transfer]].

## Isaac Lab and its relatives

| | Role |
|---|---|
| **Isaac Gym** (2021) | The original end-to-end GPU RL simulator; superseded by Isaac Lab |
| **Isaac Sim** | Full simulation application: photoreal rendering, synthetic data, ROS bridge |
| **Isaac Lab** | Robot-learning framework; uses Isaac Sim, or runs kit-less on Newton or OV libraries from 3.0 |
| **Newton** | Open, Apache-2.0 GPU physics engine (Linux Foundation) used as an Isaac Lab backend |

## Requirements and licence

Isaac Lab needs an NVIDIA GPU. Its core is released under the BSD-3-Clause licence (some components, such as the imitation-learning "mimic" extension, use Apache-2.0); Isaac Sim itself has its own NVIDIA licence.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Isaac_Lab]])
```
