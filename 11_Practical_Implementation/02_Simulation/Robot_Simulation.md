---
title: Robot Simulation
description: "Section overview: what a robot simulator computes (dynamics, contact, sensors, rendering), what simulation is used for (testing, training, data, digital twins), the trade-off between speed and fidelity, and a map of the 2026 simulator landscape."
aliases:
  - Simulation
  - Robot Simulator
  - Robotics Simulation
  - Simulators
tags:
  - simulation
  - robotics-software
  - dynamics
  - robot-learning
type: overview
level: intermediate
subfield:
  - software
  - robot-learning
status: keystone
author: Jordan_Smith
date: 2026-11-15
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Dynamics]]"
  - "[[Kinematics]]"
related:
  - "[[Physics_Engines]]"
  - "[[GPU-Parallel_Simulation]]"
  - "[[Sim-to-Real_Transfer]]"
  - "[[Gazebo_Simulator]]"
  - "[[MuJoCo]]"
  - "[[Isaac_Lab]]"
  - "[[World_Models]]"
key_takeaways:
  - "A robot simulator steps the equations of motion forward in time, resolves contacts, and imitates sensors and cameras."
  - "Simulation is used to test software, train learned policies, generate data and run digital twins; each use wants a different balance of speed and fidelity."
  - "In 2026 the field splits into ROS-oriented simulators (Gazebo), learning-oriented GPU simulators (Isaac Lab, MuJoCo Playground, Genesis, ManiSkill) and engines underneath (MuJoCo, Newton, PhysX)."
sources:
  - "Collins, J. et al. (2021). A Review of Physics Simulators for Robotic Applications. https://ieeexplore.ieee.org/document/9386154"
  - "Google DeepMind (2026). MuJoCo 4: Studio & Filament, Deformables, Speculative Contacts. https://github.com/google-deepmind/mujoco/discussions/3773"
  - "NVIDIA (2026). Isaac Lab 3.0 release notes. https://github.com/isaac-sim/IsaacLab/releases/tag/v3.0.0"
  - "Open Robotics (2026). Installing Gazebo with ROS (Gazebo Jetty documentation). https://gazebosim.org/docs/latest/ros_installation/"
  - "Zhao, W., Peña Queralta, J. and Westerlund, T. (2020). Sim-to-Real Transfer in Deep Reinforcement Learning for Robotics: a Survey. https://arxiv.org/abs/2009.13303"
---

# Robot Simulation

A **robot simulator** is a program that predicts what a robot and its surroundings would do, without the robot. It steps the laws of motion forward in small time increments, works out where things touch and push on each other, and imitates the robot's sensors and cameras. Engineers use it to test code before it touches hardware; robot-learning researchers use it to give a policy millions of attempts it could never get in the real world.

> **Etymology.** *Simulate* is from Latin *simulare*, "to make like, imitate, pretend", from *similis*, "like". For centuries it meant feigning. Its modern sense, imitating a system's behaviour with a model, took hold in the 1940s–50s, when the Monte Carlo method and early computers made it possible to imitate neutron diffusion and, later, aircraft and factories.

## What a simulator computes, every step

```
read actuator commands ─► compute forces ─► detect contacts ─► solve dynamics (q̈) ─► integrate (q, q̇ at t + Δt) ─► render sensors ─► repeat
```

| Part | What it does | Where to read more |
|---|---|---|
| **Kinematics** | Where every link is, given joint angles | [[Kinematics]] |
| **Dynamics** | How joints accelerate under torques, gravity and contact forces | [[Dynamics]], [[Rigid_Body_Dynamics]] |
| **Collision and contact** | Which bodies touch, and the forces that stop them interpenetrating | [[Physics_Engines]] |
| **Actuators** | How a motor turns a command into torque, with limits, delays and gearing | [[Electric_Motors]] |
| **Sensors** | Simulated cameras, depth, LIDAR, IMU, force and touch | [[Sensors]] |
| **Rendering** | Images for cameras (and people); from simple rasterisation to ray tracing | [[Camera_Systems]] |

## What simulation is for

| Use | What matters most | Typical tool |
|---|---|---|
| **Testing software** (navigation, ROS nodes) | Sensor realism, ROS integration | [[Gazebo_Simulator]] |
| **Training policies with RL** | Raw speed, many parallel worlds, stable contact | [[GPU-Parallel_Simulation]], [[Isaac_Lab]], MuJoCo Playground |
| **Generating training data** | Visual realism, variety | Isaac Sim, video [[World_Models]] |
| **Planning and control research** | Exact, differentiable dynamics | Drake, [[MuJoCo]] |
| **Digital twins** | Matching one specific real cell or robot | Calibrated simulators, see [[Sim-to-Real_Transfer]] |
| **Teaching** | Easy install, good GUI | Webots, Gazebo, MuJoCo in the browser |

## Speed versus fidelity

Every simulator chooses a point on one trade-off: faster means simpler models, larger time steps and cruder contact; more faithful means slower. GPU simulators run thousands of simplified worlds at once; photorealistic simulators render a few very carefully. The question is never "which simulator is most realistic?" but "realistic enough in *which* way for *this* task?" A locomotion policy needs accurate foot contact and motor response, not photorealism; a vision-based grasping policy needs realistic images more than perfect friction.

## Worked example: how much faster than reality?

A simulator's **real-time factor** is simulated seconds per wall-clock second. Suppose one simulated quadruped runs at real-time factor 20 on one CPU core, and a GPU simulator runs 4,096 copies at real-time factor 2 each. The GPU produces $4096 \times 2 = 8192$ simulated seconds per second, versus $20$ for the CPU core: about 400 times more experience. One hour of wall time on the GPU is roughly $8192 \times 3600 / 86400 \approx 341$ days of robot experience. This arithmetic is why reinforcement learning for robots moved to GPUs; see [[GPU-Parallel_Simulation]].

## The 2026 landscape

| Simulator | Built on | Best for | Licence |
|---|---|---|---|
| **Gazebo** (Jetty, Harmonic) | DART, Bullet and other engines | ROS 2 development and testing | Apache-2.0 |
| **MuJoCo 4** and **MuJoCo Playground 1.0** | MuJoCo, MJX, MuJoCo Warp | Contact-rich control research, GPU RL | Apache-2.0 |
| **Isaac Lab 3.0** | PhysX or Newton; Isaac Sim optional | GPU-parallel robot learning at scale | BSD-3-Clause (core) |
| **Isaac Sim** | PhysX, RTX rendering | Photorealistic simulation, synthetic data, ROS bridge | NVIDIA licence |
| **Newton** | NVIDIA Warp | Open GPU physics engine with several solvers (MuJoCo Warp, VBD, MPM) | Apache-2.0 |
| **Genesis** | Its own multi-solver engine | GPU multi-physics robot learning | Apache-2.0 |
| **ManiSkill3** | SAPIEN / PhysX | GPU-parallel manipulation benchmarks | Apache-2.0 |
| **Drake** | Its own engine | Model-based planning, optimisation, verification | BSD-3-Clause |
| **Webots** | ODE | Education, quick prototypes | Apache-2.0 |

See [[Simulators_and_IDEs]] for the longer reference, including visualisation tools and editors.

## The gap

No simulator matches reality exactly. The differences (in appearance, physics, actuation and the task itself) are the **sim-to-real gap**, and closing it is its own discipline: [[Sim-to-Real_Transfer]]. Learned [[World_Models]] offer a complementary route: predicting the world from data rather than from equations.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Robot_Simulation]])
```
