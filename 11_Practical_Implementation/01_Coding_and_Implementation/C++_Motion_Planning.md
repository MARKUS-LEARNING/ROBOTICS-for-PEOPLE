---
title: C++ Motion Planning
description: "How motion planning is implemented in C++ for robot arms and mobile robots: the stack of robot model, collision checking (FCL), planners (OMPL), trajectory processing and execution, as used inside MoveIt and Nav2. Includes a computed example showing why collision checking dominates planning time, and the 2026 speed-ups from vectorised (VAMP) and GPU (cuRobo) planning."
aliases:
  - Motion Planning in C++
tags:
  - cpp
  - motion-planning
  - collision-detection
type: tutorial
level: advanced
subfield:
  - planning
  - software
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[C++]]"
  - "[[Path_Planning_Algorithms]]"
related:
  - "[[OMPL]]"
  - "[[MoveIt2]]"
  - "[[Collision_Detection]]"
  - "[[Nav2_Navigation]]"
key_takeaways:
  - "A planning stack is: robot model (URDF), scene representation, collision checker, planner, path smoothing and time-parameterisation, then execution through a controller."
  - "Collision checking dominates: a planner making 10,000 checks at 50 µs each spends 0.5 s checking; at 1 µs per check it spends 10 ms. That is why vectorised and GPU checking changed planning speeds."
  - "In 2026 C++ planning is usually reached through MoveIt or Nav2, with OMPL 2.0 (VAMP) and cuRobo as fast backends."
sources:
  - "Şucan, I. A., Moll, M. and Kavraki, L. E. (2012). The Open Motion Planning Library. https://ieeexplore.ieee.org/document/6377468"
  - "Thomason, W., Kingston, Z. and Kavraki, L. E. (2023). Motions in Microseconds via Vectorized Sampling-Based Planning. https://arxiv.org/abs/2309.14545"
  - "Sundaralingam, B. et al. (2023). cuRobo: Parallelized Collision-Free Minimum-Jerk Robot Motion Generation. https://arxiv.org/abs/2310.17274"
  - "moveit (2026). moveit2: MoveIt for ROS 2. https://github.com/moveit/moveit2"
---

# C++ Motion Planning

Motion planners for real robots are mostly written in [[C++]] because they must check thousands of candidate motions per second against detailed robot and world models. This note describes the pieces of a planning stack and how they are used in practice.

> **Etymology.** See [[Path_Planning_Algorithms]].

## The stack

| Stage | Purpose | Common C++ components |
|---|---|---|
| Robot model | Links, joints, limits, collision shapes | URDF/SRDF parsers, KDL, Pinocchio |
| Scene | Obstacles from CAD and sensors | Octomap, meshes, point clouds (PCL) |
| Collision checking | Is this configuration (or motion) free? | FCL, Coal, Bullet; vectorised or GPU checkers |
| Planner | Find a feasible (or optimal) path | OMPL, Pilz industrial planners, STOMP, CHOMP |
| Post-processing | Shortcut, smooth, add timing within velocity and acceleration limits | Time-optimal path parameterisation (TOTG), Ruckig |
| Execution | Send the trajectory to controllers and monitor it | ros2_control `joint_trajectory_controller` |

[[MoveIt2]] packages all of this for arms, [[Nav2_Navigation]] for mobile bases.

## Worked example: where the time goes

A sampling planner solving a cluttered pick might evaluate 10,000 configurations and edges. If each collision check of a 7-joint arm against a detailed scene takes 50 µs, checking alone takes $10{,}000 \times 50\ \mu\text{s} = 0.5$ s. Sampling, nearest-neighbour search and smoothing add much less. Cutting the check to 1 µs, for example with simpler collision geometry (spheres or capsules instead of meshes), SIMD vectorisation, or batching on a GPU, cuts the total to about 10 ms.

This is the idea behind OMPL 2.0's VAMP backend, reported to plan at up to 25 kHz on common arms, and behind GPU planners such as cuRobo, which evaluate thousands of candidate trajectories in parallel.

## Practical tips

- Use **simplified collision geometry** for planning (capsules, convex hulls) and detailed meshes only where needed.
- **Pad** collision models a little for safety, but not so much that narrow passages close.
- Always **time-parameterise**: a geometric path is not a trajectory until velocity and acceleration limits are applied.
- Measure **success rate and planning time** on benchmark scenes before tuning planners.
- Treat planning failures as normal: plan again from a different seed, relax constraints, or ask a higher layer to change the goal.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[C++_Motion_Planning]])
```
