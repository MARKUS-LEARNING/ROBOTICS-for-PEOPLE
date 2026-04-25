---
title: Kinematics and Dynamics
description: Chapter overview. Kinematics is the geometry of motion (where, how fast). Dynamics is the physics of motion (why, how forced). Together they are the language every robot mechanism is described in.
tags:
  - robotics
  - kinematics
  - dynamics
  - mechanics
  - chapter-overview
type: Chapter Overview
application: Mechanics of robotic systems
layout: default
category: robotics
author: Jordan_Smith
date: 2025-04-29
permalink: /kinematics-and-dynamics/
related:
  - "[[Kinematics]]"
  - "[[Dynamics]]"
  - "[[Forward_Kinematics]]"
  - "[[Inverse_Kinematics]]"
  - "[[Jacobian_Matrix]]"
  - "[[Manipulator_Dynamics]]"
  - "[[Trajectory_Planning]]"
  - "[[Mechanisms_and_Actuation]]"
---

# Kinematics and Dynamics

This chapter is the mechanical heart of the vault. Two complementary disciplines describe every robot:

- **[[Kinematics]]** — the geometry of motion: position, velocity, acceleration, *force-free*.
- **[[Dynamics]]** — the physics of motion: mass, inertia, force, torque, *force-driven*.

Together they constitute *mechanics*, the science of bodies in motion. The two halves were separated as scientific subjects in 1834 by André-Marie Ampère, who coined the term *kinematics* to mark the study of pure motion-geometry as distinct from the study of its causes.

> **Etymology.** *Kinematics* — Greek *kinēma*, "movement." *Dynamics* — Greek *dynamis*, "power" or "force." *Mechanics* — Greek *mēchanē*, "machine" or "device" (the same root as *machine* itself).

---

## Why both, in one chapter

A complete robot model needs both halves:

| Question | Kinematics? | Dynamics? |
|---|---|---|
| Where will the gripper end up? | Yes | No |
| Will the robot reach this pose? | Yes | No |
| What joint angles produce this pose? | Yes (IK) | No |
| What torque does the elbow motor need? | No | Yes |
| Can the robot lift a 5 kg payload? | No | Yes |
| How does the arm respond when bumped? | No | Yes |
| Is this trajectory smooth in joint space? | Yes | No |
| Is this trajectory feasible at runtime? | No | Yes (torque limits, gravity comp) |

In practice, the two interleave constantly: motion plans live in kinematic space, but the controllers that execute them are dynamic.

---

## Chapter map

### 1. Forward and Inverse Kinematics — `01_Forward_and_Inverse_Kinematics/`

The geometry layer. Frames, links, joints, kinematic chains, and the maps between joint space and task space:

- [[Kinematics]] — the chapter's keystone
- [[Joints]], [[Links]], [[Joint_Kinematics]], [[Kinematic_Chains]]
- [[Cartesian_Space]], [[Configuration_Space]]
- [[Homogeneous_Transformation]] — the $4 \times 4$ workhorse
- [[Forward_Kinematics]] and [[DH_Parameters]] / [[Product_of_Exponentials]] — the two standard parameterizations
- [[Inverse_Kinematics]] — closed-form ([[Analytical_IK_Solutions]]) vs. iterative ([[Numerical_IK_Solutions]])
- [[Jacobian_Matrix]] — velocity, force, singularity, and manipulability all live here
- [[Manipulability]], [[Instantaneous_Center_of_Rotation_(ICR)]]

### 2. Dynamics of Robot Manipulators — `02_Dynamics_of_Robot_Manipulators/`

The physics layer. Forces, torques, inertias, and equations of motion:

- [[Dynamics]] — the chapter's keystone for forces
- [[Newton-Euler_Equations]], [[Lagrangian_Dynamics]] — the two classical derivations
- [[Manipulator_Dynamics]] — the canonical $M(\boldsymbol{\theta}) \ddot{\boldsymbol{\theta}} + C \dot{\boldsymbol{\theta}} + \mathbf{g} = \boldsymbol{\tau}$
- [[Inertia]], [[Angular_Momentum]], [[Rotational_Dynamics]], [[Rigid_Body_Dynamics]]
- [[Torque]], [[Torque_and_Force_Calculations]], [[Friction]], [[Stiffness]], [[Stability]]
- [[Linear_Quadratic_Regulator_(LQR)]] — the optimal-control bridge
- Specialized cases: [[Parallel_Mechanisms_and_Robots]], [[Stewart_Platform]], [[Singularities]], [[Nonholonomic_Constraint]], [[Locomotion]], [[Robot_Gaits]]

### 3. Trajectory Planning — `03_Trajectory_Planning/`

What path does the robot take through configuration space?

- [[Trajectory_Planning]], [[Motion_Planning]], [[Path_Planning]], [[Motion_Control]]
- [[Search_Algorithms]], [[Sampling-Based_Planning]]
- [[Configuration_Space]], [[Collision_Detection]], [[Backlash]]

### 4. Mechanisms and Actuation — `04_Mechanisms_and_Actuation/`

The hardware that turns commanded torques into actual motion:

- [[Mechanisms_and_Actuation]] — the section overview
- Actuators: [[Electric_Motors]], [[Servo_Motors]], [[Stepper_Motors]], [[Hydraulic_Systems]], [[Pneumatic_Systems]], [[Variable_Stiffness_Actuators]]
- Transmission: [[Transmission_Mechanisms]], [[Gears]], [[Ball_Screw]], [[Remote_Actuation]]
- Design: [[Mechanism_Design]], [[Mechanics]]

---

## The two equations to memorize

Almost every concept in the chapter folds back into one of these two equations.

### Forward kinematics (geometry)

$$
T_{\text{ee}}(\boldsymbol{\theta}) = T_{0,1}(\theta_1) \, T_{1,2}(\theta_2) \, \cdots \, T_{n-1, n}(\theta_n)
$$

A composition of $4 \times 4$ rigid-body transforms — one per joint — yielding the end-effector pose as a function of joint angles. Cheap to evaluate, exact, and uniquely defined. The Jacobian $J = \partial T_{\text{ee}} / \partial \boldsymbol{\theta}$ falls out by differentiation.

### Manipulator equation (physics)

$$
M(\boldsymbol{\theta}) \ddot{\boldsymbol{\theta}} + C(\boldsymbol{\theta}, \dot{\boldsymbol{\theta}}) \dot{\boldsymbol{\theta}} + \mathbf{g}(\boldsymbol{\theta}) = \boldsymbol{\tau} + J^T \mathbf{F}_{\text{ext}}
$$

The torque-balance for an $n$-link manipulator: inertia × acceleration, plus velocity-coupled (Coriolis/centrifugal) terms, plus gravity, equals applied torque plus reflected external wrench.

Together these two equations describe every serial-arm robot from a desktop SCARA to a humanoid.

---

## How this chapter feeds the rest of the vault

- **Robot Control** ([[Control_Theory]], [[PID_Control]], [[Model_Predictive_Control]]) — every controller is a feedback law on the dynamic model.
- **AI / Machine Learning** ([[Reinforcement_Learning]], imitation learning) — policies output actions in joint or task space, decoded by kinematics.
- **Sensors & Perception** — pose estimates from [[SLAM]] live in $SE(3)$, the same Lie group as kinematic transforms.
- **ROS** — the `tf2` library and `URDF` description format codify exactly the kinematic-tree formalism described here.

---

## Recommended reading

- Lynch & Park, *Modern Robotics* (free online) — the modern PoE-first textbook covering both kinematics and dynamics
- Spong, Hutchinson, Vidyasagar, *Robot Modeling and Control* — staple comprehensive textbook
- Murray, Li, Sastry, *A Mathematical Introduction to Robotic Manipulation* (free online) — screw-theoretic foundation
- Featherstone, *Rigid Body Dynamics Algorithms* — the canonical algorithmic reference for fast computation
- Tedrake, *Underactuated Robotics* (free MIT course) — when the simple cases run out

---

## Dataview

```dataview
LIST FROM #kinematics OR #dynamics OR #robotics WHERE contains(file.outlinks, [[Kinematics_and_Dynamics]])
```
