---
title: Trajectory Planning
description: Describes Trajectory Planning (or Trajectory Generation), the process of computing a time-based sequence of states (position, velocity, acceleration) for robot motion.
tags:
  - kinematics
  - motion-planning
  - path-planning
  - control-theory
  - splines
  - via-points
layout: default
category: robotics
author: Jordan_Smith_&_le_Chat
date: 2025-05-03
permalink: /trajectory_planning/
related:
  - "[[Path_Planning_Algorithms]]"
  - "[[Kinematics]]"
  - "[[Dynamics]]"
  - "[[Motion_Control]]"
  - "[[Joint_Space]]"
  - "[[Cartesian_Space]]"
  - "[[Via_Points]]"
  - "[[Splines]]"
  - "[[Inverse_Kinematics]]"
  - "[[Singularities]]"
  - "[[Workspace]]"
---

# Trajectory Planning (Trajectory Generation)

**Trajectory Planning**, also often called **Trajectory Generation**, is the process of computing a time history of position, velocity, and acceleration for each [[Degrees_of_Freedom|degree of freedom]] (DoF) of a robot. The goal is to generate a smooth, executable motion that moves the robot's end-effector (for manipulators) or body (for mobile robots) from a starting configuration to a goal configuration, potentially passing through intermediate [[Via Points|via points]], while respecting kinematic and dynamic constraints.

---

## Path vs. Trajectory

It's important to distinguish between a path and a trajectory:

* **Path:** A purely geometric description of the sequence of configurations (poses) the robot should follow. It specifies the locus of points in the configuration space but contains no timing information. Path planning algorithms (see [[Path Planning Algorithms]]) typically output such geometric paths.
* **Trajectory:** A path parameterized by time. It specifies *when* the robot should be at each configuration along the path, implicitly or explicitly defining velocity and acceleration profiles at every point in time. Trajectory planning adds this crucial time-scheduling aspect to a geometric path.

---

## Planning Spaces

Trajectory planning can be performed in different configuration spaces:

### 1. [[Joint Space]] Trajectories

* **Concept:** The trajectory is planned directly in terms of the robot's joint variables ($q_i(t), \dot{q}_i(t), \ddot{q}_i(t)$) over time.
* **Method:** Typically involves:
    1. Defining start, goal, and via points in Cartesian space.
    2. Using [[Inverse_Kinematics|Inverse Kinematics (IK)]] to convert these Cartesian poses into corresponding sets of joint variables.
    3. Interpolating between these sets of joint variables using smooth mathematical functions ([[Splines]], e.g., cubic or quintic polynomials, or linear segments with parabolic blends - LSPB). Each joint follows its own time-based function, but all joints typically start and end their motion segment simultaneously.
* **Pros:**
    * Computationally simpler than Cartesian-space methods.
    * Directly generates the joint commands needed by the controller.
    * Avoids problems related to manipulator [[Singularities]] and [[Workspace]] limits during the interpolation phase itself (though IK for via points might still fail).
* **Cons:**
    * The resulting path of the end-effector in Cartesian space is generally complex (not a straight line or simple curve) and hard to predict or constrain.

### 2. [[Cartesian Space]] Trajectories (Task Space Trajectories)

* **Concept:** The trajectory is planned directly for the end-effector's pose (position and orientation) in Cartesian space over time.
* **Method:** Typically involves:
    1. Defining start, goal, and via poses in Cartesian space.
    2. Defining a geometric path in Cartesian space connecting these poses (e.g., straight line segments for position, Slerp or axis-angle interpolation for orientation).
    3. Applying a time parameterization function to the path (e.g., cubic, quintic, LSPB applied to a path parameter $s \in [0, 1]$) to generate desired Cartesian positions, velocities, and accelerations as functions of time.
    4. Using [[Inverse_Kinematics|Inverse Kinematics (IK)]] frequently (often at the control system's update rate) to convert the desired Cartesian pose/velocity/acceleration into the required joint commands ($\mathbf{q}_d, \dot{\mathbf{q}}_d, \ddot{\mathbf{q}}_d$).
* **Pros:**
    * Allows direct specification and control over the shape of the end-effector path (e.g., ensuring straight-line motion for welding or inserting).
* **Cons:**
    * Requires repeated computationally intensive IK calculations at runtime.
    * The planned Cartesian path might pass through or near kinematic [[Singularities]], potentially requiring infeasible joint velocities or causing instability.
    * The planned path might exit the manipulator's [[Workspace]].
    * Requires careful handling of orientation interpolation to ensure smoothness and uniqueness.

---

## Key Concepts in Trajectory Planning

* **[[Via Points]]:** Intermediate poses specified along a desired path. Trajectories can be designed to pass through via points precisely or smoothly blend near them.
* **Smoothness:** Trajectories are usually required to be smooth, typically implying continuous velocity ($C^1$ continuity) and often continuous acceleration ($C^2$ continuity) profiles. This avoids jerky motions, reduces wear, minimizes excitation of structural vibrations, and ensures dynamic feasibility.
* **[[Splines]]:** Piecewise polynomial functions used to interpolate between points.
    * **Cubic Polynomials:** Sufficient to satisfy constraints on position and velocity at the start and end of a segment (4 constraints).
    * **Quintic Polynomials:** Required to satisfy constraints on position, velocity, and acceleration at the start and end of a segment (6 constraints).
    * **Linear Segments with Parabolic Blends (LSPB):** Uses linear interpolation for constant velocity segments, connected by parabolic segments representing constant acceleration blends. Ensures continuous velocity.
* **Time Parameterization:** The process of assigning timing information (velocity and acceleration profiles) to a geometric path. This must respect the robot's physical limits, such as maximum joint velocities, accelerations, and [[Actuator|actuator]] torque capabilities derived from its [[Dynamics]]. Planning minimum-time trajectories under dynamic constraints is a complex optimization problem.

---

## Applications

Trajectory planning provides the time sequence of desired states (position, velocity, acceleration) used as input setpoints for the robot's [[Motion_Control]] system. It is essential for generating the smooth, coordinated, and dynamically feasible motions required for nearly all robotic tasks, including pick-and-place, welding, painting, assembly, inspection, and navigation. Trajectory specifications are typically generated from higher-level task descriptions within a robot programming language or system.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #motion-planning OR #kinematics WHERE contains(file.outlinks, [[Trajectory_Planning]])
