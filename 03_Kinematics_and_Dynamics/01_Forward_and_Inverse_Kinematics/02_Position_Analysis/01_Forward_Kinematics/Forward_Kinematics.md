---
title: Forward Kinematics (FK)
description: Describes Forward Kinematics (FK), the process of calculating the end-effector pose from joint variables for manipulators, or chassis velocity from wheel velocities for mobile robots.
tags:
  - kinematics
  - manipulator
  - mobile-robot
  - mapping
  - pose-estimation
  - DH-convention
layout: default
category: robotics
author: Jordan_Smith_&_le_Chat
date: 2025-04-28
permalink: /forward_kinematics/
related:
  - "[[Kinematics]]"
  - "[[Inverse_Kinematics]]"
  - "[[Homogeneous_Transformation_Matrix]]"
  - "[[Joint_Space]]"
  - "[[Cartesian_Space]]"
  - "[[End-Effector]]"
  - "[[Kinematic_Chains]]"
  - "[[Manipulator_Arm_Types]]"
  - "[[Mobile_Robots]]"
  - "[[Differential_Drive]]"
  - "[[Jacobian_Matrix]]"
---

# Forward Kinematics (FK)

**Forward Kinematics (FK)**, also known as **direct kinematics**, refers to the use of the kinematic equations of a robot to compute the position and orientation of its end-effector (or tool frame) in a specific reference frame (typically the base frame or world frame), given the values of its joint parameters (angles for revolute joints, displacements for prismatic joints).

Essentially, forward kinematics solves the problem: **Given the joint configuration, where is the robot's end-effector?** This represents a mapping from the robot's [[Joint_Space]] to its [[Cartesian_Space]] or task space.

---

## Serial Manipulators

For serial manipulators (like typical robot arms), forward kinematics involves calculating the cumulative effect of the transformations across each link in the [[Kinematic_Chains]].

* **Method:** The standard approach is to:
    1.  Assign coordinate frames {0}, {1}, ..., {N} to the base and each link, often following a systematic procedure like the [[DH_Parameters|Denavit-Hartenberg Convention|Denavit-Hartenberg (DH) convention]].
    2.  Determine the [[Homogeneous_Transformation_Matrix|homogeneous transformation matrix]] $^{i-1}T_i$ relating frame {i} to frame {i-1}. This matrix is a function of the fixed link parameters (like link length and twist) and the variable joint parameter $q_i$ (either angle $\theta_i$ or displacement $d_i$).
    3.  Calculate the overall transformation from the base frame {0} to the final link frame {N} by multiplying the individual link transformations:
        $$
        ^0T_N = ^0T_1 \, ^1T_2 \, \dots \, ^{N-1}T_N
        $$
    4.  If a specific tool frame ${T}$ is defined relative to the last link frame ${N}$ by a constant transformation $^N T_T$, the final tool pose relative to the base is:
        $$
        ^0T_T = ^0T_N \, ^N T_T
        $$
* **Denavit-Hartenberg (DH):** The DH convention provides a standardized method for assigning link frames and defining the four parameters ($\theta_i, d_i, a_i, \alpha_i$) that characterize the transformation $^{i-1}T_i$.
* **Solution:** For serial chain manipulators, the forward kinematics problem generally has a unique, closed-form analytical solution. Given the joint values, calculating the end-effector pose is computationally straightforward.

---

## Mobile Robots

For mobile robots, the term "forward kinematics" typically refers to the instantaneous velocity relationships rather than static pose calculation from joint angles (as mobile robots usually don't have a fixed base in the same way manipulators do).

* **Concept:** It describes how the robot chassis's linear velocity ($v_x, v_y$) and angular velocity ($\omega_z$) in a chosen frame (local robot frame or global world frame) result from the velocities of its actuated components (e.g., wheel angular velocities $\phi_L, \phi_R$).
* **Example ([[Differential_Drive]]):** For a differential drive robot with wheel radius $r$ and wheel separation $b$, the forward kinematics relating wheel speeds to chassis velocity $(v_x, \omega_z)$ in the local frame are:
    $$v_x = \frac{r (\phi_R + \phi_L)}{2}$$
    $$\omega_z = \frac{r (\phi_R - \phi_L)}{b}$$

---

## Purpose and Application

Forward kinematics is fundamental in robotics for:

* **State Estimation:** Determining the current pose of the end-effector based on readings from joint sensors.
* **Trajectory Planning:** Verifying that planned points in joint space correspond to desired points in Cartesian space.
* **Control:** Computing the current end-effector pose or velocity for feedback control loops.
* **Simulation & Visualization:** Calculating the robot's geometry for display or physics simulation.

---

## Relation to Inverse Kinematics

Forward kinematics is the counterpart to [[Inverse_Kinematics]] (IK). While FK computes the end-effector pose from given joint values, IK computes the required joint values to achieve a desired end-effector pose. FK usually has a unique, easily computed solution, whereas IK is often more complex, potentially having multiple solutions, no solutions, or requiring numerical methods.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #robotics OR #kinematics WHERE contains(file.outlinks, [[Forward_Kinematics]])
