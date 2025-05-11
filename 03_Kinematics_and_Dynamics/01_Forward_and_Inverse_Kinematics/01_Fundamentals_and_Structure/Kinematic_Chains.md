---
title: Kinematic Chains
description: "Defines Kinematic Chain: An assembly of rigid bodies (links) connected by joints, forming the structural skeleton of a robot or mechanism."
tags:
  - glossary-term
  - kinematics
  - mechanism
  - structure
  - link
  - joint
layout: default
category: robotics
author: Jordan_Smith_and_le_Chat
date: 2025-04-28
permalink: /kinematic_chain/
related:
  - "[[Links]]"
  - "[[Joint_Kinematics]]"
  - "[[Degrees_of_Freedom]]"
  - "[[Forward_Kinematics]]"
  - "[[Inverse_Kinematics]]"
  - "[[Manipulator_Arm_Types]]"
  - "[[Parallel_Mechanisms_and_Robots]]"
---

# Kinematic Chain

A **kinematic chain** is an assembly of rigid bodies, called **links**, connected by **joints** that allow relative motion between the links. Kinematic chains form the fundamental structure, or skeleton, of most robot manipulators and mechanisms. The study of the motion of these chains, without regard to the forces causing it, is the core of [[Kinematics]].

---
![[Kinematic-chain-for-the-UR5-robot.png]]
<font size=1>*source: https://www.researchgate.net/figure/Kinematic-chain-for-the-UR5-robot_fig1_352115594*</font>
---

## Components

* **Links**: These are the rigid bodies that make up the chain. For kinematic analysis, a link's primary function is to maintain a fixed geometric relationship (distance and orientation) between the axes of the joints it connects.
* **Joints**: These are the connections between links that allow constrained relative motion. Common joints in robotics include [[Revolute Joint|revolute (rotary)]] and [[Prismatic Joint|prismatic (sliding)]] joints, each typically providing one [[Degrees_of_Freedom]] (DoF). Other joint types like spherical or universal joints provide multiple DoF.

---

## Types of Kinematic Chains

* **Open Kinematic Chain (Serial Chain)**: This is the most common structure for robot manipulators (arms). Links are connected sequentially in a chain, starting from a fixed base (link 0) and ending with an end-effector attached to the final link (link N). Each link (except the base and the final link) connects to exactly two other links. Examples include typical industrial arms like the [[PUMA_560]].
* **Closed Kinematic Chain**: This structure contains one or more closed loops, meaning there are multiple paths through the links connecting certain points. [[Parallel_Mechanisms_and_Robots|Parallel robots]], like the Stewart platform, are prime examples, where the end-effector (moving platform) is connected to the base by several independent kinematic chains. Mechanisms like four-bar linkages are also closed chains.
* **Branched Kinematic Chain**: This is a kinematic tree structure where at least one link connects to more than two other links. Humanoid robots, with a torso connecting to arms, legs, and a head, are examples of branched chains.

---

## Mathematical Representation

### Forward Kinematics

The forward kinematics of a kinematic chain describes the position and orientation of the end-effector based on the joint variables. For a serial manipulator with $n$ joints, the forward kinematics is given by:

$$
T_n = T_1 \cdot T_2 \cdot \ldots \cdot T_n
$$

where $T_i$ represents the transformation matrix for the $i$-th joint.

### Inverse Kinematics

Inverse kinematics involves finding the joint variables that achieve a desired end-effector pose. This is often more complex and may not have a unique solution. The problem can be expressed as:

$$
\text{Find } \theta_1, \theta_2, \ldots, \theta_n \text{ such that } T_n = T_{\text{desired}}
$$

where $T_{\text{desired}}$ is the desired transformation matrix for the end-effector.

### Jacobian Matrix

The Jacobian matrix $J$ relates the joint velocities $\dot{\theta}$ to the end-effector velocities $v$:

$$
v = J \dot{\theta}
$$

The Jacobian is crucial for velocity and force analysis in robotic manipulators.

---

## Relevance in Robotics

* **Structure and Mobility**: The type and arrangement of links and joints in a kinematic chain determine the robot's overall structure, its [[Degrees_of_Freedom]], and its [[Workspace]] (the set of reachable positions and orientations for the end-effector).
* **Kinematic Analysis**: Understanding the kinematic chain is essential for deriving the [[Forward_Kinematics]] (calculating end-effector pose from joint values) and [[Inverse_Kinematics]] (calculating joint values for a desired end-effector pose).
* **Velocity and Force Analysis**: The [[Jacobian_Matrix]] relates joint velocities to the end-effector's linear and angular velocity (twist) and dually relates forces/torques at the end-effector (wrench) to the required joint torques/forces.
* **Modeling**: Conventions like the Denavit-Hartenberg parameters are used to systematically describe the geometry of serial kinematic chains for analysis and control.

The concept of the kinematic chain is fundamental to designing, analyzing, and controlling robotic mechanisms.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #component OR #kinematics WHERE contains(file.outlinks, [[Kinematic_Chains]])
