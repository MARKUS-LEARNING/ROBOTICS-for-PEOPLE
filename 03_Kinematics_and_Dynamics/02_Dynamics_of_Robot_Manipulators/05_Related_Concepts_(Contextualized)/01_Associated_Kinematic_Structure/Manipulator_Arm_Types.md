---
title: Manipulator Arm Types
description: Classifies common robotic manipulator arm types based on their kinematic structure, primarily the first three joints determining positioning capabilities.
tags:
  - kinematics
  - manipulator
  - robot-types
  - classification
  - industrial-robot
layout: default
category: robotics
author: Jordan_Smith_&_Gemini
date: 2025-04-28
permalink: /manipulator_arm_types/
related:
  - "[[Kinematics]]"
  - "[[Degrees_of_Freedom]]"
  - "[[Workspace]]"
  - "[[Kinematic_Chains]]"
  - "[[Prismatic_Joint]]"
  - "[[Revolute_Joint]]"
  - "[[Industrial_Arms]]"
  - "[[Parallel_Mechanisms_and_Robots]]"
  - "[[Kinematically_Redundant_Manipulators]]"
---

# Manipulator Arm Types

Robot manipulators, particularly [[Industrial_Arms]], are typically classified based on their kinematic structure, which determines their motion capabilities and workspace geometry. The classification often focuses on the arrangement and type of the first three joints of the [[Kinematic_Chains]], which primarily define the manipulator's positioning capabilities (the "regional structure"). Subsequent joints, usually 2 or 3 revolute joints forming a "wrist", handle the orientation of the [[End_Effector_Definition|End Effector]] (the "orientation structure").

The most common joint types are [[Revolute_Joint|Revolute (R)]] (rotary) and [[Prismatic_Joint|Prismatic (P)]] (sliding/linear).

---

## Common Serial Manipulator Configurations

These configurations describe the first three joints starting from the base:

1.  **Cartesian (PPP):**
    * **Structure:** Consists of three mutually orthogonal Prismatic joints. Often implemented in a gantry configuration.
    * **Characteristics:**
        * Simple [[Forward_Kinematics]] and [[Inverse_Kinematics]].
        * Rectangular [[Workspace]].
        * Structurally very stiff, suitable for high precision and large payloads.
        * Requires a large operating volume relative to its reach.
        * Can be slower due to large moving mass.
    * **Examples:** Large gantry robots for material handling (e.g., cars), CNC machines.

2.  **Cylindrical (RPP):**
    * **Structure:** A Revolute joint at the base (vertical axis), followed by a Prismatic joint for radial extension, and another Prismatic joint for vertical motion.
    * **Characteristics:** Cylindrical workspace. Less common in modern designs.
    * **Examples:** Early robots like the Versatran.

3.  **Spherical (RRP or Polar):**
    * **Structure:** A Revolute joint at the base (vertical axis), a Revolute joint for elevation, and a Prismatic joint for radial reach.
    * **Characteristics:** Spherical workspace.
    * **Examples:** The first industrial robot, the Unimate.

4.  **Articulated (RRR or Anthropomorphic/Elbow):**
    * **Structure:** Typically consists of three Revolute joints: a base rotation (waist), a shoulder elevation joint, and an elbow joint. The shoulder and elbow axes are often parallel.
    * **Characteristics:**
        * Most common configuration for industrial robots.
        * Large, dexterous workspace relative to the robot's size.
        * Ability to reach around obstacles.
        * More complex kinematics and dynamics compared to Cartesian.
    * **Examples:** PUMA 560, KUKA Famulus, ASEA IRB-6, most modern 6-axis industrial arms. (The early Stanford Arm was a 5R 1P variation).

5.  **SCARA (Selective Compliance Assembly Robot Arm) (RRP or RRP):**
    * **Structure:** Usually two parallel vertical Revolute joints (shoulder and elbow) followed by a Prismatic joint for vertical motion. A final Revolute joint for wrist rotation is common (making it 4 DoF total).
    * **Characteristics:**
        * Selectively compliant: high stiffness vertically, lower stiffness horizontally.
        * Very fast for planar tasks (pick-and-place, assembly).
        * Actuators for the first two joints can be placed in the base, reducing moving mass.
    * **Examples:** Adept SCARA robots, Hirata AR-300.

---

## Other Structures

* **[[Parallel_Mechanisms_and_Robots|Parallel Manipulators]]:** The end-effector is connected to the base by multiple independent kinematic chains working in parallel.
    * **Characteristics:** Typically higher speed, stiffness, and accuracy than serial arms, but often have smaller and more complex workspaces. Kinematics can be complex (especially forward kinematics).
    * **Examples:** Delta robot (often used for high-speed picking), Stewart-Gough platform (used in flight simulators).

* **[[Kinematically Redundant Manipulators|Redundant Manipulators]]:** Possess more [[Degrees_of_Freedom]] than required for the task (typically >6 DoF for spatial positioning and orientation).
    * **Characteristics:** Increased dexterity allows for [[Singularities|singularity avoidance]], obstacle avoidance, and optimization of secondary criteria (e.g., minimizing joint torques).
    * **Examples:** Human arms (7 DoF), DLR Lightweight Robot (7 DoF), snake-like robots.

---

## Wrists

Most positioning structures are combined with a wrist mechanism, typically comprising 2 or 3 Revolute joints, to provide orientation capabilities for the end-effector. A common configuration is a **spherical wrist**, where the axes of three R joints intersect at a single point (the wrist center). This design simplifies the inverse kinematics problem.

