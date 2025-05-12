---
title: Manipulator Arm
description: Defines a robotic manipulator arm, its key components (links, joints, actuators, end-effector), structure, and basic function.
tags:
  - glossary-term
  - manipulator
  - robot-arm
  - kinematic-chain
  - industrial-robot
  - actuator
  - end-effector
layout: default
category: robotics
author: Jordan_Smith_&_le_Chat
date: 2025-04-29
permalink: /manipulator_arm/
related:
  - "[[Robots]]"
  - "[[Glossary]]"
  - "[[Manipulator_Arm_Types]]"
  - "[[Industrial_Arms]]"
  - "[[Kinematic_Chains]]"
  - "[[Links]]"
  - "[[Joint_Kinematics]]"
  - "[[Actuator]]"
  - "[[Degrees_of_Freedom]]"
  - "[[Workspace]]"
  - "[[End-Effector]]"
  - "[[Forward_Kinematics]]"
  - "[[Inverse_Kinematics]]"
  - "[[Control_Systems]]"
  - "[[Manipulation]]"
---

# Manipulator Arm (Robotic Arm)

A **Manipulator Arm**, or **Robotic Arm**, is a type of [[Robots|robot]] designed primarily for [[Manipulation|manipulation]] tasks. It typically consists of a series of rigid [[Links|links]] connected by [[Joint_Kinematics|joints]], forming a [[Kinematic_Chains]]. One end of the chain (the base) is usually attached to a fixed surface or a [[Mobile_Robots|mobile platform]], while the other end is equipped with an [[End_Effector_Definition|End Effector]] or a tool designed to interact with objects or the environment.

While often associated with [[Industrial_Arms]] used in manufacturing, the concept applies broadly to arms found on [[Humanoid_Robots]], [[Underwater_and_Space_Robots|space station robots]], [[Mobile_Robots|mobile manipulators]], and research platforms.

---

## Key Components

* **Base:** The foundation of the arm, providing support and often containing the first joint(s). It can be fixed or mounted on a mobile platform.
* **Links:** The rigid structural segments connecting the joints.
* **[[Joint_Kinematics|Joints]]:** Allow relative motion between links. Common types are [[Revolute Joint|revolute]] (rotation) and [[Prismatic Joint|prismatic]] (linear sliding). The number and arrangement of joints determine the arm's [[Degrees_of_Freedom]] (DoF).
* **[[Actuator|Actuators]]:** Devices (e.g., electric motors, hydraulic/pneumatic cylinders) that power the movement of the joints. See [[Actuator]].
* **[[End-Effector]] (EE):** The device attached to the final link of the arm, designed to perform the specific task. Examples include grippers, welding tools, paint sprayers, drills, cameras, or simply a mounting flange for custom tools. Often interchangeable.
* **Control System:** The hardware and software responsible for calculating the required joint motions ([[Inverse_Kinematics|Inverse Kinematics]]), planning trajectories ([[Trajectory_Planning]]), and commanding the [[Actuator|actuators]] based on task requirements and [[Sensors|Sensor]] feedback. See [[Control_Systems|Control Systems]].

---

## Structure and Functionality

* **Structure:** Most manipulator arms are **serial kinematic chains**, where links and joints are arranged sequentially from base to end-effector. However, [[Parallel_Mechanisms_and_Robots|parallel]] or hybrid kinematic structures are also used, particularly for high-speed/high-precision tasks (e.g., [[Delta Robot]]). Common serial configurations are detailed in [[Manipulator_Arm_Types]].
* **Functionality:** The primary function is to position and orient the [[End_Effector_Definition|End-Effector]] within the arm's reachable [[Workspace]] to perform manipulation tasks. Key aspects defining functionality include:
    * **[[Kinematics]]:** Describes the geometric motion capabilities ([[Forward_Kinematics]], [[Inverse_Kinematics]], [[Workspace]], [[Degrees_of_Freedom]], [[Singularities]]).
    * **[[Dynamics]]:** Describes the relationship between forces/torques and motion, considering mass, inertia, gravity, etc. ([[Torque_and_Force_Calculations]], [[Computed_Torque_Control|Computed Torque Control]]).
    * **Performance Metrics:** Payload capacity, reach, speed, accuracy, and repeatability are critical specifications, especially for [[Industrial_Arms]].

Manipulator arms are fundamental components in automation, capable of performing tasks requiring precise positioning, orientation, and interaction forces within their operational space.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #robotics OR #automation WHERE contains(file.outlinks, [[Robots]])