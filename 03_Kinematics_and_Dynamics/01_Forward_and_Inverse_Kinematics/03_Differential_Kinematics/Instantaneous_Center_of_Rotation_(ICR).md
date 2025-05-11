---
title: Instantaneous Center of Rotation (ICR)
description: The Instantaneous Center of Rotation (ICR) is a concept used in the kinematic analysis of mobile robots to describe the point around which the robot rotates at any given moment.
tags:
  - kinematics
  - mobile-robot
  - wheeled-robot
  - locomotion
  - nonholonomic
  - glossary-term
  - mechanism
  - manipulator-arm
  - mobile-robot
layout: default
category: robotics
author: Jordan_Smith_and_le_Chat
date: 2025-05-02
permalink: /instantaneous_center_of_rotation/
related:
  - "[[Differential_Drive]]"
  - "[[Wheeled_Mobile_Robots]]"
  - "[[Mobile_Robots]]"
  - "[[Kinematics]]"
  - "[[Locomotion]]"
  - "[[Nonholonomic_Constraint]]"
---

# Instantaneous Center of Rotation (ICR)

The **Instantaneous Center of Rotation (ICR)** is a concept used in the kinematic analysis of mobile robots to describe the point around which the robot rotates at any given moment. It is particularly relevant for wheeled robots, such as those using a differential drive system, where the ICR helps in understanding and controlling the robot's motion.

---

## Concept of ICR

The ICR is a theoretical point about which a robot's motion can be considered as pure rotation at any instant in time. For differential drive robots, the ICR lies along the axis connecting the centers of the two drive wheels. The position of the ICR depends on the relative velocities of the wheels:

* **Straight Motion**: When both wheels have the same velocity, the ICR is effectively at infinity, and the robot moves in a straight line.
* **Turning**: When the wheels have different velocities, the ICR is located at a finite distance from the robot, causing the robot to follow a curved path.
* **Rotation in Place**: When the wheels rotate at equal speeds but in opposite directions, the ICR is at the midpoint of the axis connecting the two wheels, and the robot rotates in place.

---

## Mathematical Representation

### Position of the ICR

The distance $R$ of the ICR from the midpoint of the axis connecting the two drive wheels can be calculated as:

$$
R = \frac{v_x}{\omega_z} = \frac{b}{2} \frac{\phi_R + \phi_L}{\phi_R - \phi_L}
$$

where:
- $v_x$ is the linear velocity of the robot.
- $\omega_z$ is the angular velocity of the robot.
- $b$ is the distance between the centers of the two drive wheels (track width).
- $\phi_R$ and $\phi_L$ are the angular velocities of the right and left wheels, respectively.

### Special Cases

- **Straight Motion**: If $\phi_R = \phi_L$, then $\omega_z = 0$ and $R \to \infty$, indicating straight-line motion.
- **Rotation in Place**: If $\phi_R = -\phi_L$, then $v_x = 0$ and $R = 0$, indicating rotation in place.

---

## Impact on Robotics

- **Motion Planning**: Understanding the ICR is crucial for planning the motion of mobile robots, especially in navigating through constrained environments or avoiding obstacles.
- **Control Systems**: The concept of ICR is used in the design of control systems for mobile robots, enabling precise control over the robot's trajectory and orientation.
- **Kinematic Analysis**: The ICR is a fundamental concept in the kinematic analysis of mobile robots, helping to describe and predict the robot's motion accurately.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #kinematics OR #mobile-robot WHERE contains(file.outlinks, [[Instantaneous_Center_of_Rotation_(ICR)]])
