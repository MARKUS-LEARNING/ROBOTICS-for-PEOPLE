---
title: Wheeled Mobile Robots
description: Wheeled Mobile Robots are robotic systems that use wheels for locomotion, providing efficient and stable movement across various terrains.
tags:
  - robotics
  - mobility
  - mechanics
  - control
  - engineering
type: Robot Type
application: Efficient and stable locomotion in robotic systems
layout: default
category: robotics
author: Jordan_Smith_&_le_Chat
date: 2025-04-29
permalink: /wheeled-mobile-robots/
related:
  - "[[Robot_Design]]"
  - "[[Mechatronics]]"
  - "[[Control_Systems]]"
  - "[[Actuator]]"
  - "[[Manipulator_Arm]]"
  - "[[Legged_Robots]]"
  - "[[Sensors]]"
  - "[[Compliance]]"
---

# Wheeled Mobile Robots

**Wheeled Mobile Robots** are robotic systems that use wheels for locomotion, providing efficient and stable movement across various terrains. These robots are widely used in applications requiring mobility, such as exploration, transportation, and service tasks. Wheeled robots are favored for their simplicity, energy efficiency, and ability to navigate structured environments effectively.

---

## Types of Wheeled Mobile Robots

1. **Differential Drive Robots**: Use two independently driven wheels to achieve motion and steering. These robots are simple and maneuverable, making them suitable for indoor environments.

2. **Ackermann Steering Robots**: Use a steering mechanism similar to that of a car, with fixed rear wheels and steerable front wheels. This design is suitable for high-speed and outdoor navigation.

3. **Omnidirectional Robots**: Use special wheels, such as Mecanum or omni-wheels, that allow movement in any direction. These robots are highly maneuverable and can move sideways or rotate in place.

4. **Tricycle and Bicycle Robots**: Use three or two wheels, respectively, with various steering mechanisms to balance stability and maneuverability.

---

## Key Equations

- **Kinematics of a Differential Drive Robot**:
  $$
  \begin{cases}
  v_l = r \cdot \omega_l \\
  v_r = r \cdot \omega_r \\
  v = \frac{v_l + v_r}{2} \\
  \omega = \frac{v_r - v_l}{b}
  \end{cases}
  $$
  where $v_l$ and $v_r$ are the velocities of the left and right wheels, $r$ is the wheel radius, $\omega_l$ and $\omega_r$ are the angular velocities of the left and right wheels, $v$ is the linear velocity of the robot, $\omega$ is the angular velocity of the robot, and $b$ is the distance between the wheels.
  <br></br>

- **Ackermann Steering Geometry**:
  $$
  \delta = \tan^{-1}\left(\frac{L}{R}\right)
  $$
  where $\delta$ is the steering angle, $L$ is the wheelbase (distance between front and rear axles), and $R$ is the turning radius.
  <br></br>

- **Omnidirectional Wheel Velocity**:
  $$
  \begin{cases}
  v_{x} = \frac{v_1 + v_2 + v_3 + v_4}{4} \\
  v_{y} = \frac{v_1 - v_2 + v_3 - v_4}{4} \\
  \omega = \frac{v_1 - v_2 - v_3 + v_4}{4d}
  \end{cases}
  $$
  where $v_x$ and $v_y$ are the linear velocities in the x and y directions, $\omega$ is the angular velocity, $v_1$, $v_2$, $v_3$, and $v_4$ are the velocities of the four omnidirectional wheels, and $d$ is the distance from the wheel to the center of the robot.

---

## Impact on Robotics

- **Efficient Navigation**: Wheeled mobile robots are energy-efficient and capable of navigating structured environments with ease, making them suitable for indoor and urban applications.

- **Simplicity and Reliability**: The mechanical simplicity of wheeled designs makes these robots reliable and easy to maintain, reducing complexity in control and design.

- **Maneuverability**: Different wheel configurations allow for a range of maneuverability options, from simple differential drives to highly agile omnidirectional systems.

- **Design and Integration**: The selection and integration of wheeled mobile robot designs are important aspects of [[Robot_Design]] and [[Mechatronics]], influencing the mobility and performance of robotic systems. Wheeled robots enable efficient and stable locomotion across various terrains.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #robotics OR #automation WHERE contains(file.outlinks, [[Robots]])
