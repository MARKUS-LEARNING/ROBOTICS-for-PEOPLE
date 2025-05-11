---
title: Differential Drive
description: Describes the kinematics and characteristics of the differential drive locomotion system commonly used in mobile robots.
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
permalink: /differential_drive/
related:
  - "[[Wheeled_Mobile_Robots]]"
  - "[[Mobile_Robots]]"
  - "[[Kinematics]]"
  - "[[Locomotion]]"
  - "[[Instantaneous_Center_of_Rotation_(ICR)]]"
  - "[[Odometry]]"
  - "[[Nonholonomic_Constraint]]"
---

# Differential Drive

A **differential drive** (or differential steer) is one of the simplest and most common drive mechanisms for wheeled mobile robots. It typically consists of two wheels mounted on a common axis, each driven by a separate motor, allowing independent control of their velocities. Often, one or more passive, free-swiveling caster wheels (or skids) are added at the front and/or rear for stability.

---

## Operation Principle

The motion of a differential drive robot is controlled by adjusting the relative speeds of the two main drive wheels. Let $\phi_L$ and $\phi_R$ be the angular velocities of the left and right wheels, respectively.

* **Straight Motion**: If both wheels rotate at the same speed ($\phi_L = \phi_R$), the robot moves forward or backward in a straight line.
* **Turning**: If the wheels rotate at different speeds ($\phi_L \neq \phi_R$), the robot follows a curved path. The robot turns towards the side with the slower wheel.
* **Rotation in Place**: If the wheels rotate at equal speeds but in opposite directions ($\phi_L = -\phi_R$), the robot spins around the midpoint of the axis connecting the two wheels (zero turning radius).

---

## Kinematic Model

Assuming pure rolling and no slipping, the motion of the robot chassis can be described relative to a local reference frame {R} attached to the robot, typically with the origin P at the midpoint of the wheel axis and the $X_R$ axis pointing forward.

Let:
* $r$ be the radius of the drive wheels.
* $b$ be the distance between the centers of the two drive wheels (the track width or wheelbase).
* $v_x$ be the robot's linear velocity along its $X_R$ axis.
* $\omega_z$ be the robot's angular velocity about its vertical $Z_R$ axis.

### Forward Kinematics

The linear velocity $v_x$ is the average of the linear speeds of the two wheels:

$$
v_x = \frac{r (\phi_R + \phi_L)}{2}
$$

The angular velocity $\omega_z$ depends on the difference between the wheel speeds and the track width:

$$
\omega_z = \frac{r (\phi_R - \phi_L)}{b}
$$

### Inverse Kinematics

Given a desired linear velocity $v_x$ and angular velocity $\omega_z$, the required wheel velocities are:

$$
\phi_R = \frac{v_x + (b/2)\omega_z}{r}
$$

$$
\phi_L = \frac{v_x - (b/2)\omega_z}{r}
$$

### Instantaneous Center of Rotation (ICR)

The robot's motion at any instant can be viewed as a pure rotation about an ICR. For a differential drive robot, the ICR always lies somewhere along the line extending through the common axis of the two drive wheels. Its distance $R$ from the midpoint P is given by:

$$
R = \frac{v_x}{\omega_z} = \frac{b}{2} \frac{\phi_R + \phi_L}{\phi_R - \phi_L}
$$

If $\phi_R = \phi_L$ (straight motion), then $\omega_z = 0$ and $R \to \infty$. If $\phi_R = -\phi_L$ (rotation in place), then $v_x = 0$ and $R = 0$.

### Nonholonomic Constraint

The differential drive mechanism imposes a [[Nonholonomic Constraint|nonholonomic constraint]]: the robot cannot move instantaneously sideways. In its local frame {R}, the velocity component along the $Y_R$ axis (the wheel axis) must be zero: $v_y = 0$. This makes it a Type(2,0) mobile robot according to the mobility classification.

---

## Advantages

* **Mechanical Simplicity**: Relatively simple design, often inexpensive to build.
* **Maneuverability**: High maneuverability, capable of turning with a zero radius (spinning in place).
* **Control**: Relatively straightforward kinematic modeling and control.

---

## Disadvantages

* **Odometry Errors**: Prone to [[Odometry]] errors, primarily due to wheel slippage during turns or acceleration/deceleration, unequal wheel diameters, or variations in floor contact. Drift error (error in orientation) accumulates quickly and leads to large position errors over time.
* **Surface Dependence**: Performs best on smooth, hard surfaces. Uneven terrain can cause one wheel to lose traction or lift off the ground, leading to significant errors.
* **Caster Wheels**: Passive caster wheels, while providing stability, can introduce odometry errors and affect dynamic stability, especially at higher speeds or on uneven surfaces.
* **Straight-Line Motion**: Maintaining perfectly straight motion can be difficult due to slight differences between motors, wheels, and wheel-ground interaction on either side.

---

## Examples

Differential drive is used in a vast number of mobile robots, including:
* Early research platforms (e.g., Pygmalion, Khepera, Pioneer)
* Educational robots (e.g., TurtleBot)
* Commercial domestic robots (e.g., iRobot Roomba)

Despite its limitations, its simplicity and maneuverability make it a popular choice for indoor mobile robots.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #kinematics OR #mobile-robot WHERE contains(file.outlinks, [[Differential_Drive]])
