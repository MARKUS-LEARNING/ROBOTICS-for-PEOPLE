---
title: Friction
description: Friction is a force that resists the relative motion or tendency of such motion of two surfaces in contact. It plays a crucial role in robotics, affecting movement, control, and efficiency.
tags:
  - physics
  - mechanics
  - robotics
  - dynamics
  - control
  - tribology
layout: default
category: robotics
author: Jordan_Smith_&_le_Chat
date: 2025-04-29
permalink: /friction/
related:
  - "[[Kinematics_and_Dynamics]]"
  - "[[Control_Systems]]"
  - "[[Actuator]]"
  - "[[Mechanisms_and_Actuation]]"
  - "[[Robot_Design]]"
  - "[[Transmission_Mechanisms]]"
  - "[[Gears]]"
  - "[[Backlash]]"
  - "[[Stiffness]]"
  - "[[Compliance]]"
  - "[[Power_Density]]"
  - "[[Manipulator_Arm]]"
  - "[[Legged_Robots]]"
  - "[[Wheeled_Mobile_Robots]]"
  - "[[Mechatronics]]"
---

# Friction

**Friction** is a force that resists the relative motion or tendency of such motion of two surfaces in contact. It is a critical factor in robotics, influencing the design and performance of robotic systems, particularly in areas like motion control, actuation, and efficiency. Understanding and managing friction is essential for optimizing robotic movement and interaction with the environment.

---

## Types of Friction

1. **Static Friction**: The friction that prevents two surfaces from moving relative to each other. It must be overcome to initiate motion.
   - Mathematically represented as:
     $F_s \leq \mu_s F_n$
     where $F_s$ is the static friction force, $\mu_s$ is the coefficient of static friction, and $F_n$ is the normal force.

2. **Kinetic Friction**: The friction that acts between moving surfaces.
   - Mathematically represented as:
     $F_k = \mu_k F_n$
     where $F_k$ is the kinetic friction force, $\mu_k$ is the coefficient of kinetic friction, and $F_n$ is the normal force.

3. **Rolling Friction**: The friction that acts when one surface rolls over another, such as a wheel on a surface.
   - Often represented in terms of a rolling resistance coefficient.

4. **Fluid Friction**: The friction that occurs between layers of a fluid or between a fluid and a surface.
   - Relevant in hydraulic systems and underwater robots.

---

## Key Equations

- **Coulomb's Law of Friction**:

  $$F_f = \mu F_n$$
  
  where $F_f$ is the frictional force, $\mu$ is the coefficient of friction, and $F_n$ is the normal force.
<br>
- **Stribeck Curve**: Describes the relationship between the coefficient of friction and the sliding velocity, particularly in lubricated systems.

---

## Impact on Robotics

- **Actuation and Control**: Friction affects the efficiency and precision of actuators and control systems. It can introduce errors in positioning and require additional power to overcome.
- **Transmission Mechanisms**: In systems like gears and belts, friction can lead to energy losses and wear, impacting the overall performance and lifespan of the components.
- **Locomotion**: In [[Wheeled_Mobile_Robots]] and [[Legged_Robots]], friction influences traction and the ability to move across different surfaces.
- **Manipulation**: In [[Manipulator_Arm]] systems, friction in joints can affect the accuracy and repeatability of movements.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #component OR #kinematics WHERE contains(file.outlinks, [[Friction]])




