---
title: DC Motors
description: DC Motors are electrical machines that convert direct current electrical energy into mechanical motion, widely used in robotics for actuation and control.
tags:
  - component
  - actuation
  - electrical
  - robotics
  - mechatronics
  - control
  - glossary-term
  - actuator
  - motion
  - kinematics
  - dynamics
  - mechanism
  - manipulator-arm
  - mobile-robot
layout: default
category: robotics
author: Jordan_Smith_and_le_Chat
date: 2025-05-02
permalink: /dc-motors/
related:
  - "[[Electric_Motors]]"
  - "[[Actuator]]"
  - "[[Mechanisms_and_Actuation]]"
  - "[[Control_Systems]]"
  - "[[Robot_Design]]"
  - "[[Power_Density]]"
  - "[[Transmission_Mechanisms]]"
  - "[[Feedback_Control]]"
  - "[[Manipulator_Arm]]"
  - "[[Legged_Robots]]"
  - "[[Wheeled_Mobile_Robots]]"
  - "[[Human-Robot_Interaction]]"
  - "[[Mechatronics]]"
  - "[[Kinematics_and_Dynamics]]"
  - "[[Statics]]"
  - "[[Gears]]"
  - "[[Backlash]]"
  - "[[Friction]]"
  - "[[Stiffness]]"
  - "[[Compliance]]"
---

# DC Motors

**DC Motors** (Direct Current Motors) are electrical machines that convert direct current electrical energy into mechanical motion. They are widely used in robotics for actuation and control due to their simplicity, reliability, and ease of control. DC motors are essential components in various robotic systems, enabling movement, manipulation, and interaction with the environment.

---

## Types of DC Motors

1. **Permanent Magnet DC Motors**:
   - Utilize permanent magnets to generate a magnetic field.
   - Simple and reliable, commonly used in educational and hobbyist robots.
   - Key characteristics include torque constant ($k_t$) and back-EMF ($k_e \omega$).
   <br>

2. **Brushless DC Motors (BLDC)**:
   - Eliminate the need for brushes, offering higher reliability and efficiency.
   - Common in modern industrial robots and drones.
   - Require electronic commutation for operation.
   <br>

3. **Stepper Motors**:
   - Rotate in discrete steps, ideal for precise positioning in low-cost robots.
   - Often used in 3D printers and CNC machines.
   - Typically operated in an open-loop control system.
   <br>

4. **Servo Motors**:
   - Combine a motor with a position sensor and controller for precise control of position, velocity, or torque.
   - Common in robotic arms and [[Legged_Robots|Legged Robots]].
   - Utilize feedback control for accurate operation.
   <br>

---

## Key Equations

### Torque Constant ($k_t$)

The torque $\tau$ produced by the motor is given by:

$$
\tau = k_t \cdot I
$$

where $k_t$ is the torque constant, and $I$ is the current through the motor.

### Back-EMF ($k_e$)

The induced voltage $V$ (back-EMF) is given by:

$$
V = k_e \cdot \omega
$$

where $k_e$ is the back-EMF constant, and $\omega$ is the angular velocity of the motor.

### Power Output

The power output $P$ of the motor is calculated as:

$$
P = \tau \cdot \omega
$$

where $\tau$ is the torque, and $\omega$ is the angular velocity.

### Electrical Equation

The electrical behavior of a DC motor can be described by:

$$
V_s = R \cdot I + L \frac{dI}{dt} + k_e \cdot \omega
$$

where $V_s$ is the supply voltage, $R$ is the resistance, $L$ is the inductance, $I$ is the current, and $\omega$ is the angular velocity.

---

## Impact on Robotics

- **Actuation and Control**: DC motors are fundamental for actuation in robotic systems, enabling precise control of movement and interaction with the environment.
  <br>

- **Efficiency**: The efficiency of DC motors is crucial for optimizing power usage, especially in battery-powered robots.
  <br>

- **Design and Integration**: The selection and integration of DC motors are critical aspects of [[Robot_Design]] and [[Mechatronics]], influencing the overall performance and capabilities of robotic systems.
  <br>

- **Versatility**: DC motors are used in a wide range of applications, from simple educational robots to complex industrial systems, due to their versatility and ease of control.
  <br>

- **Feedback Control**: Many DC motors, especially servo motors, utilize feedback control systems to achieve precise positioning and dynamic response, enhancing the performance of robotic systems.
  <br>

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #component OR #actuation WHERE contains(file.outlinks, [[DC_Motors]])
