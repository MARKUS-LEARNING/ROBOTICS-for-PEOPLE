---
title: Computed Torque Control
description: Computed Torque Control is a control strategy used in robotics to achieve precise and dynamic control of robotic manipulators by compensating for nonlinear dynamics and ensuring accurate tracking of desired trajectories.
tags:
  - robotics
  - control-systems
  - computed-torque-control
  - dynamics
  - engineering
  - glossary-term
  - manipulator-arm
  - mobile-robot
layout: default
category: robotics
author: Jordan_Smith_and_le_Chat
date: 2025-05-02
permalink: /computed_torque_control/
related:
  - "[[Control_Theory]]"
  - "[[Dynamics]]"
  - "[[PID_Control]]"
  - "[[Robot_Design]]"
  - "[[Trajectory_Planning]]"
---

# Computed Torque Control

**Computed Torque Control** is a control strategy used in robotics to achieve precise and dynamic control of robotic manipulators. It involves compensating for the nonlinear dynamics of the robotic system to ensure accurate tracking of desired trajectories. Computed torque control is based on the principle of feedback linearization, where the nonlinear dynamics of the system are canceled out, allowing for the design of linear control strategies that achieve precise control.

---

## Key Concepts

### Feedback Linearization

Feedback linearization is a technique used to transform a nonlinear system into an equivalent linear system through feedback control. This allows for the application of linear control techniques to achieve precise and stable control.

### Robot Dynamics

Robot dynamics involve the study of the forces and torques acting on a robotic system and how they affect its motion. Computed torque control uses the dynamic model of the robot to compute the required control inputs.

### Trajectory Planning

Trajectory planning involves determining the desired path and motion of a robotic system over time. Computed torque control ensures that the robot follows the planned trajectory accurately by compensating for the system's dynamics.

### Control Inputs

Control inputs are the signals sent to the actuators of a robotic system to achieve the desired motion. Computed torque control computes these inputs based on the dynamic model and the desired trajectory.

---

## Mathematical Formulation

### Dynamic Model

The dynamic model of a robotic manipulator can be represented by the equation:

$$
M(\mathbf{q}) \ddot{\mathbf{q}} + C(\mathbf{q}, \dot{\mathbf{q}}) \dot{\mathbf{q}} + G(\mathbf{q}) = \tau
$$

where:
- $M(\mathbf{q})$ is the inertia matrix.
- $C(\mathbf{q}, \dot{\mathbf{q}})$ is the Coriolis and centrifugal forces matrix.
- $G(\mathbf{q})$ is the gravitational forces vector.
- $\tau$ is the vector of applied torques.

### Computed Torque Control Law

The computed torque control law is given by:

$$
\tau = M(\mathbf{q}) \mathbf{a} + C(\mathbf{q}, \dot{\mathbf{q}}) \dot{\mathbf{q}} + G(\mathbf{q})
$$

where $\mathbf{a}$ is the desired acceleration, computed as:

$$
\mathbf{a} = \ddot{\mathbf{q}}_d + K_d (\dot{\mathbf{q}}_d - \dot{\mathbf{q}}) + K_p (\mathbf{q}_d - \mathbf{q})
$$

where:
- $\mathbf{q}_d$ is the desired position.
- $\dot{\mathbf{q}}_d$ is the desired velocity.
- $\ddot{\mathbf{q}}_d$ is the desired acceleration.
- $K_p$ and $K_d$ are the proportional and derivative gain matrices, respectively.

### Example: Robotic Arm

Consider a robotic arm with multiple joints. The computed torque control strategy is used to ensure that the arm follows a desired trajectory accurately. The control law computes the required torques based on the dynamic model of the arm and the desired trajectory. This allows the arm to perform tasks such as grasping and manipulating objects with high precision and stability.

---

## Applications in Robotics

- **Manipulation**: Computed torque control is used to achieve precise and dynamic control of robotic manipulators, enabling tasks such as assembly, welding, and material handling.
- **Autonomous Systems**: Enhances the control of autonomous robots, allowing them to navigate and interact with their environment accurately.
- **Trajectory Tracking**: Ensures that robotic systems follow planned trajectories accurately, improving the performance and reliability of tasks.
- **Adaptive Control**: Computed torque control can be combined with adaptive control techniques to handle uncertainties and variations in the system's dynamics.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #robotics OR #control-systems WHERE contains(file.outlinks, [[Computed_Torque_Control]])
