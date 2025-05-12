---
title: Control of Underactuated Systems
description: The control of underactuated systems involves managing systems with fewer control inputs than degrees of freedom, requiring specialized strategies to achieve desired performance and stability.
tags:
  - control
  - robotics
  - underactuated-systems
  - control-theory
  - dynamics
  - engineering
  - glossary-term
  - mechanism
  - manipulator-arm
  - mobile-robot
layout: default
category: robotics
author: Jordan_Smith_and_le_Chat
date: 2025-05-02
permalink: /control_of_underactuated_systems/
related:
  - "[[Control_Theory]]"
  - "[[Dynamics]]"
  - "[[Nonlinear_Control_Theory]]"
  - "[[Energy-Based_Control]]"
  - "[[Passivity-Based_Control]]"
  - "[[Robot_Design]]"
  - "[[Acrobot]]"
  - "[[Pendubot]]"
---

# Control of Underactuated Systems

The **control of underactuated systems** involves managing systems with fewer control inputs than degrees of freedom. These systems are challenging to control because they lack direct actuation for all their degrees of freedom, requiring specialized strategies to achieve desired performance and stability. Underactuated systems are common in robotics, particularly in applications where weight, complexity, or cost constraints limit the number of actuators.

---

## Key Concepts

### Underactuation

Underactuation refers to the condition where a system has fewer control inputs than degrees of freedom. This results in the system being unable to directly control all its states, necessitating indirect control methods to manage its dynamics effectively.

### Energy-Based Control

Energy-based control methods are often used for underactuated systems, leveraging the system's natural dynamics and energy properties to achieve control objectives. These methods focus on managing the system's energy to stabilize and regulate its behavior.

### Passivity-Based Control

Passivity-based control is another approach used for underactuated systems, which involves designing control laws that ensure the system remains passive. This property helps in stabilizing the system and ensuring robust performance, even in the presence of uncertainties and disturbances.

---

## Mathematical Formulation

### System Dynamics

The dynamics of an underactuated system can be represented as:

$$
M(\mathbf{q})\ddot{\mathbf{q}} + C(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + G(\mathbf{q}) = B(\mathbf{q}) \mathbf{u}
$$

where:
- $\mathbf{q}$ is the vector of generalized coordinates.
- $M(\mathbf{q})$ is the inertia matrix.
- $C(\mathbf{q}, \dot{\mathbf{q}})$ is the Coriolis and centrifugal forces matrix.
- $G(\mathbf{q})$ is the gravitational forces vector.
- $B(\mathbf{q})$ is the input matrix.
- $\mathbf{u}$ is the vector of control inputs.

### Energy-Based Control Law

Energy-based control laws often involve designing a control input that modifies the system's energy to achieve desired behavior. For example, the control law might be designed to inject or dissipate energy to stabilize the system:

$$
\mathbf{u} = -K_d \dot{\mathbf{q}} - K_p (\mathbf{q} - \mathbf{q}_d)
$$

where $K_d$ and $K_p$ are gain matrices, and $\mathbf{q}_d$ is the desired configuration.

### Passivity-Based Control Law

Passivity-based control laws ensure that the system remains passive, which can be achieved by designing the control input to satisfy passivity conditions:

$$
\mathbf{u} = -K_d \dot{\mathbf{q}} - K_p (\mathbf{q} - \mathbf{q}_d) + \mathbf{u}_{passive}
$$

where $\mathbf{u}_{passive}$ is an additional control term that ensures the system remains passive.

---

## Applications in Robotics

- **Acrobot and Pendubot**: Underactuated systems like the acrobot and pendubot are used to study and demonstrate control strategies for underactuated systems. These systems typically have two links but only one actuator, requiring clever control strategies to achieve desired motions.
- **Mobile Robots**: Underactuated mobile robots, such as those with non-holonomic constraints, require specialized control strategies to navigate and perform tasks effectively.
- **Aerospace**: Control of underactuated systems is crucial in aerospace applications, where weight and complexity constraints often limit the number of actuators.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #control OR #robotics WHERE contains(file.outlinks, [[Control_of_Underactuated_Systems]])
