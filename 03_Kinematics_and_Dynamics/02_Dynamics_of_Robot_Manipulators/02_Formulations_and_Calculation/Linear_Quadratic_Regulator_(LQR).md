---
title: Linear Quadratic Regulator (LQR)
description: The Linear Quadratic Regulator (LQR) is a control strategy used to minimize a quadratic cost function for linear systems, providing optimal feedback control for stabilization and regulation.
tags:
  - control
  - robotics
  - optimization
  - lqr
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
permalink: /linear_quadratic_regulator/
related:
  - "[[Control_Theory]]"
  - "[[Optimal_Control]]"
  - "[[State_Space_Representation]]"
  - "[[Pontryagin's_Minimum_Principle]]"
  - "[[Hamilton-Jacobi-Bellman_Equation]]"
  - "[[Dynamics]]"
  - "[[Riccati_Equation]]"
---

# Linear Quadratic Regulator (LQR)

The **Linear Quadratic Regulator (LQR)** is a control strategy used to minimize a quadratic cost function for linear systems, providing optimal feedback control for stabilization and regulation. It is widely used in robotics for tasks that require precise and efficient control, such as stabilizing robotic manipulators and controlling autonomous systems. The LQR provides a systematic approach to designing optimal controllers for linear systems with quadratic performance criteria.

---

## Key Concepts

### Linear System Representation

The LQR is applied to linear systems represented in state-space form:

$$
\dot{\mathbf{x}}(t) = A \mathbf{x}(t) + B \mathbf{u}(t)
$$

where:
- $\mathbf{x}(t)$ is the state vector.
- $\mathbf{u}(t)$ is the control input vector.
- $A$ and $B$ are the system matrices.

### Quadratic Cost Function

The cost function to be minimized is given by:

$$
J = \int_0^\infty (\mathbf{x}^T(t) Q \mathbf{x}(t) + \mathbf{u}^T(t) R \mathbf{u}(t)) \, dt
$$

where:
- $Q$ is a positive semi-definite matrix weighing the state variables.
- $R$ is a positive definite matrix weighing the control inputs.

---

## Mathematical Formulation

### Algebraic Riccati Equation

The optimal control law for the LQR is derived by solving the Algebraic Riccati Equation (ARE):

$$
A^T P + PA - PBR^{-1}B^T P + Q = 0
$$

where $P$ is the solution to the ARE, which is a symmetric positive definite matrix.

### Optimal Control Law

The optimal control input is given by:

$$
\mathbf{u}(t) = -K \mathbf{x}(t)
$$

where $K$ is the optimal gain matrix, calculated as:

$$
K = R^{-1} B^T P
$$

This control law ensures that the system is stabilized and the cost function is minimized.

---

## Applications in Robotics

- **Robotic Manipulators**: LQR is used to stabilize and control robotic manipulators, ensuring precise and efficient motion control.
- **Autonomous Vehicles**: Ensures stable and efficient control of autonomous vehicles in dynamic environments.
- **Aerospace**: Designing control systems for aircraft and spacecraft that require precise stabilization and maneuvering.
- **Industrial Automation**: Implementing control strategies for manufacturing processes that involve linear dynamics and require optimal regulation.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #control OR #robotics WHERE contains(file.outlinks, [[Linear_Quadratic_Regulator_(LQR)]])
