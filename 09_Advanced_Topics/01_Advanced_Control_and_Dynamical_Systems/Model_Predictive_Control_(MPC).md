---
title: Model Predictive Control (MPC)
description: Model Predictive Control (MPC) is an advanced control strategy that uses a model of the system to predict future behavior and optimize control actions over a finite horizon.
tags:
  - control
  - robotics
  - optimization
  - mpc
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
permalink: /model_predictive_control/
related:
  - "[[Control_Theory]]"
  - "[[Optimal_Control]]"
  - "[[State_Space_Representation]]"
  - "[[Dynamic_Programming]]"
  - "[[Convex_Optimization]]"
  - "[[Dynamics]]"
  - "[[Receding_Horizon_Control]]"
---

# Model Predictive Control (MPC)

**Model Predictive Control (MPC)** is an advanced control strategy that uses a model of the system to predict future behavior and optimize control actions over a finite horizon. It is particularly useful for handling complex systems with constraints, nonlinearities, and time-varying dynamics. MPC is widely used in robotics for tasks that require precise control and adaptation to changing conditions.

---

## Key Concepts

### Predictive Model

MPC relies on a dynamic model of the system to predict its future behavior based on current states and potential control actions. This model is used to simulate the system's response over a prediction horizon.

### Optimization

At each time step, MPC solves an optimization problem to determine the optimal sequence of control actions that minimize a cost function, subject to system dynamics and constraints. The optimization is typically performed over a finite horizon, known as the prediction horizon.

### Receding Horizon

MPC employs a receding horizon approach, where the optimization is repeated at each time step with updated information. Only the first control action from the optimized sequence is applied, and the process is repeated at the next time step.

---

## Mathematical Formulation

### Prediction Model

The prediction model is typically represented in state-space form:

$$
\mathbf{x}_{k+1} = f(\mathbf{x}_k, \mathbf{u}_k)
$$

where:
- $\mathbf{x}_k$ is the state vector at time step $k$.
- $\mathbf{u}_k$ is the control input vector at time step $k$.
- $f(\mathbf{x}_k, \mathbf{u}_k)$ represents the system dynamics.

### Cost Function

The cost function to be minimized is usually a quadratic function of the states and control inputs over the prediction horizon:

$$
J = \sum_{i=0}^{N-1} (\mathbf{x}_{k+i}^T Q \mathbf{x}_{k+i} + \mathbf{u}_{k+i}^T R \mathbf{u}_{k+i}) + \mathbf{x}_{k+N}^T Q_N \mathbf{x}_{k+N}
$$

where:
- $N$ is the prediction horizon.
- $Q$ and $R$ are weighting matrices for the states and control inputs, respectively.
- $Q_N$ is the terminal cost weighting matrix.

### Optimization Problem

The optimization problem is formulated as:

$$
\min_{\mathbf{u}_k, \mathbf{u}_{k+1}, \ldots, \mathbf{u}_{k+N-1}} J
$$

subject to:

$$
\mathbf{x}_{k+i+1} = f(\mathbf{x}_{k+i}, \mathbf{u}_{k+i}), \quad i = 0, 1, \ldots, N-1
$$

and any additional constraints on the states and control inputs.

---

## Applications in Robotics

- **Robotic Manipulators**: MPC is used to control robotic manipulators with complex dynamics and constraints, ensuring precise and efficient motion control.
- **Autonomous Vehicles**: Ensures stable and efficient control of autonomous vehicles in dynamic and uncertain environments.
- **Aerospace**: Designing control systems for aircraft and spacecraft that require precise stabilization and maneuvering.
- **Industrial Automation**: Implementing control strategies for manufacturing processes that involve complex dynamics and constraints.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #control OR #robotics WHERE contains(file.outlinks, [[Model_Predictive_Control_(MPC)]])
