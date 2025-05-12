---
title: Hamilton-Jacobi-Bellman Equation
description: The Hamilton-Jacobi-Bellman (HJB) Equation is a partial differential equation central to optimal control theory, used to determine the optimal value function and derive optimal control policies.
tags:
  - control
  - robotics
  - optimization
  - optimal-control
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
permalink: /hamilton_jacobi_bellman_equation/
related:
  - "[[Control_Theory]]"
  - "[[Optimal_Control]]"
  - "[[Pontryagin's_Minimum_Principle]]"
  - "[[Dynamic_Programming]]"
  - "[[State_Space_Representation]]"
  - "[[Dynamics]]"
---

# Hamilton-Jacobi-Bellman Equation

The **Hamilton-Jacobi-Bellman (HJB) Equation** is a partial differential equation central to optimal control theory, used to determine the optimal value function and derive optimal control policies. It provides a framework for solving optimization problems involving dynamic systems, where the goal is to minimize a cost function over time. The HJB equation is essential for designing control strategies that optimize performance in robotic systems.

---

## Key Concepts

### Value Function

The value function $V(\mathbf{x}, t)$ represents the optimal cost-to-go from a given state $\mathbf{x}$ at time $t$. It encapsulates the minimum cost that can be achieved from that state onward, following an optimal control policy.

### Hamiltonian

The Hamiltonian function $H(\mathbf{x}, \mathbf{u}, \mathbf{\lambda}, t)$ is used in the HJB equation to relate the value function to the system dynamics and cost function. It is defined as:

$$
H(\mathbf{x}, \mathbf{u}, \mathbf{\lambda}, t) = J(\mathbf{x}, \mathbf{u}, t) + \mathbf{\lambda}^T f(\mathbf{x}, \mathbf{u}, t)
$$

where:
- $J(\mathbf{x}, \mathbf{u}, t)$ is the cost function.
- $\mathbf{x}$ is the state vector.
- $\mathbf{u}$ is the control input vector.
- $f(\mathbf{x}, \mathbf{u}, t)$ represents the system dynamics.
- $\mathbf{\lambda}$ is the costate vector.

---

## Mathematical Formulation

### HJB Equation

The HJB equation is given by:

$$
-\frac{\partial V}{\partial t} = \min_{\mathbf{u}} \left[ J(\mathbf{x}, \mathbf{u}, t) + \nabla V \cdot f(\mathbf{x}, \mathbf{u}, t) \right]
$$

where:
- $V(\mathbf{x}, t)$ is the value function.
- $\nabla V$ is the gradient of the value function with respect to the state vector $\mathbf{x}$.
- The minimization is performed over all admissible control inputs $\mathbf{u}$.

### Optimal Control Policy

The optimal control policy $\mathbf{u}^*(\mathbf{x}, t)$ is derived by minimizing the Hamiltonian:

$$
\mathbf{u}^*(\mathbf{x}, t) = \arg\min_{\mathbf{u}} H(\mathbf{x}, \mathbf{u}, \nabla V, t)
$$

This policy provides the optimal control input at each state and time, ensuring that the system follows the optimal trajectory.

---

## Applications in Robotics

- **Trajectory Planning**: The HJB equation is used to plan optimal trajectories for robotic systems, minimizing cost functions such as energy consumption or execution time.
- **Real-Time Control**: Enables the design of real-time control strategies that adapt to changing conditions and optimize performance.
- **Autonomous Systems**: Optimizes the control of autonomous vehicles and robots in dynamic environments.
- **Energy Management**: Minimizes energy consumption in robotic systems, extending their operational duration.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #control OR #robotics WHERE contains(file.outlinks, [[Hamilton-Jacobi-Bellman_Equation]])
