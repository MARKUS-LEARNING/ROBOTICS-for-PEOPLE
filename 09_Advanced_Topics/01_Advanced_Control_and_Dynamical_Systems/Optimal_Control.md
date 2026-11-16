---
title: Optimal Control
description: "Optimal control chooses control inputs that minimise a cost over time subject to the system's dynamics and limits. Covers the problem statement, the two classical routes (dynamic programming / HJB and Pontryagin's principle), LQR with computed gains for a double integrator, trajectory optimisation, MPC as online optimal control, and reinforcement learning as optimal control without a known model."
aliases:
  - Optimal Control Theory
  - LQR
  - Linear Quadratic Regulator
  - Trajectory Optimization
tags:
  - optimal-control
  - control
  - optimization
type: concept
level: advanced
subfield:
  - control
  - planning
status: keystone
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[State_Space_Representation]]"
  - "[[Optimization_and_Loss_Functions]]"
related:
  - "[[Hamilton-Jacobi-Bellman_Equation]]"
  - "[[Pontryagin's_Minimum_Principle]]"
  - "[[Model_Predictive_Control_(MPC)]]"
  - "[[Reinforcement_Learning]]"
  - "[[Markov_Decision_Process]]"
key_takeaways:
  - "Optimal control asks for the input history that minimises a cost (tracking error, effort, time, energy) while obeying the dynamics and constraints."
  - "LQR solves the linear-quadratic case exactly: for a double integrator with Q = diag(1, 0), R = 1 the optimal gain is K = [1, 1.414], giving closed-loop poles at −0.707 ± 0.707j (damping 0.707)."
  - "MPC is optimal control re-solved online; reinforcement learning is optimal control learned from samples when the model is unknown; all share Bellman's principle."
sources:
  - "Liberzon, D. (2011). Calculus of Variations and Optimal Control Theory: A Concise Introduction. http://liberzon.csl.illinois.edu/teaching/cvoc.pdf"
  - "Bertsekas, D. P. (2017). Dynamic Programming and Optimal Control, 4th edition. http://www.athenasc.com/dpbook.html"
  - "Tedrake, R. (2026). Underactuated Robotics, chapter: Linear Quadratic Regulators. https://underactuated.csail.mit.edu/lqr.html"
  - "Rawlings, J. B., Mayne, D. Q. and Diehl, M. M. (2020). Model Predictive Control: Theory, Computation, and Design, 2nd edition. https://sites.engineering.ucsb.edu/~jbraw/mpc/"
---

# Optimal Control

**Optimal control** finds the best way to steer a dynamic system: the input history that minimises a chosen cost, such as time, energy, tracking error or effort, subject to the system's dynamics and limits. It underlies trajectory planning for arms and drones, MPC for legged robots, and, through Bellman's principle, reinforcement learning.

> **Etymology.** *Optimal* from Latin *optimus*, "best". The field grew from the calculus of variations (Euler, Lagrange, 18th century). In the 1950s Richard Bellman (dynamic programming, 1957) and Lev Pontryagin's group (maximum principle, 1956–62) gave it its two main methods, driven by rocket and missile guidance.

## The problem

$$\min_{\mathbf u(\cdot)}\ \ \phi(\mathbf x(T)) + \int_0^T \ell(\mathbf x(t), \mathbf u(t))\, dt \quad \text{s.t.} \quad \dot{\mathbf x} = f(\mathbf x, \mathbf u),\ \ \mathbf u \in \mathcal U,\ \ \mathbf x \in \mathcal X$$

The terminal cost $\phi$ and the running cost $\ell$ express what "best" means. Change them and the optimal behaviour changes: minimum time gives aggressive bang-bang control, minimum effort gives gentle motion.

## Two classical routes

| Route | Idea | Gives | See |
|---|---|---|---|
| **Dynamic programming** | The optimal cost-to-go $V(\mathbf x, t)$ satisfies a recursion (Bellman) or PDE (HJB) | A feedback **policy** for every state | [[Hamilton-Jacobi-Bellman_Equation]] |
| **Pontryagin's principle** | Necessary conditions along one optimal trajectory, using costates | An open-loop **trajectory** from one start | [[Pontryagin's_Minimum_Principle]] |

Dynamic programming suffers the "curse of dimensionality" (the value function lives on the whole state space). Trajectory methods scale to many dimensions but give one trajectory at a time.

## LQR: the case solved exactly

For linear dynamics $\dot{\mathbf x} = A\mathbf x + B\mathbf u$ and quadratic cost $\int (\mathbf x^\top Q \mathbf x + \mathbf u^\top R \mathbf u)\, dt$, the optimal policy is linear state feedback $\mathbf u = -K\mathbf x$ with $K = R^{-1}B^\top P$, where $P$ solves the algebraic Riccati equation.

**Worked example.** A double integrator (position $x$, velocity $v$, acceleration input):

| Weights | Optimal gain $K$ | Closed-loop poles |
|---|---|---|
| $Q = \mathrm{diag}(1, 0)$, $R = 1$ | [1.000, 1.414] | $-0.707 \pm 0.707j$ |
| $Q = \mathrm{diag}(100, 0)$, $R = 1$ | [10.0, 4.472] | $-2.236 \pm 2.236j$ |

Raising the position weight 100-fold makes the controller 10× stiffer and about 3× faster, but always at damping ratio 0.707: LQR finds a well-damped compromise automatically. LQR stabilises balancing robots, drones about hover, and the linearised dynamics inside trajectory-tracking controllers.

## Trajectory optimisation

For nonlinear robots, **direct methods** turn the problem into a finite-dimensional optimisation over discretised states and inputs:
- direct collocation;
- shooting;
- DDP and iLQR, which iterate LQR around a trajectory.

Solvers such as Drake, Crocoddyl, CasADi and GPU-parallel cuRobo plan motions for arms, legged robots and drones.

## Optimal control everywhere in 2026

- **[[Model_Predictive_Control_(MPC)|MPC]]**: optimal control re-solved every few milliseconds with the current state.
- **[[Reinforcement_Learning|Reinforcement learning]]**: optimal control of a [[Markov_Decision_Process]] learned from experience, with reward as negative cost. Locomotion policies trained in simulation are approximate optimal feedback policies.
- **Hybrid**: learned value functions as MPC terminal costs; policies distilled from trajectory optimisation.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Optimal_Control]])
```
