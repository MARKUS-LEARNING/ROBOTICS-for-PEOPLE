---
title: State Space Representation
description: "State-space models describe a system by a state vector and first-order equations, ẋ = Ax + Bu, y = Cx + Du. Covers choosing states, a worked mass-spring-damper example (eigenvalues, damping, controllability, computed), observability and observers, linearising robot dynamics, and why state space is the language of modern control, Kalman filtering, MPC and learned world models."
aliases:
  - State Space
  - State-Space Model
  - State Space Model
tags:
  - control
  - state-estimation
  - mathematics
type: concept
level: intermediate
subfield:
  - control
  - mathematics
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Control_Theory]]"
  - "[[Linear_Algebra_for_Robotics]]"
related:
  - "[[Transfer_Function]]"
  - "[[Model_Predictive_Control_(MPC)]]"
  - "[[Probabilistic_Robotics]]"
  - "[[World_Models]]"
key_takeaways:
  - "A state is the smallest set of numbers that, with future inputs, determines the future: for a mass on a spring, position and velocity."
  - "The eigenvalues of A are the system's poles: for m = 1, k = 4, b = 1.2 they are −0.6 ± 1.91j, i.e. natural frequency 2 rad/s and damping ratio 0.3."
  - "Controllability says whether inputs can steer every state; observability whether outputs reveal every state; together they let observers (Kalman filters) and state feedback (LQR, MPC) work."
sources:
  - "Åström, K. J. and Murray, R. M. (2021). Feedback Systems, 2nd edition, chapters 6–8 (state space, reachability, observability). https://fbswiki.org/wiki/index.php/Feedback_Systems:_An_Introduction_for_Scientists_and_Engineers"
  - "Lynch, K. M. and Park, F. C. (2017). Modern Robotics, chapter 8: Dynamics of Open Chains. https://hades.mech.northwestern.edu/index.php/Modern_Robotics"
  - "Tedrake, R. (2026). Underactuated Robotics. https://underactuated.csail.mit.edu/"
---

# State Space Representation

A **state-space model** describes a dynamic system with a **state vector** $\mathbf x$, a list of numbers that captures everything about the system's present that matters for its future, and first-order differential equations for how it evolves:

$$\dot{\mathbf x} = A\mathbf x + B\mathbf u, \qquad \mathbf y = C\mathbf x + D\mathbf u$$

Here $\mathbf u$ is the input (motor torques) and $\mathbf y$ the measured output (encoder readings). Nonlinear systems use $\dot{\mathbf x} = f(\mathbf x, \mathbf u)$.

> **Etymology.** *State* is from Latin *status*, "condition". Rudolf Kálmán's work around 1960 made the state-space approach the basis of "modern" control, alongside his filter.

## Choosing the state

For a mass $m$ on a spring $k$ with damper $b$, driven by force $u$, the state is position and velocity, $\mathbf x = (x, \dot x)$:

$$\dot{\mathbf x} = \underbrace{\begin{bmatrix}0 & 1\\ -k/m & -b/m\end{bmatrix}}_{A}\mathbf x + \underbrace{\begin{bmatrix}0\\ 1/m\end{bmatrix}}_{B} u, \qquad y = \begin{bmatrix}1 & 0\end{bmatrix}\mathbf x$$

For a robot arm with $n$ joints, the state is the $2n$ joint angles and velocities. For a drone, it is position, velocity, orientation and angular velocity.

## Worked example

Take $m = 1$ kg, $k = 4$ N/m, $b = 1.2$ N·s/m:

$$A = \begin{bmatrix}0 & 1\\ -4 & -1.2\end{bmatrix}$$

- **Eigenvalues** (the poles): $\lambda = -0.6 \pm 1.908j$. The negative real part means the system is stable; the imaginary part means it oscillates.
- **Natural frequency and damping**: from $\lambda^2 + 1.2\lambda + 4 = 0$, $\omega_n = \sqrt4 = 2$ rad/s and $\zeta = 1.2/(2 \times 2) = 0.3$. A step response overshoots by about 37% (see [[Control_Theory]]).
- **Controllability**: the matrix $[B\ \ AB] = \begin{bmatrix}0 & 1\\ 1 & -1.2\end{bmatrix}$ has rank 2, so the force can steer both position and velocity anywhere.

## Controllability, observability and observers

- **Controllable**: the inputs can drive the state anywhere. Test: $[B\ AB\ \dots\ A^{n-1}B]$ has full rank.
- **Observable**: the outputs reveal the whole state over time. Test: $[C;\ CA;\ \dots;\ CA^{n-1}]$ has full rank. Measuring only position still lets you infer velocity.
- **Observer**: an algorithm that estimates the state from outputs. The **Kalman filter** is the optimal observer under Gaussian noise (see [[Probabilistic_Robotics]]).
- **State feedback**: $\mathbf u = -K\mathbf x$ places the poles where you choose; **LQR** picks $K$ optimally.

## Linearising robots

Robots are nonlinear, but near an operating point $(\mathbf x_0, \mathbf u_0)$:

$$A = \frac{\partial f}{\partial \mathbf x}\Big|_0, \qquad B = \frac{\partial f}{\partial \mathbf u}\Big|_0$$

LQR around an upright pose balances an inverted pendulum or a hovering drone; MPC relinearises at every step along a trajectory.

## State space in 2026 robotics

- **MPC and whole-body control** solve optimisation problems over state trajectories (see [[Model_Predictive_Control_(MPC)]]).
- **State estimation** (Kalman filters, factor graphs) keeps track of $\mathbf x$ from noisy sensors.
- **Reinforcement learning** calls the state the "observation" when it is incomplete, and the MDP framework is a state-space model with rewards (see [[Markov_Decision_Process]]).
- **Learned world models** learn a latent state and its dynamics from video, a learned state-space model (see [[World_Models]]); state-space sequence models (S4, Mamba) bring the same idea into neural architectures.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[State_Space_Representation]])
```
