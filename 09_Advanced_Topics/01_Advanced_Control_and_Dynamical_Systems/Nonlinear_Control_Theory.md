---
title: Nonlinear Control Theory
description: "Tools for analysing and controlling nonlinear systems such as robots: Lyapunov stability (with an energy-based pendulum example), LaSalle's principle, feedback linearisation, backstepping, sliding modes, passivity, and control Lyapunov and barrier functions, which in 2026 are the main way to give learned controllers stability and safety guarantees."
aliases:
  - Nonlinear Control
  - Lyapunov Stability
  - Lyapunov Theory
tags:
  - control
  - stability
  - mathematics
type: concept
level: research
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
  - "[[State_Space_Representation]]"
related:
  - "[[Feedback_Linearization]]"
  - "[[Robust_Control_Methods]]"
  - "[[Adaptive_Control_Theory]]"
  - "[[Collision_Avoidance]]"
key_takeaways:
  - "Robots are nonlinear (gravity terms with sin q, posture-dependent inertia, contact), so linear tools only hold locally; nonlinear theory gives guarantees over large regions."
  - "Lyapunov's direct method proves stability without solving the equations: find an energy-like V > 0 that never increases; for a damped pendulum, total energy works and V̇ = −b θ̇²."
  - "Control Lyapunov and barrier functions turn stability and safety into constraints a quadratic program can enforce in real time, around any nominal controller, including learned ones."
sources:
  - "Khalil, H. K. (2002). Nonlinear Systems, 3rd edition. https://www.pearson.com/en-us/subject-catalog/p/nonlinear-systems/P200000003364"
  - "Murray, R. M., Li, Z. and Sastry, S. S. (1994). A Mathematical Introduction to Robotic Manipulation. http://www.cds.caltech.edu/~murray/books/MLS/pdf/mls94-complete.pdf"
  - "Tedrake, R. (2026). Underactuated Robotics, chapter: Lyapunov Analysis. https://underactuated.csail.mit.edu/lyapunov.html"
  - "Ames, A. D. et al. (2019). Control Barrier Functions: Theory and Applications. https://arxiv.org/abs/1903.11199"
---

# Nonlinear Control Theory

**Nonlinear control theory** analyses and designs controllers for systems whose dynamics are not linear: almost every real robot. Gravity torques vary with $\sin q$, inertia changes with posture, friction and contact switch abruptly. Linearising works near one operating point; nonlinear tools give guarantees across large regions of the state space.

> **Etymology.** Aleksandr Lyapunov's 1892 doctoral thesis, *The General Problem of the Stability of Motion*, founded the field. Its methods were largely unknown in the West until the 1960s.

## Lyapunov stability

An equilibrium is **stable** if starting near it keeps you near it, and **asymptotically stable** if you also return to it. Lyapunov's **direct method** proves this without solving the dynamics: find a function $V(\mathbf x)$ that is positive except at the equilibrium (where it is 0) and never increases along trajectories, $\dot V \le 0$. If $\dot V < 0$ away from the equilibrium, the system is asymptotically stable.

### Worked example: a damped pendulum

$ml^2\ddot\theta + b\dot\theta + mgl\sin\theta = 0$. Take total energy as $V$:

$$V = \tfrac12 ml^2\dot\theta^2 + mgl(1 - \cos\theta) \ \ge 0.$$

Differentiating along the dynamics,

$$\dot V = ml^2\dot\theta\ddot\theta + mgl\sin\theta\,\dot\theta = \dot\theta(-b\dot\theta - mgl\sin\theta) + mgl\sin\theta\,\dot\theta = -b\dot\theta^2 \le 0.$$

Energy never increases, so the pendulum is stable. $\dot V$ is zero whenever $\dot\theta = 0$ (not only at the bottom), so the direct method alone gives only stability. **LaSalle's invariance principle** completes the argument: the only trajectory that can stay where $\dot\theta = 0$ is the equilibrium itself, so the pendulum converges to hanging straight down.

## Design methods

| Method | Idea | Robot use |
|---|---|---|
| **[[Feedback_Linearization]]** | Cancel the nonlinearity with the control input | Computed-torque control of arms |
| **Backstepping** | Build a controller and Lyapunov function step by step through a chain of integrators | Drones, underactuated systems |
| **Sliding mode** | Drive the state onto a surface where dynamics are simple, and stay there despite bounded uncertainty | Robust tracking; see [[Robust_Control_Methods]] |
| **Passivity-based control** | Shape the system's energy; passive systems interconnect stably | Interaction and teleoperation with force feedback |
| **Control Lyapunov functions (CLF)** | Choose inputs that make $\dot V$ sufficiently negative, by quadratic program | Bipedal walking controllers |
| **Control barrier functions (CBF)** | Choose inputs that keep a safe set invariant | Safety filters (see [[Collision_Avoidance]]) |

## Nonlinear theory and learned control (2026)

Neural-network policies are nonlinear controllers whose stability is not obvious. Active research directions:
- **learned Lyapunov or barrier certificates**: train a second network to be a Lyapunov or barrier function and verify its conditions, by sampling, by formal verification for small networks, or in a defined region;
- **CBF safety filters**: a QP adjusts a learned policy's command minimally so the barrier condition holds;
- **stable-by-construction architectures** for learned dynamics.

These give learned robots guarantees in the same mathematical language as classical control.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Nonlinear_Control_Theory]])
```
