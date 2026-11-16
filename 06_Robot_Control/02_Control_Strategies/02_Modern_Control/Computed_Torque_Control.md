---
title: Computed Torque Control
description: "Computed torque control uses the robot's dynamic model to cancel its nonlinear dynamics, leaving simple linear error dynamics that a PD law can shape exactly. Covers the control law (feedback linearisation for manipulators), a simulated one-link example comparing PD alone, exact-model computed torque and a 20%-wrong model, gain selection, and its descendants (operational-space and whole-body control) in 2026."
aliases:
  - Computed-Torque Control
  - Inverse Dynamics Control
  - Feedback Linearization
tags:
  - control
  - dynamics
  - feedback-linearization
type: method
level: advanced
subfield:
  - control
  - kinematics-dynamics
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Robot_Control]]"
  - "[[PID_Control]]"
related:
  - "[[Adaptive_Control]]"
  - "[[Robust_Control]]"
  - "[[Model_Predictive_Control_(MPC)]]"
  - "[[Compliance]]"
key_takeaways:
  - "Computed torque applies τ = M(q)(q̈_d + K_d ė + K_p e) + C(q, q̇)q̇ + g(q): the model cancels the dynamics so every joint's error obeys ë + K_d ė + K_p e = 0."
  - "In a simulated one-link arm tracking a sine, PD alone left 0.033 rad RMS error, exact computed torque 0.00014 rad, and a model 20% too light 0.0093 rad: the method is only as good as the model."
  - "It is the foundation of inverse-dynamics, operational-space and whole-body controllers used on humanoids and torque-controlled arms; adaptive and learned models address the model error."
sources:
  - "Lynch, K. M. and Park, F. C. (2017). Modern Robotics, chapter 11.4: Motion control with torque or force inputs. https://hades.mech.northwestern.edu/index.php/Modern_Robotics"
  - "Siciliano, B. and Khatib, O., eds. (2016). Springer Handbook of Robotics, 2nd edition, chapter on motion control. https://link.springer.com/referencework/10.1007/978-3-319-32552-1"
  - "Khatib, O. (1987). A unified approach for motion and force control of robot manipulators: The operational space formulation. https://ieeexplore.ieee.org/document/1087068"
  - "Tedrake, R. (2026). Underactuated Robotics. https://underactuated.csail.mit.edu/"
---

# Computed Torque Control

**Computed torque control** is the classic model-based controller for robot arms. Instead of fighting the robot's nonlinear dynamics with high gains, it uses a model of those dynamics to **cancel** them, leaving each joint behaving like a simple, linear system that a PD controller can tune exactly.

> **Etymology.** The torque is *computed* from the dynamic model at every step. In control theory the same idea is called **feedback linearisation**: feedback is used to make a nonlinear system behave linearly. It became practical in the 1980s, as computers grew fast enough to evaluate arm dynamics in real time.

## The control law

For manipulator dynamics $M(\mathbf q)\ddot{\mathbf q} + C(\mathbf q,\dot{\mathbf q})\dot{\mathbf q} + \mathbf g(\mathbf q) = \boldsymbol\tau$ and a desired trajectory $\mathbf q_d(t)$, with error $\mathbf e = \mathbf q_d - \mathbf q$:

$$\boldsymbol\tau = M(\mathbf q)\big(\ddot{\mathbf q}_d + K_d\dot{\mathbf e} + K_p\mathbf e\big) + C(\mathbf q,\dot{\mathbf q})\dot{\mathbf q} + \mathbf g(\mathbf q)$$

If the model is exact, substituting gives

$$\ddot{\mathbf e} + K_d\dot{\mathbf e} + K_p\mathbf e = 0$$

for every joint, independently and whatever the posture. Choose $K_p = \omega_n^2$ and $K_d = 2\zeta\omega_n$. For example, $K_p = 100$, $K_d = 20$ gives $\omega_n = 10$ rad/s and $\zeta = 1$: critically damped, settling in roughly $4/(\zeta\omega_n) = 0.4$ s.

## Worked example: how much the model matters

A one-link arm (inertia 0.5 kg·m², mass 2 kg, centre of mass 0.4 m from the joint) tracks $q_d = 0.5\sin 2t$ rad. Gains $K_p = 100$, $K_d = 20$; simulated at 1 kHz; RMS tracking error after the first second:

| Controller | RMS error |
|---|---|
| PD alone (no model terms) | 0.0332 rad (1.9°) |
| Computed torque, exact model | 0.00014 rad |
| Computed torque, inertia and mass 20% too low | 0.0093 rad |

Cancelling the dynamics cut the error by over 200 times. With a 20% model error, much of the benefit is lost, and the remaining error comes directly from the mismatch. Hence the next steps: identify the model carefully, adapt it online ([[Adaptive_Control]]), or add robustness ([[Robust_Control]]).

## Practical notes

- **Needs torque control**: the motors must accept torque (current) commands, as on cobots, legged robots and direct-drive arms.
- **Computation**: the recursive Newton–Euler algorithm computes the full inverse dynamics in $O(n)$, well under a millisecond for a 7-joint arm with libraries such as Pinocchio or RBDL.
- **Friction and gearboxes** are the usual sources of model error; add friction models, or measure joint torque directly.

## Descendants in 2026

- **Operational-space control** (Khatib, 1987) applies the same principle to end-effector motion and force.
- **Whole-body control** for humanoids solves a quadratic program each cycle: computed-torque-style dynamics plus contact constraints, torque limits and task priorities.
- **MPC** looks ahead over a horizon rather than cancelling dynamics instantaneously (see [[Model_Predictive_Control_(MPC)]]).
- **Learned residual models**: a neural network learns the difference between the rigid-body model and reality, improving feedforward torque while keeping the model's structure.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Computed_Torque_Control]])
```
