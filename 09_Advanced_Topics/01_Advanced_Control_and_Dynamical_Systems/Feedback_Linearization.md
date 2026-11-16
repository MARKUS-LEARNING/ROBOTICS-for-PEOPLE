---
title: Feedback Linearization
description: "Feedback linearisation uses the control input to cancel a system's nonlinearities so that the closed loop behaves linearly. Covers full-state and input–output linearisation, relative degree and Lie derivatives, zero dynamics, a worked pendulum example, the robot-arm case (computed torque), and its limits: model accuracy, actuator limits and underactuation."
aliases:
  - Feedback Linearisation
  - Input-Output Linearization
  - Exact Linearization
tags:
  - feedback-linearization
  - control
  - mathematics
type: method
level: research
subfield:
  - control
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Nonlinear_Control_Theory]]"
  - "[[Computed_Torque_Control]]"
related:
  - "[[Robust_Control_Methods]]"
  - "[[Control_of_Underactuated_Systems]]"
key_takeaways:
  - "If a system's nonlinearities enter through the same channel as the input, the input can cancel them exactly, leaving linear dynamics to design for."
  - "For a pendulum, u = ml²v + b θ̇ + mgl sin θ makes θ̈ = v; at θ = 60° (m = 1 kg, l = 0.5 m) the gravity term alone is 4.25 N·m."
  - "It needs an accurate model and enough actuator authority, and it fails for underactuated systems whose nonlinearities sit in unactuated directions; there, partial linearisation, energy shaping or learning are used."
sources:
  - "Khalil, H. K. (2002). Nonlinear Systems, 3rd edition, chapter 13: Feedback Linearization. https://www.pearson.com/en-us/subject-catalog/p/nonlinear-systems/P200000003364"
  - "Murray, R. M., Li, Z. and Sastry, S. S. (1994). A Mathematical Introduction to Robotic Manipulation. http://www.cds.caltech.edu/~murray/books/MLS/pdf/mls94-complete.pdf"
  - "Tedrake, R. (2026). Underactuated Robotics. https://underactuated.csail.mit.edu/"
---

# Feedback Linearization

**Feedback linearisation** uses the control input to cancel a system's nonlinear terms, so that what remains is a linear system that standard methods (pole placement, LQR) can control. It is the theory behind [[Computed_Torque_Control]] for robot arms.

> **Etymology.** *Linearise*: make linear. Unlike Jacobian linearisation, which approximates dynamics near one point, feedback linearisation makes them *exactly* linear across a region, through feedback. The geometric theory was developed in the late 1970s and 1980s (Brockett, Isidori, Hunt, Su and Meyer).

## The idea on a pendulum

For $ml^2\ddot\theta + b\dot\theta + mgl\sin\theta = u$, choose

$$u = ml^2 v + b\dot\theta + mgl\sin\theta.$$

Substituting gives $\ddot\theta = v$: a double integrator, whatever the angle. Then choose $v = \ddot\theta_d + k_d\dot e + k_p e$ for any desired error dynamics.

**Numbers.** With $m = 1$ kg and $l = 0.5$ m at $\theta = 60°$, the gravity term the controller must cancel is $1 \times 9.81 \times 0.5 \times \sin 60° = 4.25$ N·m. If the motor can deliver only 3 N·m, the cancellation is impossible at that angle. That is the most common practical failure of feedback linearisation: actuator limits.

## General theory, briefly

For $\dot{\mathbf x} = f(\mathbf x) + g(\mathbf x)u$ with output $y = h(\mathbf x)$:
- differentiate $y$ until $u$ appears. Using Lie derivatives, $y^{(r)} = L_f^r h + L_g L_f^{r-1}h\ u$, where $r$ is the **relative degree**;
- choose $u = (v - L_f^r h)/(L_g L_f^{r-1}h)$, which makes $y^{(r)} = v$: **input–output linearisation**;
- if $r$ equals the state dimension, the whole state is linearised (full-state linearisation);
- otherwise the leftover **zero dynamics** must be stable, or the internal state can drift even while the output is perfect. Systems with unstable zero dynamics are called non-minimum phase.

## Robot arms

Fully actuated arms (one motor per joint) have relative degree 2 in every joint, with no zero dynamics. Computed torque, $\boldsymbol\tau = M(\mathbf q)\mathbf v + C\dot{\mathbf q} + \mathbf g$, linearises them exactly (see [[Computed_Torque_Control]]).

## Limits

- **Model dependence**: cancellation is only as good as the model; errors appear as disturbances. Combine with [[Robust_Control_Methods]] or adaptation.
- **Actuator limits**: cancelling large nonlinearities needs large inputs.
- **Underactuation**: legged robots in flight phase, drones and balancing robots cannot cancel everything. Partial feedback linearisation, energy shaping, MPC and learned policies take over (see [[Control_of_Underactuated_Systems]]).
- **Wasted effort**: cancelling helpful dynamics (for example gravity that would assist a motion) can be inefficient.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Feedback_Linearization]])
```
