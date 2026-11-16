---
title: Control Theory
description: "The mathematics of control: modelling dynamic systems (differential equations, transfer functions, state space), stability (poles and Lyapunov), how systems respond (time constants, damping and overshoot, computed), frequency-domain design (Bode, margins), and optimal and learning-based control, with what each tells a robot engineer."
aliases:
  - Control Theory Fundamentals
tags:
  - control
  - stability
  - mathematics
type: concept
level: intermediate
subfield:
  - control
  - mathematics
status: keystone
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Control_Theory_Overview]]"
  - "[[Calculus_for_Robotics]]"
  - "[[Linear_Algebra_for_Robotics]]"
related:
  - "[[Transfer_Function]]"
  - "[[State_Space_Representation]]"
  - "[[Nonlinear_Control_Theory]]"
  - "[[Model_Predictive_Control_(MPC)]]"
  - "[[Control_Systems]]"
key_takeaways:
  - "Control theory models a system's dynamics, then asks whether the closed loop is stable, how fast and how smoothly it responds, and how it copes with disturbances and model error."
  - "A second-order loop's damping ratio sets its overshoot: ζ = 0.5 overshoots by 16%, ζ = 0.7 by under 5%, which is why designers aim near 0.7."
  - "Classical (frequency-domain), modern (state-space, optimal) and learning-based control are complementary; learned controllers increasingly come with certificates (Lyapunov or barrier functions) checked by classical theory."
sources:
  - "Åström, K. J. and Murray, R. M. (2021). Feedback Systems: An Introduction for Scientists and Engineers, 2nd edition. https://fbswiki.org/wiki/index.php/Feedback_Systems:_An_Introduction_for_Scientists_and_Engineers"
  - "Tedrake, R. (2026). Underactuated Robotics. https://underactuated.csail.mit.edu/"
  - "Boyd, S. and Vandenberghe, L. (2004). Convex Optimization. https://web.stanford.edu/~boyd/cvxbook/"
  - "Lynch, K. M. and Park, F. C. (2017). Modern Robotics, chapter 11: Robot Control. https://hades.mech.northwestern.edu/index.php/Modern_Robotics"
---

# Control Theory

**Control theory** is the mathematics of making dynamic systems behave. It answers three questions about any controller:
- **Is it stable?** Does it settle, or oscillate and diverge?
- **Does it perform?** How fast, how accurately, with how much overshoot?
- **Is it robust?** Does it still work when the model is wrong or the world pushes back?

> **Etymology.** See [[Control_Theory_Overview]]. James Clerk Maxwell's 1868 paper *On Governors*, analysing steam-engine speed regulators, is often called the founding paper of control theory.

## Modelling

| Model | Form | Best for |
|---|---|---|
| **Differential equations** | $m\ddot x + b\dot x + kx = u$ | Physics-based derivation |
| **Transfer function** | $G(s) = \dfrac{Y(s)}{U(s)}$ (Laplace domain) | Single-input single-output linear systems, frequency-domain design; see [[Transfer_Function]] |
| **State space** | $\dot{\mathbf x} = A\mathbf x + B\mathbf u,\ \mathbf y = C\mathbf x$ | Many inputs and outputs, modern and optimal control; see [[State_Space_Representation]] |
| **Nonlinear models** | $\dot{\mathbf x} = f(\mathbf x, \mathbf u)$ | Robot arms, legged robots, drones (linearised locally for design) |
| **Learned models** | Neural networks fitted to data | Hard-to-model effects; world models |

## Stability

- **Linear systems** are stable when all **poles**, the roots of the characteristic equation (the eigenvalues of $A$), have negative real parts.
- **Nonlinear systems**: find a **Lyapunov function** $V(\mathbf x) > 0$ that always decreases along trajectories, $\dot V < 0$. Then the system settles like a ball rolling to the bottom of a bowl (see [[Nonlinear_Control_Theory|Lyapunov_Stability]]).

## How systems respond, computed

**First order** ($\tau \dot y + y = u$): after a step, the output reaches 63.2% of its final value after one time constant $\tau$ ($1 - e^{-1}$) and 95.0% after three ($1 - e^{-3}$). A joint velocity loop with $\tau = 10$ ms is "settled" in about 30 ms.

**Second order**, with natural frequency $\omega_n$ and damping ratio $\zeta$: the percentage overshoot of a step response is

$$\text{OS} = e^{-\zeta\pi/\sqrt{1-\zeta^2}}$$

| Damping ratio $\zeta$ | Overshoot |
|---|---|
| 0.5 | 16.3% |
| 0.7 | 4.6% |
| 1.0 (critical) | 0% |

Designers often aim for $\zeta \approx 0.7$: fast, with little overshoot. For a robot arm, overshoot means the gripper swings past its target, so position loops are usually tuned at or near critical damping.

## Frequency-domain design

A **Bode plot** shows how a loop amplifies and delays sinusoids of each frequency.
- **Bandwidth** sets how fast the system can track.
- **Gain and phase margins** measure how far the loop is from instability. Delay eats phase margin directly (see [[Control_Systems]] for a computed example).

## Optimal and model-predictive control

**LQR** chooses the feedback gain that minimises a quadratic cost of state error and effort, $J = \int (\mathbf x^\top Q\mathbf x + \mathbf u^\top R\mathbf u)\,dt$. **[[Model_Predictive_Control_(MPC)|MPC]]** solves a finite-horizon optimisation online at every step, with constraints, which makes it the workhorse of legged and humanoid whole-body control.

## Control theory meets learning (2026)

Reinforcement-learned policies now control many legged robots and drones (see [[Reinforcement_Learning_for_Robots]]). Control theory remains essential:
- to design the low-level loops policies command;
- to analyse stability and robustness margins;
- to **certify** learned controllers: control barrier functions and learned Lyapunov functions provide safety filters that override unsafe actions.

The trend is not learning *instead of* control theory but learning *inside* a structure control theory can check.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Control_Theory]])
```
