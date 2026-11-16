---
title: Pontryagin's Minimum Principle
description: "Pontryagin's minimum (maximum) principle gives necessary conditions for an optimal trajectory using costates and the Hamiltonian, without solving for the value function everywhere. Covers the conditions, a worked minimum-time example (bang-bang control of a double integrator: 1 m in 2 s with one switch), singular arcs, and its role in trajectory optimisation."
aliases:
  - Pontryagin's Maximum Principle
  - Pontryagin Minimum Principle
  - PMP
tags:
  - optimal-control
  - control
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
  - "[[Optimal_Control]]"
related:
  - "[[Hamilton-Jacobi-Bellman_Equation]]"
  - "[[Model_Predictive_Control_(MPC)]]"
key_takeaways:
  - "Along an optimal trajectory the control minimises the Hamiltonian H = ℓ + λᵀf at every instant, where the costate λ evolves backwards with λ̇ = −∂H/∂x."
  - "For minimum-time motion with bounded acceleration, the control is bang-bang: a double integrator covers 1 m from rest to rest at |a| ≤ 1 m/s² in 2 s, switching once at 1 s, peaking at 1 m/s."
  - "It gives necessary conditions for one trajectory, not a feedback law; indirect trajectory-optimisation methods solve its two-point boundary-value problem."
sources:
  - "Liberzon, D. (2011). Calculus of Variations and Optimal Control Theory: A Concise Introduction, chapter 4: The Maximum Principle. http://liberzon.csl.illinois.edu/teaching/cvoc.pdf"
  - "Bertsekas, D. P. (2017). Dynamic Programming and Optimal Control, 4th edition. http://www.athenasc.com/dpbook.html"
  - "Tedrake, R. (2026). Underactuated Robotics. https://underactuated.csail.mit.edu/"
---

# Pontryagin's Minimum Principle

**Pontryagin's minimum principle** characterises optimal trajectories: it tells you what must be true along a trajectory if it is optimal. Instead of computing the best action from every state (the [[Hamilton-Jacobi-Bellman_Equation|HJB]] route), it works along one trajectory using auxiliary **costate** variables.

> **Etymology.** Lev Pontryagin and his students Boltyanskii, Gamkrelidze and Mishchenko formulated it in 1956–1962 in Moscow, motivated by optimal flight and rocket problems. It was stated as a *maximum* principle (maximising $-H$); engineers usually write it as minimising the Hamiltonian.

## The conditions

For dynamics $\dot{\mathbf x} = f(\mathbf x, \mathbf u)$, running cost $\ell$ and admissible controls $\mathbf u \in \mathcal U$, define the **Hamiltonian**

$$H(\mathbf x, \mathbf u, \boldsymbol\lambda) = \ell(\mathbf x, \mathbf u) + \boldsymbol\lambda^\top f(\mathbf x, \mathbf u).$$

If $(\mathbf x^*, \mathbf u^*)$ is optimal, there is a costate $\boldsymbol\lambda(t)$ such that:
1. **State**: $\dot{\mathbf x}^* = \partial H/\partial \boldsymbol\lambda = f(\mathbf x^*, \mathbf u^*)$;
2. **Costate**: $\dot{\boldsymbol\lambda} = -\partial H/\partial \mathbf x$;
3. **Minimum condition**: $\mathbf u^*(t) = \arg\min_{\mathbf u \in \mathcal U} H(\mathbf x^*, \mathbf u, \boldsymbol\lambda)$ at every instant;
4. **Boundary conditions**, plus $H = 0$ along the trajectory when the final time is free.

The costate is the sensitivity of the optimal cost to the state, the gradient of the value function along the trajectory.

## Worked example: minimum time with bounded acceleration

Move a mass from rest at $x = 0$ to rest at $x = 1$ m as fast as possible, with acceleration $|u| \le 1$ m/s². State $(x, v)$, dynamics $\dot x = v$, $\dot v = u$, cost $\int 1\, dt$.

- $H = 1 + \lambda_1 v + \lambda_2 u$. Minimising over $|u| \le 1$ gives $u^* = -\operatorname{sign}(\lambda_2)$: **always at a limit** (bang-bang).
- The costate equations give $\dot\lambda_1 = 0$ and $\dot\lambda_2 = -\lambda_1$, so $\lambda_2$ is linear in time and changes sign **at most once**.

So the optimal motion is full acceleration, then full braking, with one switch. By symmetry the switch is halfway: accelerating at 1 m/s² for time $t_s$ covers $\tfrac12 t_s^2$, and braking covers the same, so $t_s^2 = 1$:

- switch at $t_s = 1$ s, at peak speed 1 m/s;
- total time $T = 2$ s, which matches $T = 2\sqrt{d/a}$.

Robots that move "as fast as possible" (pick-and-place, time-optimal paths along fixed geometric paths) show exactly this bang-bang structure, softened in practice by jerk limits for smoothness and wear.

## Subtleties

- **Necessary, not sufficient**: the conditions can have several solutions; extra checks or global methods are needed.
- **Singular arcs**: if $H$ does not depend on $\mathbf u$ over an interval, the minimum condition says nothing and higher-order conditions are needed.
- **Two-point boundary-value problem**: the state is fixed at the start and the costate (or state) at the end, so solving requires shooting methods.

## In practice (2026)

**Indirect** trajectory optimisation solves Pontryagin's conditions directly: very accurate, but sensitive to initial guesses. Robotics more often uses **direct** methods (collocation, shooting, DDP) and MPC solvers, whose solutions satisfy the same conditions implicitly (see [[Optimal_Control]], [[Model_Predictive_Control_(MPC)]]). The principle remains the best tool for understanding *why* optimal motions look as they do.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Pontryagin's_Minimum_Principle]])
```
