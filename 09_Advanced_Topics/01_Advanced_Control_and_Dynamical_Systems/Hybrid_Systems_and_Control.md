---
title: Hybrid Systems and Control
description: "Hybrid systems mix continuous dynamics with discrete jumps or mode switches: feet touching down, grippers closing, controllers switching. Covers the hybrid-automaton model, a computed bouncing-ball example with Zeno behaviour (infinitely many bounces in about 4 s), stability under switching, walking robots as hybrid systems, and how contact-rich simulation and learning handle hybrid dynamics."
aliases:
  - Hybrid Systems
  - Hybrid Dynamical Systems
  - Hybrid Automata
tags:
  - control
  - locomotion
  - mathematics
type: concept
level: research
subfield:
  - control
  - legged-humanoid
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Nonlinear_Control_Theory]]"
related:
  - "[[Control_of_Underactuated_Systems]]"
  - "[[Formal_Methods_in_Control]]"
  - "[[Legged_Robots]]"
  - "[[Physics_Engines]]"
key_takeaways:
  - "A hybrid system has modes with their own continuous dynamics, plus guard conditions and reset maps that switch modes and jump the state: exactly what contact does to a robot."
  - "A ball dropped from 1 m with restitution 0.8 bounces infinitely often yet stops in about 4.06 s (Zeno behaviour), a reminder that hybrid models can have pathologies simulators must handle."
  - "Walking is hybrid (swing, impact, stance); classical controllers exploit the structure (hybrid zero dynamics, contact-aware MPC), while RL policies trained in contact-rich simulators learn it implicitly."
sources:
  - "Tedrake, R. (2026). Underactuated Robotics, chapter: Simple Models of Walking and Running. https://underactuated.csail.mit.edu/simple_legs.html"
  - "Belta, C., Yordanov, B. and Aydin Gol, E. (2017). Formal Methods for Discrete-Time Dynamical Systems. https://link.springer.com/book/10.1007/978-3-319-50763-7"
  - "Bansal, S., Chen, M., Herbert, S. and Tomlin, C. J. (2017). Hamilton-Jacobi Reachability: A Brief Overview and Recent Advances. https://arxiv.org/abs/1709.07523"
---

# Hybrid Systems and Control

A **hybrid system** combines continuous dynamics (motion governed by differential equations) with discrete events (switches between modes, sudden jumps in state). Robots are full of them: a foot touching down, a gripper closing on an object, a controller switching from "approach" to "insert", a drone's motor failing.

> **Etymology.** Latin *hybrida*, "offspring of mixed parentage": a mix of continuous and discrete. The hybrid-automaton formalism took shape in the early 1990s at the meeting point of control theory and computer science.

## The model

A **hybrid automaton** has:
- **modes** $q$ (flight, stance, grasped, free), each with continuous dynamics $\dot{\mathbf x} = f_q(\mathbf x)$;
- **guards**: conditions that trigger a switch (foot height reaches zero);
- **reset maps**: an instantaneous change of state at the switch (impact makes the velocity jump);
- **domains**: where each mode's dynamics are valid.

## Worked example: the bouncing ball and Zeno behaviour

A ball is dropped from 1 m with coefficient of restitution $e = 0.8$:
- **flight mode**: $\ddot y = -g$;
- **guard**: $y = 0$ with $\dot y < 0$;
- **reset**: $\dot y^+ = -e\,\dot y^-$.

Each bounce reaches $e^2$ of the previous height: 0.64 m, then 0.41 m, and so on. The first fall takes $t_0 = \sqrt{2 \times 1/9.81} = 0.452$ s. Each later flight time is $e$ times the one before, so the total time is a geometric series:

$$T = t_0\Big(1 + \frac{2e}{1 - e}\Big) = 0.452 \times 9 = 4.06\ \text{s}.$$

There are **infinitely many bounces in finite time**: Zeno behaviour. A naive simulator would take ever-smaller steps and stall. Real simulators and controllers must detect this and switch to a resting mode. Robots meet the same issue when a foot or fingertip chatters against a surface.

## Stability under switching

Switching between individually stable modes can still be unstable, and vice versa. Tools include:
- **common Lyapunov functions**, one $V$ that decreases in every mode;
- **dwell-time conditions**, which require enough time between switches;
- **reachability analysis**, computing every state the system can reach, used for safety verification (see [[Formal_Methods_in_Control]]).

## Walking robots as hybrid systems

A walking step is swing (continuous) → impact (reset) → stance with the other leg. Controllers that respect this structure include:
- **hybrid zero dynamics**: design virtual constraints so the closed-loop gait is a stable periodic orbit; used on bipeds such as MABEL and Cassie;
- **contact-aware MPC**, which plans over a given or optimised contact sequence (see [[Model_Predictive_Control_(MPC)]]);
- **contact-implicit optimisation**, where the optimiser chooses when contacts happen.

## Hybrid dynamics in 2026

Reinforcement-learned locomotion and manipulation policies are trained in simulators that model contact as complementarity constraints or soft contacts (see [[Physics_Engines]]). The policy never sees "modes" explicitly; it learns to act well across them. Formal and classical hybrid methods remain important for verification, for safety-critical switching logic, and for understanding why learned gaits are stable.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Hybrid_Systems_and_Control]])
```
