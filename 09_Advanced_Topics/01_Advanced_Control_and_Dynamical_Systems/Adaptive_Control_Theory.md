---
title: Adaptive Control Theory
description: "The theory behind adaptive control: model-reference adaptive control (MRAC) with a Lyapunov-derived update law, a simulated scalar example that stabilises an unknown unstable plant and learns its ideal gains, persistence of excitation, robustness modifications (sigma, projection), L1 adaptive control, and the relationship to learned adaptation in 2026."
aliases:
  - MRAC Theory
  - Lyapunov-Based Adaptation
tags:
  - adaptive-control
  - control
  - stability
type: concept
level: research
subfield:
  - control
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Adaptive_Control]]"
  - "[[Nonlinear_Control_Theory]]"
related:
  - "[[Robust_Control_Methods]]"
  - "[[Reinforcement_Learning_for_Robots]]"
key_takeaways:
  - "MRAC makes an uncertain plant follow a reference model by adjusting gains with an update law derived from a Lyapunov function, which guarantees bounded signals and convergence of the tracking error."
  - "In simulation, MRAC stabilised an unstable plant (ẋ = x + u, with the 1 unknown to the controller) and its tracking error fell from 0.32 to 0.001 RMS in 20 s, with gains approaching the ideal −3 and 2."
  - "Parameters converge only with persistently exciting signals; robustness modifications prevent drift; learned adaptation (e.g. RMA) is the data-driven counterpart."
sources:
  - "Slotine, J.-J. E. and Li, W. (1987). On the Adaptive Control of Robot Manipulators. https://journals.sagepub.com/doi/10.1177/027836498700600303"
  - "Khalil, H. K. (2002). Nonlinear Systems, 3rd edition. https://www.pearson.com/en-us/subject-catalog/p/nonlinear-systems/P200000003364"
  - "Kumar, A., Fu, Z., Pathak, D. and Malik, J. (2021). RMA: Rapid Motor Adaptation for Legged Robots. https://arxiv.org/abs/2107.04034"
---

# Adaptive Control Theory

This note gives the theory behind [[Adaptive_Control]]: how an adaptive law is derived so that learning the parameters cannot destabilise the system.

> **Etymology.** See [[Adaptive_Control]]. Lyapunov-based design of adaptive laws (Parks, 1966) replaced the earlier "MIT rule", a gradient method that could go unstable, and became the standard way to prove adaptive systems safe.

## Model-reference adaptive control (MRAC)

**Plant:** $\dot x = a x + u$, with $a$ unknown (it may be unstable). **Reference model:** $\dot x_m = a_m x_m + b_m r$, with $a_m < 0$ (the behaviour we want). **Controller:** $u = k_x x + k_r r$.

If $a$ were known, the ideal gains would be $k_x^* = a_m - a$ and $k_r^* = b_m$. With the tracking error $e = x - x_m$ and the Lyapunov function

$$V = \tfrac12 e^2 + \tfrac{1}{2\gamma}\big[(k_x - k_x^*)^2 + (k_r - k_r^*)^2\big],$$

choosing the update laws

$$\dot k_x = -\gamma\, e\, x, \qquad \dot k_r = -\gamma\, e\, r$$

makes $\dot V = a_m e^2 \le 0$. All signals stay bounded and, by Barbalat's lemma, $e \to 0$, without knowing $a$.

## Worked example (simulated)

Plant $\dot x = x + u$ (unstable, $a = 1$ unknown to the controller); model $\dot x_m = -2x_m + 2r$; $r$ a ±1 square wave with a 10 s period; $\gamma = 5$; gains start at 0; Euler integration at 1 ms:

| Time window | RMS tracking error |
|---|---|
| 0–1 s | 0.321 |
| 4–5 s | 0.137 |
| 9–10 s | 0.011 |
| 19–20 s | 0.0012 |

After 20 s the gains were $k_x = -2.90$ and $k_r = 1.90$, approaching the ideal $-3$ and $2$. The square-wave reference is **persistently exciting**: it varies enough to reveal both parameters. With a constant reference, the error would still converge but the gains need not reach their ideal values.

## Robustness modifications

Pure adaptive laws can drift under noise and unmodelled dynamics (the "bursting" phenomenon). Standard remedies:
- **σ-modification and e-modification**: add leakage toward a nominal value;
- **projection**: keep estimates in a known bounded set;
- **dead zone**: stop adapting when the error is at noise level.

**L1 adaptive control** decouples fast adaptation from robustness with a low-pass filter on the control signal, and has been flight-tested on aircraft.

## Learned adaptation

Modern robots often adapt with networks trained in simulation: an adaptation module infers latent environment parameters from recent history, and the policy conditions on them (RMA, 2021). That approach is data-driven and lacks the Lyapunov guarantees above, but scales to high-dimensional legged dynamics. Combining both, learned adaptation inside a provably stable structure, is an active research topic.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Adaptive_Control_Theory]])
```
