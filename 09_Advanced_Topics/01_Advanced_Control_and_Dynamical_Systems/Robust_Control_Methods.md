---
title: Robust Control Methods
description: "The mathematics behind robust control: uncertainty models, the H-infinity norm and small-gain theorem, mixed sensitivity and mu-synthesis, and sliding-mode control with a computed reaching-time bound. A deeper companion to the Robust Control note, with connections to robust MPC and to domain randomisation in learning."
aliases:
  - H-infinity
  - Mu-Synthesis
  - Small Gain Theorem
tags:
  - robust-control
  - control
  - mathematics
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
  - "[[Robust_Control]]"
  - "[[Nonlinear_Control_Theory]]"
related:
  - "[[Adaptive_Control_Theory]]"
  - "[[Model_Predictive_Control_(MPC)]]"
  - "[[Sim-to-Real_Transfer]]"
key_takeaways:
  - "The H-infinity norm is a system's worst-case gain over all frequencies; the small-gain theorem guarantees stability when the loop gain through the uncertainty stays below 1."
  - "Mixed-sensitivity design trades disturbance rejection, noise rejection and robustness by weighting S, T and KS across frequency; mu-synthesis handles structured uncertainty."
  - "Sliding-mode control rejects any matched disturbance below its switching gain: with |d| ≤ 2 and gain 2.5, the state reaches the sliding surface from |s| = 0.5 within 1 s."
sources:
  - "Åström, K. J. and Murray, R. M. (2021). Feedback Systems, 2nd edition, chapters 12–13. https://fbswiki.org/wiki/index.php/Feedback_Systems:_An_Introduction_for_Scientists_and_Engineers"
  - "Khalil, H. K. (2002). Nonlinear Systems, 3rd edition, chapter 14 (sliding mode control). https://www.pearson.com/en-us/subject-catalog/p/nonlinear-systems/P200000003364"
  - "Peng, X. B., Andrychowicz, M., Zaremba, W. and Abbeel, P. (2017). Sim-to-Real Transfer of Robotic Control with Dynamics Randomization. https://arxiv.org/abs/1710.06537"
---

# Robust Control Methods

This note goes beneath the overview in [[Robust_Control]] to the tools themselves: how uncertainty is quantified, and how controllers are designed with guarantees for every system within it.

> **Etymology.** *H-infinity* refers to the Hardy space $\mathcal H_\infty$ of stable transfer functions with bounded peak gain, named after mathematician G. H. Hardy. George Zames introduced H-infinity control in 1981; Doyle, Glover, Khargonekar and Francis gave the state-space solution in 1989.

## The H-infinity norm and the small-gain theorem

The **H-infinity norm** of a stable system is its peak gain over frequency, $\lVert G\rVert_\infty = \sup_\omega |G(j\omega)|$: the worst-case amplification of any input.

**Small-gain theorem**: if a stable system $M$ is connected in a loop with an uncertainty $\Delta$, and $\lVert M\rVert_\infty \lVert\Delta\rVert_\infty < 1$, the loop is stable. Model the uncertainty as $\Delta$ (for example, the unmodelled flexibility of a link, bounded by a weight $W(s)$), and robust stability becomes a norm condition you can design for.

## Mixed sensitivity and mu-synthesis

H-infinity design chooses a controller $K$ to keep weighted closed-loop functions small:
- $W_1 S$: disturbance rejection and tracking at low frequency;
- $W_2 T$: robustness to high-frequency model error and noise;
- $W_3 KS$: limits actuator effort.

The waterbed limits (see [[Control_Theory_and_Principles]]) mean the weights must be feasible together. **Mu-synthesis** extends this to structured uncertainty (separate uncertain parameters) using the structured singular value $\mu$.

## Sliding-mode control

Define a sliding surface, for example $s = \dot e + \lambda e$. On $s = 0$ the error decays as $e^{-\lambda t}$. The control drives $s$ to zero with a switching term:

$$u = u_{eq} - k\,\operatorname{sign}(s), \qquad \tfrac{d}{dt}\big(\tfrac12 s^2\big) = s\dot s \le -(k - d_{max})\lvert s\rvert.$$

**Worked example.** If the matched disturbance is bounded by $|d| \le 2$ and we choose $k = 2.5$, then $|s|$ decreases at least at rate $k - d_{max} = 0.5$. From $|s(0)| = 0.5$, the state reaches the surface within $0.5/0.5 = 1$ s, and stays there whatever the disturbance does. The price is **chattering** (fast switching), so practical designs replace $\operatorname{sign}(s)$ with a saturation function or use higher-order sliding modes.

## Robust MPC

Tube MPC plans a nominal trajectory and a "tube" around it that contains every trajectory the bounded disturbance could produce, tightening the constraints so that all of them stay feasible (see [[Model_Predictive_Control_(MPC)]]).

## Robustness and learning

Domain randomisation trains a policy to perform across a *distribution* of models, the sampling counterpart of designing for an uncertainty set. Robust-control analysis is increasingly applied to learned policies, estimating their gains and margins, and robust objectives (worst case over randomised parameters) are used in RL training.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Robust_Control_Methods]])
```
