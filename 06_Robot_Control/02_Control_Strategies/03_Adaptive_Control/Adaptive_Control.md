---
title: Adaptive Control
description: "Adaptive control adjusts a controller's parameters while it runs, to cope with unknown or changing dynamics such as an unknown payload. Covers the main approaches, Slotine–Li adaptive control for robot arms with a simulated example in which the controller learns a link's mass and tracking error falls 150-fold, conditions for convergence, and learned adaptation in legged robots."
aliases:
  - Adaptive Controller
  - MRAC
  - Model Reference Adaptive Control
tags:
  - control
  - adaptive-control
  - system-identification
type: method
level: advanced
subfield:
  - control
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Computed_Torque_Control]]"
  - "[[Control_Theory]]"
related:
  - "[[Robust_Control]]"
  - "[[Reinforcement_Learning_for_Robots]]"
  - "[[Legged_Robots]]"
key_takeaways:
  - "Adaptive controllers estimate unknown parameters online and use the estimates in the control law, with an update rule chosen so that a Lyapunov argument guarantees stable tracking."
  - "In a simulated arm whose controller started with a link mass 20% too low, the adaptive law converged to the true 2.00 kg and the tracking error fell from 0.011 rad to 0.00007 rad RMS."
  - "Learned adaptation (RMA, 2021) does the same job for legged robots: a network infers terrain and payload properties from recent history and adjusts the policy within a fraction of a second."
sources:
  - "Slotine, J.-J. E. and Li, W. (1987). On the Adaptive Control of Robot Manipulators. https://journals.sagepub.com/doi/10.1177/027836498700600303"
  - "Kumar, A., Fu, Z., Pathak, D. and Malik, J. (2021). RMA: Rapid Motor Adaptation for Legged Robots. https://arxiv.org/abs/2107.04034"
  - "Lee, J. et al. (2020). Learning Quadrupedal Locomotion over Challenging Terrain. https://arxiv.org/abs/2010.11251"
  - "Åström, K. J. and Murray, R. M. (2021). Feedback Systems, 2nd edition. https://fbswiki.org/wiki/index.php/Feedback_Systems:_An_Introduction_for_Scientists_and_Engineers"
---

# Adaptive Control

An **adaptive controller** changes its own parameters while it runs. When a robot picks up an unknown payload, walks onto a slippery floor or wears out its gearbox, a fixed controller tuned for nominal conditions degrades. An adaptive one estimates the change and compensates.

> **Etymology.** Latin *adaptare*, "to fit to". Adaptive control began in the 1950s with aircraft autopilots that had to work across very different speeds and altitudes; early failures, including the 1967 X-15 crash, spurred the stability theory that now underpins it.

## Main approaches

| Approach | Idea |
|---|---|
| **Model-reference adaptive control (MRAC)** | Adjust parameters so the real system follows a reference model's response |
| **Self-tuning regulators** | Identify the plant online (e.g. recursive least squares), then redesign the controller from the estimate |
| **Gain scheduling** | Switch between pre-tuned gains according to measured conditions (not adaptive in the strict sense, but widely used) |
| **Lyapunov-based adaptation for robots** | Estimate the robot's physical parameters (masses, inertias, friction) inside a computed-torque-style law; Slotine and Li (1987) |
| **Learned adaptation** | A neural network infers hidden parameters from recent sensor history |

## The robot-arm case

Robot dynamics are **linear in the physical parameters**: $M\ddot{\mathbf q} + C\dot{\mathbf q} + \mathbf g = Y(\mathbf q,\dot{\mathbf q},\ddot{\mathbf q})\,\boldsymbol\theta$, where $Y$ is known and $\boldsymbol\theta$ collects masses and inertias. Slotine and Li's controller uses the estimate $\hat{\boldsymbol\theta}$ in the control law and updates it with

$$\dot{\hat{\boldsymbol\theta}} = \Gamma\, Y^\top \mathbf s, \qquad \mathbf s = \dot{\mathbf e} + \Lambda\mathbf e,$$

which a Lyapunov argument proves keeps tracking errors bounded and driving to zero.

## Worked example: learning an unknown link mass

A one-link arm (true mass 2.0 kg, centre of mass 0.4 m) tracks $q_d = 0.5\sin 2t$. The controller starts with an estimate of 1.6 kg, 20% low. Simulated at 1 kHz ($\Lambda = 10$, damping gain 5, adaptation gain $\Gamma = 2$):

| | RMS error, second 1–2 | second 5–6 | second 19–20 | Final mass estimate |
|---|---|---|---|---|
| Fixed estimate | 0.0090 rad | 0.0118 rad | 0.0108 rad | 1.6 kg |
| Adaptive | 0.0037 rad | 0.00027 rad | 0.00007 rad | 2.00 kg |

The adaptive controller found the true mass and reduced the error about 150-fold compared with the fixed controller. Convergence of the *parameter* (not just the error) needed the motion to be **persistently exciting**: the sinusoid moves the arm through enough postures to reveal the gravity term. A robot holding still cannot learn its payload's mass this way.

## Cautions

- Adaptation can drift with noise or unmodelled dynamics. Projection, dead zones or leakage terms keep estimates sensible.
- Fast adaptation interacts with the feedback loop; tune the adaptation gain conservatively.
- Combine with [[Robust_Control]] so that the controller stays safe while estimates are still wrong.

## Learned adaptation (2021–2026)

**Rapid Motor Adaptation (RMA)** trains a legged robot's policy in simulation with randomised terrain, payloads and friction, together with a network that estimates those hidden parameters from the last half-second or so of joint and IMU history. On hardware, the estimate updates continuously and the policy adapts in a fraction of a second, without any explicit model. Teacher–student training (Lee et al., 2020) uses the same idea. Recurrent and transformer policies that condition on history (see [[RNN_and_LSTM]]) are now the common way legged and humanoid robots adapt.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Adaptive_Control]])
```
