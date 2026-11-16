---
title: Robust Control
description: "Robust control designs controllers that keep working when the model is wrong, within stated bounds. Covers uncertainty models, stability margins with a computed gain-uncertainty example, H-infinity and mu-synthesis, sliding-mode control, robust MPC, and the learning-era equivalent: domain randomisation, which trains policies to be robust to a range of simulated physics."
aliases:
  - H-infinity Control
  - Sliding Mode Control
  - Robustness
tags:
  - control
  - robust-control
  - stability
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
  - "[[Control_Theory]]"
  - "[[Control_Theory_and_Principles]]"
related:
  - "[[Adaptive_Control]]"
  - "[[Computed_Torque_Control]]"
  - "[[Sim-to-Real_Transfer]]"
  - "[[Model_Predictive_Control_(MPC)]]"
key_takeaways:
  - "Robust control accepts that the model is wrong and designs for every plant within a stated uncertainty set, guaranteeing stability and performance across it."
  - "Margins are the simplest robustness measure: if the plant's gain may be up to 1.5× nominal, the loop needs a gain margin above 1.5 (3.5 dB); a 6 dB margin tolerates up to 2×."
  - "Domain randomisation is robust control by training: a policy that succeeds across randomised masses, friction, delays and sensor noise in simulation tends to be robust on hardware."
sources:
  - "Åström, K. J. and Murray, R. M. (2021). Feedback Systems, 2nd edition, chapters 12–13 (robust performance, fundamental limits). https://fbswiki.org/wiki/index.php/Feedback_Systems:_An_Introduction_for_Scientists_and_Engineers"
  - "Peng, X. B., Andrychowicz, M., Zaremba, W. and Abbeel, P. (2017). Sim-to-Real Transfer of Robotic Control with Dynamics Randomization. https://arxiv.org/abs/1710.06537"
  - "Tobin, J. et al. (2017). Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World. https://arxiv.org/abs/1703.06907"
  - "Siciliano, B. and Khatib, O., eds. (2016). Springer Handbook of Robotics, 2nd edition, chapter on motion control (robust control of manipulators). https://link.springer.com/referencework/10.1007/978-3-319-32552-1"
---

# Robust Control

**Robust control** designs a controller for a *family* of possible systems rather than one model. Every model is wrong: payloads vary, friction changes with temperature, gearboxes wear, and simulators differ from hardware. A robust controller states how wrong the model may be and guarantees acceptable behaviour for all of those cases.

> **Etymology.** Latin *robustus*, "strong as oak" (*robur*, "oak"). Robust control became a field in the late 1970s and 1980s, when it was shown that some optimal controllers (LQG) could have arbitrarily small stability margins, prompting H-infinity methods (Zames, 1981; Doyle, Glover and others).

## Describing uncertainty

- **Parametric**: known structure, uncertain values (link mass between 1.5 and 3 kg).
- **Unstructured**: a bounded error at each frequency, for example the unmodelled flexibility of a link at high frequencies.
- **Disturbances and noise**: bounded external forces and sensor errors.

## Margins: the first robustness check

The **gain margin** is how much the loop gain can increase before instability; the **phase margin** is how much extra phase lag (delay) it tolerates.

**Worked example.** A joint's effective gain depends on the payload's inertia: with a heavy tool the motor's torque produces less acceleration, and with no tool more. Suppose the loop gain may be up to 1.5× the nominal value. The loop then needs a gain margin above 1.5, i.e. $20\log_{10}1.5 = 3.5$ dB. Designers commonly require 6 dB (a factor of 2) and 45–60° of phase margin, which covers this case with room for delays.

## Methods

| Method | Idea | Robotics use |
|---|---|---|
| **Loop shaping with margins** | Shape the open-loop gain for high gain at low frequency, low gain where the model is poor | Joint and drone loops |
| **H-infinity / mu-synthesis** | Optimise the worst-case gain from disturbances to errors over the uncertainty set | Precision positioning, flexible structures, aerospace |
| **Sliding-mode control** | Force the state onto a sliding surface with a switching law that overpowers bounded uncertainty | Robust tracking; needs smoothing to avoid chattering |
| **Robust MPC** | Plan so constraints hold for all disturbances in a set (tube MPC) | Safety-critical motion with constraints |
| **Disturbance observers** | Estimate the lumped disturbance and cancel it | Motor control, cobots |

## Robustness and adaptation

[[Adaptive_Control]] *learns* the actual system; robust control *tolerates* a range. Adaptive control can achieve better performance once it has converged; robust control is safe from the first moment. Practical controllers often combine both.

## Robustness by training (2017–2026)

**Domain randomisation** trains a policy in simulation over randomised physics: masses, friction, motor strength, latency and sensor noise (Peng et al., 2017), and appearance (Tobin et al., 2017). The policy learns behaviour that works across the whole range, the same goal as robust control, reached through sampling instead of analysis. It is the standard way legged robots, drones and dexterous hands are trained for zero-shot transfer to hardware (see [[Sim-to-Real_Transfer]]). Research now combines the two: robust-control certificates and safety filters around learned policies.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Robust_Control]])
```
