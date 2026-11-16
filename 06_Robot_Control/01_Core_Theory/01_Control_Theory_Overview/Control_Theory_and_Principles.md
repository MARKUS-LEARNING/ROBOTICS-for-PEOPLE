---
title: "Control Principles: What Feedback Does"
description: "The core principles of feedback, independent of any particular controller: it rejects disturbances and reduces sensitivity to model error by a factor of 1 + loop gain (computed), it can destabilise a stable system, it feeds sensor noise into the actuators, and its benefits must be paid for elsewhere (the waterbed effect). With robot examples and guidance for learned controllers."
aliases:
  - Control Principles
  - Principles of Feedback
  - Feedback Principles
tags:
  - feedback-control
  - control
  - stability
type: concept
level: intermediate
subfield:
  - control
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Feedback_Control]]"
related:
  - "[[Control_Theory]]"
  - "[[Robust_Control]]"
  - "[[PID_Control]]"
  - "[[Control_Systems]]"
key_takeaways:
  - "Feedback with loop gain L reduces the effect of disturbances and of model errors by about 1/(1 + L): a loop gain of 10 leaves 9%, a gain of 100 leaves 1%."
  - "Feedback is not free: high gain amplifies sensor noise, demands actuator effort, and with delays can turn a stable plant into an oscillating one."
  - "Improvements at some frequencies must be paid for at others (the Bode sensitivity integral, or waterbed effect), so good control design is about where to spend the benefit."
sources:
  - "Åström, K. J. and Murray, R. M. (2021). Feedback Systems: An Introduction for Scientists and Engineers, 2nd edition, chapter 12 (robust performance). https://fbswiki.org/wiki/index.php/Feedback_Systems:_An_Introduction_for_Scientists_and_Engineers"
  - "Åström, K. J. and Murray, R. M. (2020). Feedback Systems, public PDF edition. https://www.cds.caltech.edu/~murray/books/AM08/pdf/fbs-public_24Jul2020.pdf"
  - "Lynch, K. M. and Park, F. C. (2017). Modern Robotics, chapter 11: Robot Control. https://hades.mech.northwestern.edu/index.php/Modern_Robotics"
---

# Control Principles: What Feedback Does

This note is about what feedback does for a system, and what it costs, whatever controller is used. These principles explain why a well-tuned PID can make a cheap, imprecise robot accurate, and why too much gain makes it shake.

> **Etymology.** *Feedback* is a 1920s radio-engineering term: part of the output is *fed back* to the input. Harold Black's negative-feedback amplifier (1927) and Hendrik Bode's analysis of it (1940s) established most of the principles below.

## 1. Feedback rejects disturbances

With plant $P$ and controller $C$, the **loop gain** is $L = PC$. A disturbance $d$ at the output is reduced by the **sensitivity function**:

$$y_d = \frac{1}{1 + L}\, d$$

| Loop gain $L$ (at the disturbance's frequency) | Fraction of disturbance remaining |
|---|---|
| 1 | 50% |
| 10 | 9.1% |
| 100 | 1.0% |

**Robot example.** A joint holding a tool sees a load torque from the tool's weight. With a loop gain of 100 at low frequency, the position error caused by that torque is about 100 times smaller than without feedback. Integral action makes the gain at zero frequency effectively infinite, which is why PID controllers remove steady-state error entirely (see [[PID_Control]]).

## 2. Feedback reduces sensitivity to model error

The same factor applies to changes in the plant. If the motor's torque constant drifts by 10% with temperature, the closed loop's behaviour changes by only about $10\% / (1 + L)$, which is 0.1% at $L = 100$. This is why feedback lets robots work with imprecise models and cheap components.

## 3. Feedback can destabilise

A stable plant plus feedback can oscillate if the loop gain is still large when the phase lag reaches 180° (the signal returns inverted and reinforces itself). Delays, filters and flexible structures add phase lag. More gain is not always better: margins measure how far you are from this edge (see [[Control_Theory]], [[Control_Systems]]).

## 4. Feedback injects sensor noise

The controller cannot tell sensor noise from real error, so noise is fed through to the actuators by the **complementary sensitivity** $L/(1 + L)$, which is close to 1 wherever the gain is high. High-gain loops on noisy sensors make motors buzz and heat up. Hence the filtering of derivative terms, and designs that keep the gain high only at the low frequencies where disturbances live.

## 5. There is no free lunch: the waterbed effect

For most real systems, reducing sensitivity at some frequencies forces it to increase at others: Bode's sensitivity integral. Pushing down on a waterbed raises it elsewhere. Control design is therefore about **where** to put the benefit: usually at the low frequencies of loads and slow disturbances, accepting some amplification at higher frequencies where there is little energy.

## The same principles for learned controllers

Reinforcement-learned policies are feedback controllers too, and obey the same rules:
- **Effective gains**: high gains in a policy make it jittery on noisy sensors. Action smoothness penalties and low-pass filters are common fixes.
- **Delays**: latency in perception or inference erodes stability, which is why delays are randomised during training (see [[Sim-to-Real_Transfer]]).
- **Disturbance rejection**: policies trained with random pushes and payloads learn feedback that rejects them.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Control_Theory_and_Principles]])
```
