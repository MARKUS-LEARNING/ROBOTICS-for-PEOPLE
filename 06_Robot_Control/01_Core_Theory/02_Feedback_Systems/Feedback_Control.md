---
title: Feedback Control
description: "Feedback control measures what a system is actually doing and corrects the difference from what it should do. A first explanation for beginners: negative and positive feedback, the loop and its parts, a computed motor-speed example with and without feedback, everyday and robot examples, and why feedback is everywhere in robots from motor drivers to learned policies."
aliases:
  - Feedback
  - Closed-Loop Control
  - Negative Feedback
tags:
  - feedback-control
  - control
type: concept
level: beginner
subfield:
  - control
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Control_Theory_Overview]]"
related:
  - "[[PID_Control]]"
  - "[[Control_Theory_and_Principles]]"
  - "[[Control_Systems]]"
  - "[[Sensors]]"
  - "[[Encoders]]"
key_takeaways:
  - "Negative feedback subtracts the measured output from the goal and acts on the error, which makes systems accurate and steady even when loads and conditions change."
  - "With a loop gain of 20, a load that would slow an uncontrolled motor by 20% slows it by under 1%."
  - "Positive feedback amplifies change; useful for snapping between states, dangerous in a controller."
sources:
  - "Åström, K. J. and Murray, R. M. (2021). Feedback Systems: An Introduction for Scientists and Engineers, 2nd edition. https://fbswiki.org/wiki/index.php/Feedback_Systems:_An_Introduction_for_Scientists_and_Engineers"
  - "Åström, K. J. and Murray, R. M. (2020). Feedback Systems, public PDF edition. https://www.cds.caltech.edu/~murray/books/AM08/pdf/fbs-public_24Jul2020.pdf"
  - "Lynch, K. M. and Park, F. C. (2017). Modern Robotics, chapter 11: Robot Control. https://hades.mech.northwestern.edu/index.php/Modern_Robotics"
---

# Feedback Control

**Feedback control** is the idea of measuring what a system is actually doing, comparing it with what it should be doing, and using the difference to adjust. You do it constantly: steering a bicycle, adjusting the shower temperature, reaching for a cup. Robots do it thousands of times per second.

> **Etymology.** *Feed back*: send part of the output *back* to the input. The term comes from 1920s radio engineering. Feedback devices are much older: Ktesibios's water clock regulator (3rd century BC) and James Watt's centrifugal governor (1788).

## The loop

1. **Goal** (reference): what we want, for example a joint at 90°.
2. **Sensor**: measures the actual value, for example an encoder reading 85°.
3. **Comparison**: the error is 90° − 85° = 5°.
4. **Controller**: decides a correction from the error.
5. **Actuator**: applies it (motor torque), and the loop repeats.

## Negative and positive feedback

- **Negative feedback** acts *against* the error: too low, push up; too high, push down. It stabilises and is what controllers use.
- **Positive feedback** acts *with* the change, amplifying it: microphone squeal, a ball rolling off a hill. It is useful where you want a decisive switch between states (a latching circuit, a snap-through gripper) and dangerous in a controller.

## Worked example: holding speed under load

A wheel motor runs at 100 rpm with no load. A load that, without feedback, would slow it by 20% (to 80 rpm) is applied.

With negative feedback of loop gain $L = 20$ (the controller's correction is 20 times the speed error, in consistent units), the speed drop is reduced by the factor $1/(1 + L) = 1/21$:

$$\text{drop} = 20\% \times \frac{1}{21} \approx 0.95\%,$$

so the wheel keeps about **99 rpm**. Adding integral action (see [[PID_Control]]) removes even that remaining error. This is the core benefit of feedback, explained further in [[Control_Theory_and_Principles]].

## Open loop versus closed loop

| | Open loop | Closed loop (feedback) |
|---|---|---|
| Measures the result? | No | Yes |
| Handles disturbances and model error? | No | Yes |
| Risk of instability? | No | Yes, if badly tuned |
| Example | Stepper motor moved by counting steps; toaster timer | Servo motor with encoder; robot joint controller; cruise control |

Many robot systems combine **feedforward** (what a model predicts is needed) with **feedback** (correcting what the model missed).

## Feedback everywhere in a robot

- **Motor drivers** regulate current tens of thousands of times per second.
- **Joint controllers** hold positions and follow trajectories (see [[ros2_control]]).
- **Balance**: drones and legged robots use IMU feedback to stay upright.
- **Navigation**: localization corrects the robot's position estimate, and the path follower corrects deviations.
- **Learned policies** are feedback controllers too: they observe the state and act on it, many times per second.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Feedback_Control]])
```
