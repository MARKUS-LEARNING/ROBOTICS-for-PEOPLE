---
title: Control Systems
description: "How a robot control system is built and run: the parts (sensors, controller, actuators, plant), open and closed loop, cascaded and feedforward structures, discrete-time implementation on a computer, sampling rates and delay (with a computed phase-lag example), saturation and anti-windup, and the real-time software that runs it in 2026."
aliases:
  - Control System
  - Closed-Loop Control
  - Open-Loop Control
tags:
  - control
  - feedback-control
  - real-time
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
  - "[[Control_Theory]]"
  - "[[PID_Control]]"
  - "[[ros2_control]]"
  - "[[Feedback_Control]]"
  - "[[Sensors]]"
key_takeaways:
  - "A control system is sensors, a controller, actuators and the plant, connected in a loop; feedback makes it accurate despite disturbances and model errors."
  - "Structure matters as much as tuning: cascaded loops (current inside velocity inside position) and feedforward of known dynamics make robots fast and precise."
  - "Implemented digitally, a controller samples and acts at a fixed rate; every millisecond of delay costs phase margin, which is why joint loops run at 1 kHz or faster on real-time systems."
sources:
  - "Åström, K. J. and Murray, R. M. (2021). Feedback Systems: An Introduction for Scientists and Engineers, 2nd edition. https://fbswiki.org/wiki/index.php/Feedback_Systems:_An_Introduction_for_Scientists_and_Engineers"
  - "Lynch, K. M. and Park, F. C. (2017). Modern Robotics, chapter 11: Robot Control. https://hades.mech.northwestern.edu/index.php/Modern_Robotics"
  - "ros-controls (2026). ros2_control. https://github.com/ros-controls/ros2_control"
  - "Siciliano, B. and Khatib, O., eds. (2016). Springer Handbook of Robotics, 2nd edition, chapter on motion control. https://link.springer.com/referencework/10.1007/978-3-319-32552-1"
---

# Control Systems

A **control system** makes a physical system do what we want: hold a joint at an angle, follow a trajectory, keep a drone level. [[Control_Theory]] provides the mathematics; this note is about how a control system is put together and run on a real robot.

> **Etymology.** See [[Control_Theory_Overview]]. *System* is from Greek *systēma*, "an organised whole".

## The parts

| Part | Role | Robot example |
|---|---|---|
| **Plant** | The thing being controlled | A robot joint with its motor, gearbox and link |
| **Sensors** | Measure what the plant is doing | Encoders, IMU, force-torque sensors |
| **Controller** | Computes commands from the error between goal and measurement | PID or model-based controller running in software |
| **Actuators** | Apply the commands | Motors driven by power electronics |
| **Reference** | The goal | Desired angle or trajectory from a planner or policy |

## Open loop and closed loop

- **Open loop** applies a precomputed command without measuring the result: run the motor at 50% for 2 s. It is simple, but cannot correct for load changes, friction or slip.
- **Closed loop** (feedback) measures the result and corrects. It is accurate and robust, but must be designed so it does not oscillate or go unstable.

Most robots use both: **feedforward** sends the command a model predicts is needed (for example gravity compensation), and **feedback** corrects what the model got wrong.

## Common structures

- **Cascade control**: an outer position loop sets a velocity target for a middle velocity loop, which sets a current (torque) target for an inner current loop. Each inner loop runs faster than the one outside it, typically 5–10× faster.
- **Feedforward + feedback**: the model handles predictable dynamics and feedback handles the rest. See [[Computed_Torque_Control]].
- **Multiple-input multiple-output (MIMO)**: whole-body controllers coordinate many joints at once using models of the full robot.

## Running on a computer: sampling and delay

Digital controllers read sensors, compute and write commands once per **sample period** $T_s$. Delay matters because it adds phase lag: a pure delay $\tau$ adds $\omega\tau$ radians of lag at frequency $\omega$.

**Worked example.** A joint loop with a crossover (bandwidth) frequency of 20 Hz, $\omega = 2\pi \times 20 = 125.7$ rad/s, and a total delay of 2 ms (one sample at 1 kHz for sensing, one for actuation) loses $125.7 \times 0.002 = 0.251$ rad ≈ **14.4°** of phase margin. With 10 ms of delay (a 100 Hz loop) it loses 72°, enough to destabilise a loop that had 60° of margin. This is why joint controllers run at 1 kHz or more on real-time hardware, and why slower components such as policies and planners send *targets* to fast loops rather than closing the joint loop themselves.

## Practical issues

- **Saturation**: motors have current and voltage limits. When the controller asks for more than the motor can give, integral terms "wind up"; anti-windup logic stops this.
- **Noise**: derivative terms amplify sensor noise, so filter it.
- **Quantisation and resolution**: encoder counts limit precision.
- **Safety**: limits, watchdogs and emergency stops sit around the controller, not inside its tuning.

## Real-time software (2026)

On ROS 2 robots, [[ros2_control]] runs hardware interfaces and controllers in a real-time loop (often 1 kHz on a PREEMPT_RT Linux kernel). Motor drivers run the fastest current loops in firmware, and EtherCAT or CAN buses carry commands. Learned policies and planners run in separate, non-real-time processes and exchange targets with the real-time loop.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Control_Systems]])
```
