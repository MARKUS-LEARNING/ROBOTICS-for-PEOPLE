---
title: Control Theory Overview
description: "Section overview for robot control: what a controller does, the feedback loop, the layers of control on a 2026 robot (from 1 kHz joint loops to learned policies), and a guided map to the notes in this chapter: theory and modelling, classical and modern strategies, interaction control, navigation, and human-robot interaction."
aliases:
  - Control Systems Overview
  - Robot Control Overview
tags:
  - control
  - feedback-control
  - stability
type: overview
level: beginner
subfield:
  - control
status: keystone
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Robots]]"
related:
  - "[[Control_Systems]]"
  - "[[Control_Theory]]"
  - "[[Control_Theory_and_Principles]]"
  - "[[PID_Control]]"
  - "[[Model_Predictive_Control_(MPC)]]"
  - "[[Reinforcement_Learning_for_Robots]]"
key_takeaways:
  - "Control turns goals into motor commands, using feedback to correct errors as they happen."
  - "A modern robot stacks controllers by speed: current loops at 10–40 kHz, joint loops at 1 kHz, whole-body or MPC at 100–1,000 Hz, learned policies at 10–500 Hz, planners and language models slower still."
  - "Classical control supplies guarantees (stability, limits, safety); learned control supplies flexibility; 2026 systems combine both."
sources:
  - "Åström, K. J. and Murray, R. M. (2021). Feedback Systems: An Introduction for Scientists and Engineers, 2nd edition. https://fbswiki.org/wiki/index.php/Feedback_Systems:_An_Introduction_for_Scientists_and_Engineers"
  - "Lynch, K. M. and Park, F. C. (2017). Modern Robotics, chapter 11: Robot Control. https://hades.mech.northwestern.edu/index.php/Modern_Robotics"
  - "Tedrake, R. (2026). Underactuated Robotics: Algorithms for Walking, Running, Swimming, Flying, and Manipulation. https://underactuated.csail.mit.edu/"
  - "Siciliano, B. and Khatib, O., eds. (2016). Springer Handbook of Robotics, 2nd edition. https://link.springer.com/referencework/10.1007/978-3-319-32552-1"
---

# Control Theory Overview

**Control** is how a robot turns what it wants (a joint angle, a path, a grasp) into what it does (motor currents), and keeps doing the right thing when the world pushes back. This page introduces the core idea and maps the notes in this chapter.

> **Etymology.** *Control* is from Medieval Latin *contrarotulus*, "a counter-roll": a duplicate register used to check accounts. To *control* was first to check against a reference, which is still exactly what a feedback controller does. *Cybernetics* (Wiener, 1948) is from Greek *kybernētēs*, "steersman".

## The feedback loop

```
reference r ──►(+)── error e ──► controller ──► u ──► robot (plant) ──► output y
               ▲ −                                                         │
               └──────────────────── sensor ◄──────────────────────────────┘
```

The controller compares what is measured with what is wanted and acts on the difference. Feedback lets an imperfect model and an unpredictable world still produce accurate motion (see [[Control_Theory_and_Principles]]).

## Layers of control on a 2026 robot

| Layer | Typical rate | Method | Example |
|---|---|---|---|
| Motor current | 10–40 kHz | PI current control in the motor driver | Torque from each motor |
| Joint position or torque | 1 kHz | PID, impedance control | [[ros2_control]] joint controllers |
| Whole body / trajectory | 100–1,000 Hz | Computed torque, operational-space control, [[Model_Predictive_Control_(MPC)\|MPC]] | Humanoid balance, arm tracking |
| Learned policy | 10–500 Hz | Neural network trained with RL or imitation | Locomotion, manipulation skills |
| Planning and reasoning | 0.1–10 Hz | Planners, vision-language models | Paths, task sequences |

Each layer treats the one below it as an actuator and the one above as its source of goals. Safety limits (torque, speed, workspace) are enforced in the low layers, where they are fast and simple to verify.

## Map of this chapter

- **Foundations**: [[Control_Systems]] (building a controller), [[Control_Theory]] (mathematics: models, stability, response), [[Feedback_Control]], [[State_Space_Representation]], [[Transfer_Function]].
- **Classical**: [[PID_Control]].
- **Modern and model-based**: [[Computed_Torque_Control]], [[Adaptive_Control]], [[Robust_Control]], [[Model_Predictive_Control_(MPC)]].
- **Interaction**: [[Compliance]], impedance and force control.
- **Navigation and motion**: [[Autonomous_Navigation]], [[Collision_Avoidance]].
- **People**: [[Human-Robot_Interaction]].
- **Learning**: [[Neural_Networks_in_Control]], [[Reinforcement_Learning_for_Robots]].

## Where to start

[[Feedback_Control]] → [[PID_Control]] → [[Control_Theory]] → [[State_Space_Representation]] → [[Computed_Torque_Control]] → [[Model_Predictive_Control_(MPC)]]

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Control_Theory_Overview]])
```
