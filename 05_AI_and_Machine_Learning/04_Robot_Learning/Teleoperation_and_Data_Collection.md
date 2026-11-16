---
title: Teleoperation and Data Collection
description: "How demonstrations for robot learning are recorded: leader-follower arms, VR and hand tracking, handheld grippers, and human video. The interface decides how fast, cheap and faithful the data is, and data is now the main bottleneck in robot learning."
aliases:
  - Teleoperation
  - Robot Teleoperation
  - Data Collection for Robot Learning
tags:
  - teleoperation
  - imitation-learning
  - robot-learning
  - dataset
  - manipulation
type: method
level: intermediate
subfield:
  - robot-learning
  - manipulation
  - human-robot-interaction
status: reviewed
author: Jordan_Smith
date: 2026-11-15
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Imitation_Learning]]"
related:
  - "[[Robot_Learning_Datasets_and_Benchmarks]]"
  - "[[Vision-Language-Action_Models]]"
  - "[[Human-Robot_Interaction]]"
  - "[[Robot_Hands]]"
key_takeaways:
  - "Imitation learning is limited by data, and data is limited by the teleoperation interface."
  - "Leader-follower arms give precise, cheap bimanual data; VR and hand tracking scale to humanoids; handheld grippers collect data with no robot at all."
  - "The largest 2026 models mix robot teleoperation with handheld-gripper data and thousands of hours of human video."
sources:
  - "Zhao, T. et al. (2023). Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware. https://arxiv.org/abs/2304.13705"
  - "Fu, Z., Zhao, T. and Finn, C. (2024). Mobile ALOHA: Learning Bimanual Mobile Manipulation with Low-Cost Whole-Body Teleoperation. https://arxiv.org/abs/2401.02117"
  - "Wu, P. et al. (2023). GELLO: A General, Low-Cost, and Intuitive Teleoperation Framework for Robot Manipulators. https://arxiv.org/abs/2309.13037"
  - "Chi, C. et al. (2024). Universal Manipulation Interface: In-The-Wild Robot Teaching Without In-The-Wild Robots. https://arxiv.org/abs/2402.10329"
  - "Cheng, X. et al. (2024). Open-TeleVision: Teleoperation with Immersive Active Visual Feedback. https://arxiv.org/abs/2407.01512"
  - "NVIDIA (2026). GR00T 2: Scaling Robot Foundation Models for Generalization, Dexterity, and Adaptation. https://research.nvidia.com/labs/gear/gr00t-2/"
---

# Teleoperation and Data Collection

**Teleoperation** means controlling a robot remotely in real time. In robot learning it has a second job: every teleoperated episode is a recorded demonstration for [[Imitation_Learning]]. Because modern robot learning is limited more by data than by algorithms, the design of the teleoperation interface (how fast, cheap and faithful it is) shapes what robots can learn.

> **Etymology.** *Tele-* is Greek for "far off" (as in telephone, television); *operation* is from Latin *operari*, "to work". The term came from 1940s–50s nuclear laboratories, where master-slave manipulators let technicians handle radioactive material from behind shielding walls. Raymond Goertz's mechanical leader-follower arms at Argonne National Laboratory (1940s) are the direct ancestors of today's ALOHA rigs. Modern work says *leader-follower* rather than *master-slave*.

## The main interfaces

| Interface | How it works | Strengths | Limits | Example |
|---|---|---|---|---|
| **Leader-follower arms** | Operator moves a lightweight copy of the arm; the robot mirrors its joint angles | Intuitive, precise, cheap, bimanual | Must match the robot's kinematics; no force feedback on low-cost rigs | ALOHA (Zhao et al. 2023), GELLO (Wu et al. 2023) |
| **VR / hand tracking** | Headset tracks head and hands; robot head cameras stream back to the headset | Natural for humanoids and dexterous hands | Latency; retargeting human hands to robot hands is lossy | Open-TeleVision (Cheng et al. 2024) |
| **Handheld gripper** | Person uses a camera-equipped gripper by hand; actions recovered from camera tracking | No robot needed; collect anywhere | Needs a matching gripper on the robot; no proprioception | UMI (Chi et al. 2024) |
| **Whole-body rigs** | Leader arms on a mobile base, or full-body motion capture | Mobile and whole-body tasks | Expensive, tiring | Mobile ALOHA (Fu et al. 2024) |
| **Human video** | Egocentric video of people working, no robot | Enormous scale | No action labels; hands differ from grippers | Used in pretraining the largest 2026 models |
| **Spacemouse / keyboard** | 6-DoF input device controls the end effector | Cheapest | Slow, unnatural, poor for dexterity | Common in early datasets |

## What makes a good demonstration

- **Consistency** matters more than speed: a policy imitates hesitations and odd strategies too.
- **Diversity** of object positions, lighting, backgrounds and operators is what produces generalisation.
- **Recoveries** are valuable: deliberately include fixing small mistakes, so the policy learns what to do off the ideal path.
- **Synchronised recording**: camera frames, joint states and actions on one clock, at the control rate (often 30–50 Hz).
- **Language labels** for each episode or segment make the data usable for language-conditioned models.

## Worked example: the cost of data

Suppose a skilled operator records one 20-second demonstration every 60 seconds, including reset time. A working day of 6 hours yields $6 \times 60 = 360$ demonstrations, or two hours of robot data. The ACT tasks needed about 50 demonstrations each, so a new single task costs under an hour. A generalist model's pretraining mix is measured in **tens of thousands of hours**: tens of thousands of operator-days. This arithmetic is why handheld grippers and human video, which need no robot, have become so important.

## Scale in 2026

NVIDIA's GR00T 2 research page (November 2026) describes its pretraining mix as about 402,730 hours of egocentric human video, 10,184 hours of handheld UMI-style gripper demonstrations and 23,500 hours of robot teleoperation. These are vendor figures, but the proportions show the trend: robot teleoperation is the smallest and most expensive ingredient, and is used to ground the much larger human data.

## Safety and ergonomics

Teleoperators can be injured by the robot they control, and long sessions are physically tiring. Follower robots should run with torque and velocity limits, and leader devices should be light. Collecting data in homes and workplaces also raises privacy questions about what cameras record.

## Reading list

- Zhao et al. 2023 (ALOHA): the low-cost leader-follower rig that triggered the imitation-learning wave.
- Chi et al. 2024 (UMI): collecting data without a robot.
- Cheng et al. 2024 (Open-TeleVision): immersive teleoperation for humanoids.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Teleoperation_and_Data_Collection]])
```
