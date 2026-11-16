---
title: Manipulator Arm
description: "A manipulator arm is a chain of links and joints that positions a tool or hand in space. Covers the anatomy (joints, links, wrist, end effector), degrees of freedom and why six or seven, the main kinematic types, the specifications that matter (payload, reach, repeatability, speed), a computed shoulder-torque example, and how arms are controlled and taught in 2026, from teach pendants to learned policies."
aliases:
  - Robot Arm
  - Robotic Arm
  - Manipulator
  - Serial Manipulator
tags:
  - manipulator-arm
  - manipulation
  - kinematics
  - actuators
type: robot-type
level: beginner
subfield:
  - manipulation
  - kinematics-dynamics
status: keystone
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Robots]]"
  - "[[Degrees_of_Freedom]]"
related:
  - "[[Industrial_Arms]]"
  - "[[Forward_Kinematics]]"
  - "[[Inverse_Kinematics]]"
  - "[[Robot_Hands]]"
  - "[[MoveIt2]]"
  - "[[ros2_control]]"
key_takeaways:
  - "Six joints are the minimum to place a tool at any position and orientation in its workspace; a seventh adds redundancy to avoid obstacles, joint limits and singularities."
  - "Payload, reach, repeatability and speed are the headline specifications, and they interact: the full payload is rated at the flange, and torque grows with distance from each joint."
  - "Arms are programmed by teach pendant, offline simulation, motion planners (MoveIt, cuRobo) and, increasingly, learned policies trained from demonstrations."
sources:
  - "Lynch, K. M. and Park, F. C. (2017). Modern Robotics: Mechanics, Planning, and Control. https://hades.mech.northwestern.edu/index.php/Modern_Robotics"
  - "Siciliano, B. and Khatib, O., eds. (2016). Springer Handbook of Robotics, 2nd edition. https://link.springer.com/referencework/10.1007/978-3-319-32552-1"
  - "International Federation of Robotics (2026). Five Million Robots now Operate in Factories Globally (World Robotics 2026). https://ifr.org/ifr-press-releases/news/five-million-robots-now-operate-in-factories-globally"
  - "Sundaralingam, B. et al. (2023). cuRobo: Parallelized Collision-Free Minimum-Jerk Robot Motion Generation. https://arxiv.org/abs/2310.17274"
  - "Zhao, T. et al. (2023). Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware. https://arxiv.org/abs/2304.13705"
---

# Manipulator Arm

A **manipulator arm** is a series of rigid **links** connected by motorised **joints**, ending in a tool or hand (the **end effector**). Its job is to put that tool in the right place, at the right angle, at the right time. Robot arms weld cars, assemble phones, pick parcels, assist surgeons and, increasingly, learn household tasks.

> **Etymology.** Latin *manipulus*, "a handful" (from *manus*, "hand"): to manipulate is to work with the hands. The first industrial arm, the Unimate, went to work at a General Motors plant in 1961.

## Anatomy

| Part | Role |
|---|---|
| **Base** | Fixed to the floor, a table or a mobile platform |
| **Joints** | Revolute (rotate) or prismatic (slide), each driven by a motor and gearbox, with an encoder |
| **Links** | Rigid segments between joints |
| **Wrist** | The last 2–3 joints, which mostly set orientation |
| **Flange and end effector** | Mounting plate, and the gripper, suction cup, welding torch or hand fitted to it |

## Why six or seven joints

A rigid object in space has six [[Degrees_of_Freedom|degrees of freedom]]: three for position and three for orientation. An arm therefore needs at least **six** independent joints to reach arbitrary poses in its workspace. A **seventh** joint makes it **redundant**: the same hand pose can be reached with the elbow in different places. That helps it reach around obstacles, stay away from joint limits and singular configurations, and is why many research and collaborative arms (and the human arm) have seven.

## Kinematic types

| Type | Joints | Typical use |
|---|---|---|
| **Articulated (anthropomorphic)** | 6–7 revolute | General purpose: welding, assembly, cobots |
| **SCARA** | 2 revolute + 1 prismatic + wrist rotation | Fast pick-and-place on a plane, electronics |
| **Cartesian (gantry)** | 3 prismatic | Large workspaces, 3D printers, CNC-like tasks |
| **Delta (parallel)** | Three parallel arms | Very fast light picking (food, packaging); see [[Parallel_Robots]] |

## Specifications that matter

| Spec | Meaning | Typical range |
|---|---|---|
| **Payload** | Mass the arm can carry at the flange, including the gripper | 0.5 kg (small cobot) to 2,300 kg (heavy industrial) |
| **Reach** | Maximum distance from base axis to flange | 0.3–4 m |
| **Repeatability** | How closely it returns to a taught point | ±0.01–0.1 mm |
| **Accuracy** | How closely it reaches a *commanded* point (needs calibration) | Often 10× worse than repeatability |
| **Speed** | Joint speeds, or cycle time on a standard test motion | — |

## Worked example: why payload depends on reach

A 5 kg payload held at a horizontal reach of 0.85 m exerts a static torque at the shoulder of

$$\tau = m g r = 5 \times 9.81 \times 0.85 = 41.7\ \text{N·m}.$$

The arm's own links add more. Say 10 kg acting at an effective 0.4 m gives $10 \times 9.81 \times 0.4 = 39.2$ N·m, for a total of about **80.9 N·m** before any acceleration.

With a 100:1 gearbox the motor must supply $80.9 / 100 = 0.81$ N·m, or about 1.0 N·m at 80% gearbox efficiency, just to hold still. Accelerating needs more. This is why rated payload falls when the load's centre of mass sits further from the flange, and why vendors publish **payload diagrams** rather than one number.

## Programming and control (2026)

| Method | How | Good for |
|---|---|---|
| **Teach pendant** | Jog to points and record them | Simple, repeated industrial tasks |
| **Hand guiding** | Physically move a collaborative arm and record | Quick setup by non-experts |
| **Offline programming** | Program and verify in simulation (digital twin) | Complex cells, no production downtime |
| **Motion planning** | Give goals; a planner finds collision-free paths ([[MoveIt2]], cuRobo) | Changing scenes, perception-driven picking |
| **Learned policies** | Train from teleoperated demonstrations or RL ([[Imitation_Learning]], [[Vision-Language-Action_Models]]) | Contact-rich, variable tasks (folding, assembly with variation) |

Under all of these, a joint-level controller (PID or impedance control, often via [[ros2_control]]) runs at 500 Hz–1 kHz.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Manipulator_Arm]])
```
