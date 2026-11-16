---
title: Robot Hands
description: "Robot hands and grippers are where a robot touches the world. Covers the spectrum from two-finger and suction grippers to multi-fingered dexterous hands, a computed grip-force example with friction, tactile sensing (vision-based fingertips such as GelSight and DIGIT), how dexterity is learned in 2026, and why dexterous manipulation remains one of robotics' hardest open problems."
aliases:
  - Robot Hand
  - Grippers
  - Gripper
  - Dexterous Hands
  - End-Effector
tags:
  - end-effector
  - grasping
  - manipulation
  - tactile-sensing
  - robot-learning
type: component
level: intermediate
subfield:
  - manipulation
  - mechanisms-actuation
status: keystone
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Manipulator_Arm]]"
related:
  - "[[Humanoid_Robots]]"
  - "[[Manipulation]]"
  - "[[Teleoperation_and_Data_Collection]]"
  - "[[Reinforcement_Learning_for_Robots]]"
  - "[[Sim-to-Real_Transfer]]"
key_takeaways:
  - "Most industrial work is done by simple parallel-jaw and suction grippers; dexterous multi-fingered hands matter for tools, in-hand reorientation and the human-shaped world."
  - "Grip depends on friction: holding an object needs a normal force of at least its weight divided by twice the friction coefficient (for a two-finger grip), so slippery objects need much more force or a better grasp."
  - "Dexterity is learned with RL in simulation (OpenAI's Dactyl, 2018, to humanoid hands in 2025) and from teleoperation and human video; touch sensing is improving fast, but robust general dexterity is still unsolved."
sources:
  - "OpenAI et al. (2018). Learning Dexterous In-Hand Manipulation. https://arxiv.org/abs/1808.00177"
  - "Lambeta, M. et al. (2020). DIGIT: A Novel Design for a Low-Cost Compact High-Resolution Tactile Sensor with Application to In-Hand Manipulation. https://arxiv.org/abs/2005.14679"
  - "Lambeta, M. et al. (2024). Digitizing Touch with an Artificial Multimodal Fingertip. https://arxiv.org/abs/2411.02479"
  - "Shaw, K., Agarwal, A. and Pathak, D. (2023). LEAP Hand: Low-Cost, Efficient, and Anthropomorphic Hand for Robot Learning. https://arxiv.org/abs/2309.06440"
  - "Lin, T. et al. (2025). Sim-to-Real Reinforcement Learning for Vision-Based Dexterous Manipulation on Humanoids. https://arxiv.org/abs/2502.20396"
  - "Boston Dynamics (2026). Robot Hands for Modern AI and Real Work. https://bostondynamics.com/blog/robot-hands-for-modern-ai-and-real-work/"
---

# Robot Hands

The **hand** (or **end effector**) is the part of a robot that touches the world. It decides what the robot can pick up, how securely, and what it can do with an object once it holds it. Choosing or designing it is often more important than choosing the arm.

> **Etymology.** *Gripper* is from Old English *gripan*, "to seize". *Dexterous* is from Latin *dexter*, "right-handed, skilful". *End effector* is engineering jargon for whatever acts at the end of the arm.

## The spectrum

| Type | How it holds | Strengths | Limits |
|---|---|---|---|
| **Parallel-jaw gripper** | Two fingers close on the object | Simple, robust, cheap; the industrial default | Limited to graspable shapes; no in-hand manipulation |
| **Suction** | Vacuum cups | Fast for boxes and flat items; dominant in parcel handling | Needs sealable surfaces; porous or tiny items fail |
| **Adaptive / underactuated** | Fingers that wrap around shapes with few motors | Grasps varied shapes with simple control | Little precise control of finger pose |
| **Soft grippers** | Compliant fingers (pneumatic, fin-ray) | Gentle on food and fragile items | Lower force and precision; see [[Soft_Robotics]] |
| **Dexterous hands** | 3–5 fingers, 12–24 actuated joints | Tool use, reorientation in the hand, human-like tasks | Costly, fragile, hard to control |

Examples of dexterous hands include the Shadow Hand (research standard for two decades), the Allegro Hand, the low-cost LEAP Hand (2023) designed for robot learning, and the hands on 2025–26 humanoids. 1X's NEO, for instance, lists 22 degrees of freedom per hand.

## Worked example: how hard must it squeeze?

A two-finger grip holds an object by friction. Each finger presses with normal force $N$; friction can supply up to $\mu N$ at each contact. To hold mass $m$ against gravity:

$$2\mu N \ge m g \quad\Rightarrow\quad N \ge \frac{m g}{2\mu}$$

| Object (1 kg) | Friction coefficient μ | Minimum force per finger | With safety factor 2 |
|---|---|---|---|
| Rubber-coated tool | 0.5 | 9.8 N | 19.6 N |
| Wet glass | 0.3 | 16.4 N | 32.7 N |

Any acceleration of the arm adds to the required force. Squeezing too hard crushes fragile items, so good grasps balance the two. This is why **tactile sensing** (feeling incipient slip) and wider, softer fingertips (more friction, area and tolerance) matter so much.

## Touch

Vision-based tactile sensors put a camera behind a soft gel skin. Contact deforms the gel, and the camera sees the contact shape, forces and slip at high resolution:
- **GelSight**, from MIT;
- **DIGIT** (Meta, 2020), a low-cost version;
- **Digit 360** (2024), adding multimodal sensing.

Self-supervised touch models (Sparsh, 2024) learn representations from such sensors the way vision models learn from images. Tactile data is becoming an input to learned manipulation policies.

## Learning dexterity

| Approach | Milestone |
|---|---|
| **RL in simulation with domain randomisation** | OpenAI's Dactyl (2018) rotated a cube in a Shadow Hand after training in simulation; 2025 work applied sim-to-real RL to vision-based dexterous tasks on humanoid hands |
| **Teleoperation** | Data gloves, VR hand tracking and exoskeletons map human finger motion to robot hands for demonstrations (see [[Teleoperation_and_Data_Collection]]) |
| **Human video** | Hand-pose estimation from video retargeted to robot hands, a large and cheap data source |
| **Grasp datasets** | Simulation-generated grasp sets for dexterous hands (DexGraspNet, 2022) |
| **Foundation models** | VLAs controlling multi-fingered hands, though leading labs still describe dexterous manipulation as challenging |

## Open problems (2026)

- **Durability and cost.** Many small actuators and tendons wear out; hands are often the least reliable part of a humanoid.
- **Sensing coverage.** Fingertips are sensorised; palms and finger sides often are not.
- **Generalisation.** Policies that reorient one object rarely transfer to arbitrary objects and tools.
- **Contact-rich precision.** Inserting, threading and deformable objects (cables, cloth) remain hard.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Robot_Hands]])
```
