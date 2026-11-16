---
title: Autonomous Navigation
description: "Autonomous navigation gets a robot from where it is to where it should be, safely and without a driver: localization, mapping, planning and control working together. Covers the classical pipeline, a computed localization-accuracy budget for sidewalk delivery, GNSS and RTK, semantic and social navigation, and the learned navigation foundation models (ViNT, NoMaD) of 2023–2026."
aliases:
  - Robot Navigation
  - Navigation
tags:
  - navigation
  - mobile-robot
  - path-planning
  - autonomous-systems
type: concept
level: intermediate
subfield:
  - mobile-robots
  - planning
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Mobile_Robots]]"
  - "[[Localization]]"
related:
  - "[[Nav2_Navigation]]"
  - "[[SLAM]]"
  - "[[Collision_Avoidance]]"
  - "[[Path_Planning_Algorithms]]"
  - "[[Autonomous_Robots]]"
key_takeaways:
  - "Navigation combines four capabilities in a loop: localize, map, plan and control; failures in any one stop the robot."
  - "Accuracy needs follow from the environment: staying within a 1.5 m sidewalk needs lateral error well under 0.3 m, beyond standalone GNSS (metres) but within RTK (centimetres) or lidar/visual map matching."
  - "Learned navigation foundation models (ViNT, NoMaD, 2023) navigate from camera images across many robots; production systems still wrap them in, or prefer, the classical map-based stack."
sources:
  - "Siegwart, R., Nourbakhsh, I. R. and Scaramuzza, D. (2011). Introduction to Autonomous Mobile Robots, 2nd edition. https://mitpress.mit.edu/9780262015356/introduction-to-autonomous-mobile-robots/"
  - "Macenski, S., Moore, T., Lu, D. V., Merzlyakov, A. and Ferguson, M. (2023). From the desks of ROS maintainers: A survey of modern & capable mobile robotics algorithms in the robot operating system 2. https://arxiv.org/abs/2307.15236"
  - "Shah, D. et al. (2023). ViNT: A Foundation Model for Visual Navigation. https://arxiv.org/abs/2306.14846"
  - "Sridhar, A., Shah, D., Glossop, C. and Levine, S. (2023). NoMaD: Goal Masked Diffusion Policies for Navigation and Exploration. https://arxiv.org/abs/2310.07896"
---

# Autonomous Navigation

**Autonomous navigation** is a robot's ability to travel to a goal through its environment without a human steering: a vacuum crossing a living room, a forklift moving pallets, a delivery robot crossing a campus, a rover crossing Mars. It is the most deployed form of robot autonomy.

> **Etymology.** Latin *navigare*, "to sail", from *navis*, "ship". The questions of navigation (where am I, where am I going, how do I get there) are the same for sailors and robots.

## The loop

| Question | Capability | Typical methods |
|---|---|---|
| Where am I? | **Localization** | Wheel odometry + IMU; lidar or visual matching to a map; GNSS outdoors (see [[Localization]]) |
| What is around me? | **Mapping and perception** | Occupancy grids, SLAM, obstacle detection, semantic segmentation (see [[SLAM]]) |
| How do I get there? | **Planning** | Global path search, local trajectory optimisation (see [[Path_Planning_Algorithms]]) |
| How do I follow the plan? | **Control** | Path-tracking controllers (pure pursuit, MPC, MPPI) |
| What if something is in the way? | **Avoidance and recovery** | Costmaps, [[Collision_Avoidance]], recovery behaviours |

On ROS 2 robots these are packaged in [[Nav2_Navigation|Nav2]].

## Worked example: how accurate must localization be?

A delivery robot 0.6 m wide drives on a sidewalk 1.5 m wide. It has $(1.5 - 0.6)/2 = 0.45$ m of space on each side if centred. Keeping a margin for control error and obstacles, its lateral position error should stay below about **0.3 m**.

| Positioning source | Typical accuracy | Good enough? |
|---|---|---|
| Standalone GNSS (phone-grade) | 2–5 m, worse near buildings | No |
| RTK GNSS | about 2 cm in open sky | Yes, where the sky is visible |
| Lidar or visual localization against a prior map | 5–10 cm | Yes, if the map is up to date |
| Wheel odometry alone | Drifts ~1–2% of distance | Only over short stretches |

That is why urban robots fuse several sources and fall back to map-based localization in "urban canyons" where GNSS signals bounce.

## Beyond geometry

- **Semantic navigation**: "go to the kitchen", using object and room labels from vision-language models and scene graphs (see [[Knowledge_Representation_and_Reasoning]]).
- **Social navigation**: moving among people, keeping comfortable distances, passing on the expected side, and making intentions legible.
- **Terrain-aware navigation**: learned traversability (grass versus mud versus pavement) for field robots.

## Learned navigation (2023–2026)

Navigation **foundation models** learn from many hours of driving data across many robots:
- **ViNT** (2023) navigates to image goals with a transformer trained on diverse robot datasets, and adapts to new robots;
- **NoMaD** (2023) uses a diffusion policy both to explore and to reach goals.

These models handle visual, map-free navigation remarkably well, but the production stacks of warehouse, delivery and service robots in 2026 remain mostly classical and map-based, with learned components for perception, traversability and occasionally local planning. That is because the classical stack is easier to verify, debug and certify.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Autonomous_Navigation]])
```
