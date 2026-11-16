---
title: Mobile Robots
description: "Mobile robots move through their environment rather than staying bolted in place: wheeled, tracked, legged, flying and swimming. Covers the locomotion options, the navigation stack (perception, localization, mapping, planning, control), a computed stopping-distance example that sizes a safety field, where mobile robots work in 2026 (warehouses, delivery, robotaxis, inspection, homes), and what remains hard."
aliases:
  - Mobile Robot
  - AMR
  - Autonomous Mobile Robots
tags:
  - mobile-robot
  - navigation
  - locomotion
  - autonomous-systems
type: robot-type
level: beginner
subfield:
  - mobile-robots
status: keystone
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Robots]]"
related:
  - "[[Wheeled_Mobile_Robots]]"
  - "[[Legged_Robots]]"
  - "[[Drones]]"
  - "[[Nav2_Navigation]]"
  - "[[SLAM]]"
  - "[[Localization]]"
  - "[[Autonomous_Robots]]"
key_takeaways:
  - "Choose locomotion by terrain: wheels for flat floors (efficient, simple), tracks for soft ground, legs for stairs and rubble, rotors for air, propellers or thrusters for water."
  - "Every autonomous mobile robot answers four questions continuously: where am I (localization), what is around me (perception and mapping), where should I go (planning), how do I get there (control)."
  - "Warehouse robots (Amazon passed one million in 2025) and robotaxis (Waymo about 400,000 paid rides per week in early 2026) are the largest deployments; mobile manipulation and humanoids are the frontier."
sources:
  - "Siegwart, R., Nourbakhsh, I. R. and Scaramuzza, D. (2011). Introduction to Autonomous Mobile Robots, 2nd edition. https://mitpress.mit.edu/9780262015356/introduction-to-autonomous-mobile-robots/"
  - "Thrun, S., Burgard, W. and Fox, D. (2005). Probabilistic Robotics. https://mitpress.mit.edu/9780262201629/probabilistic-robotics/"
  - "Macenski, S., Moore, T., Lu, D. V., Merzlyakov, A. and Ferguson, M. (2023). From the desks of ROS maintainers: A survey of modern & capable mobile robotics algorithms in the robot operating system 2. https://arxiv.org/abs/2307.15236"
  - "TechCrunch (2025). Amazon deploys its one millionth robot, releases generative AI model. https://techcrunch.com/2025/07/01/amazon-deploys-its-one-millionth-robot-releases-generative-ai-model/"
  - "Electrek (2026). Waymo begins fully autonomous ops with 6th-gen Driver, targets 1M weekly rides. https://electrek.co/2026/02/12/waymo-begins-fully-autonomous-ops-with-6th-gen-driver-targets-1m-weekly-rides/"
---

# Mobile Robots

A **mobile robot** can move itself from place to place. A factory arm stays where it is bolted; a mobile robot carries its sensors, computer and tools through warehouses, streets, homes, fields, skies and oceans. Mobility multiplies what a robot can do and what can go wrong: the world keeps changing around it.

> **Etymology.** Latin *mobilis*, "movable", from *movere*, "to move". Early milestones: Grey Walter's tortoises (1948–49), SRI's Shakey (1966–72), which first combined perception, planning and action, and the DARPA Grand Challenges (2004–07), which launched self-driving cars.

## Ways to move

| Locomotion | Best on | Trade-offs | See |
|---|---|---|---|
| **Wheels** | Flat, hard floors and roads | Efficient and simple; stopped by steps | [[Wheeled_Mobile_Robots]], [[Omnidirectional_Robots]] |
| **Tracks** | Soft ground, rubble, slopes | Robust; inefficient when turning, damages floors | — |
| **Legs** | Stairs, rough terrain, gaps | Versatile; complex and energy-hungry | [[Legged_Robots]], [[Humanoid_Robots]] |
| **Rotors / wings** | Air | Fast, reach anywhere; short endurance | [[Drones]] |
| **Thrusters / propellers** | Water | Large range underwater; communication is hard | [[Underwater_and_Space_Robots]] |

## The navigation stack

```
sensors → perception → localization + mapping (SLAM) → planning (global path, local trajectory) → control → motors
                                     ↑                                            |
                                     └──────────────── feedback ──────────────────┘
```

- **Localization**: where am I? (wheel odometry, IMU, lidar or visual matching to a map, GNSS outdoors). See [[Localization]].
- **Mapping and SLAM**: what does the world look like? See [[SLAM]].
- **Planning**: a global path on the map, a local trajectory around nearby obstacles. See [[Path_Planning_Algorithms]].
- **Control**: track the trajectory with the drive system.

On ROS 2 robots, [[Nav2_Navigation|Nav2]] packages this stack.

## Worked example: sizing a safety field

A warehouse robot drives at 1.5 m/s and can brake at 1.0 m/s². Its safety lidar and controller take 0.2 s to react. Its stopping distance is

$$d = v\,t_{react} + \frac{v^2}{2a} = 1.5 \times 0.2 + \frac{1.5^2}{2 \times 1.0} = 0.30 + 1.125 = 1.425\ \text{m}.$$

The protective field in front of the robot must be at least 1.43 m long (plus a margin for measurement error) at this speed. At 0.5 m/s it shrinks to $0.1 + 0.125 = 0.225$ m. That is why mobile robots switch field sets with speed, and why they slow down in aisles where people walk: braking distance grows with the *square* of speed.

## Where they work (2026)

| Domain | Examples |
|---|---|
| **Warehouses and factories** | Goods-to-person robots, autonomous forklifts, tuggers. Amazon reported its one-millionth robot in July 2025, with a generative model (DeepFleet) coordinating fleet traffic |
| **Roads** | Robotaxis. Waymo gave about 400,000 paid rides per week in February 2026, after 15 million rides in 2025, with a stated goal of over one million per week by the end of 2026 |
| **Sidewalks and buildings** | Delivery robots, hospital and hotel delivery, cleaning robots |
| **Inspection** | Legged and tracked robots in plants, tunnels and construction sites |
| **Homes** | Robot vacuums and mops; home humanoids in early sales |
| **Fields, air and sea** | [[Agricultural_Robots]], [[Drones]], [[Underwater_and_Space_Robots]] |

## What is still hard

- **Crowds and social navigation**: moving efficiently without startling people.
- **Long-term change**: furniture moves, seasons change, maps go stale.
- **Rare events**: the "long tail" of unusual situations on roads and in public spaces.
- **Mobile manipulation**: navigating *and* doing useful work with arms at the destination.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Mobile_Robots]])
```
