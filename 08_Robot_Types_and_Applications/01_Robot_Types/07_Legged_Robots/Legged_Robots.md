---
title: Legged Robots
description: Describes legged robots, their classification based on leg count, advantages, challenges, stability considerations, gaits, and examples.
tags:
  - robot-types
  - legged-robot
  - locomotion
  - bipedal
  - quadruped
  - hexapod
  - stability
  - gait
  - mobile-robot
layout: default
category: robotics
author: Jordan_Smith_&_le_Chat
date: 2025-04-29
permalink: /legged_robots/
related:
  - "[[Robots]]"
  - "[[Mobile_Robots]]"
  - "[[Locomotion]]"
  - "[[Bipedal_Locomotion]]"
  - "[[Humanoid_Robots]]"
  - "[[Quadruped_Robot]]"
  - "[[Hexapod_Robot]]"
  - "[[Stability]]"
  - "[[Zero_Moment_Point_(ZMP)]]"
  - "[[Gait]]"
  - "[[Control_Theory]]"
  - "[[Kinematics]]"
  - "[[Dynamics]]"
  - "[[Robot_Types_and_Applications]]"
---

# Legged Robots

**Legged Robots** are a class of [[Mobile_Robots]] that utilize articulated limbs, analogous to biological legs, for [[Locomotion]]. They move by creating sequences of discrete contact points with the environment, offering unique advantages, particularly in navigating complex and unstructured terrain. Many designs are bio-inspired, mimicking the movement strategies of animals and humans.

---

## Classification by Leg Count

Legged robots are often categorized by their number of legs:

* **Monopods (1 Leg):** Hopping robots that require continuous dynamic balance. Examples include early experimental hoppers like the Raibert Hopper.
* **[[Bipedal Locomotion|Bipeds (2 Legs)]]:** Robots that walk or run on two legs, often resembling humans ([[Humanoid_Robots]] like ASIMO, Atlas) or birds (e.g., Cassie). Primarily rely on dynamic stability. See [[Bipedal Locomotion]].
* **Tripods (3 Legs):** Can achieve static stability if their center of gravity remains within the support triangle formed by their feet. Less common than other configurations.
* **Quadrupeds (4 Legs):** Four-legged robots, often animal-like (e.g., dogs/cheetahs). Can utilize statically stable gaits (like a tripod gait where only one leg moves at a time) or highly dynamic gaits (trot, bound, gallop). Well-known examples include Boston Dynamics' BigDog and Spot, and ANYbotics' ANYmal. See [[Quadruped Robot]].
* **Hexapods (6 Legs):** Insect-like robots with six legs. Offer excellent static stability, as they can always maintain a stable tripod of support while moving the other three legs (tripod gait). Well-suited for slow, careful movement over very rough terrain. Examples include Brooks' Genghis, RHex, and the Lauron series. See [[Hexapod Robot]].
* **Multipedal (>6 Legs):** Robots with numerous legs, sometimes designed for specific tasks like climbing complex structures or inspired by creatures like spiders or centipedes.

---

## Advantages

Compared to [[Wheeled_Mobile_Robots]] or [[Tracked Robots]], legged robots offer:

* **Superior Terrain Adaptability:** Can potentially traverse highly uneven ground, step over obstacles, cross gaps, and climb stairs – environments inaccessible to most wheeled or tracked systems.
* **Discrete Footprints:** Minimal ground contact area can reduce environmental impact and allow movement across surfaces unsuitable for continuous contact (e.g., stepping stones).
* **Maneuverability:** Potential for high agility, including turning in place and complex maneuvering within cluttered spaces.
* **Active Suspension:** The articulated legs naturally provide active suspension capabilities.
* **Manipulation Potential:** Legs can sometimes be used for basic manipulation tasks in addition to locomotion.

---

## Challenges

Despite their advantages, legged robots present significant challenges:

* **Control Complexity:** Managing the high [[Degrees_of_Freedom]] of multiple articulated limbs requires sophisticated [[Control Systems]] for coordination, balance, and [[Gait]] generation. Dynamic stability is particularly difficult.
* **[[Stability]]:** Maintaining balance is a primary challenge.
    * **Static Stability:** Possible for robots with 3+ legs forming a sufficiently wide support base relative to the center of gravity. Requires slow, deliberate gaits for robots with fewer than 6 legs.
    * **Dynamic Stability:** Required for bipeds, quadrupeds (in faster gaits), and monopods. Needs active control based on sensor feedback ([[IMU_Sensors]], joint encoders, foot contact sensors) to prevent falling. Concepts like the [[Zero Moment Point (ZMP)]] are crucial for analyzing and controlling dynamic balance in walking robots.
* **Mechanical Complexity:** Leg mechanisms with multiple joints and [[Actuator|actuators]] are inherently more complex and potentially less robust than simpler wheeled systems.
* **Energy Efficiency:** Generally lower energy efficiency compared to wheels on flat, hard surfaces due to the energy cost of lifting/swinging limbs and actively maintaining balance. Efficiency comparisons depend heavily on the terrain.
* **Payload and Speed:** Often have lower payload capacity relative to their weight and potentially lower top speeds on flat ground compared to optimized wheeled systems.

---

## [[Gait]]

A **[[Gait]]** refers to the specific pattern and sequence of leg movements and ground contacts used for locomotion. Gaits determine the robot's speed, stability, and efficiency.
* **Static Gaits:** Maintain static stability throughout the movement cycle (e.g., tripod gait for hexapods, crawl gait for quadrupeds). Typically slower.
* **Dynamic Gaits:** Rely on motion and active control to maintain balance (e.g., walking/running for bipeds; trot, bound, gallop for quadrupeds). Enable higher speeds but require more complex control.

---

Legged robots represent a vibrant area of research, pushing the boundaries of mechanical design, dynamic control, and bio-inspiration to create machines capable of navigating the complex, unstructured environments common outside of highly engineered settings.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #robotics OR #automation WHERE contains(file.outlinks, [[Robots]])