---
title: Mobile Robots
description: An overview of mobile robots, which are capable of locomotion through various environments, covering types, components, capabilities, and applications.
tags:
  - robot-types
  - mobile-robot
  - locomotion
  - navigation
  - wheeled-robot
  - legged-robot
  - AMR
  - AUV
  - rover
  - drone
layout: default
category: robotics
author: Jordan_Smith_&_le_Chat
date: 2025-04-28
permalink: /mobile_robots/
related:
  - "[[Robots]]"
  - "[[Locomotion]]"
  - "[[Wheeled_Mobile_Robots]]"
  - "[[Legged_Robots]]"
  - "[[Humanoid_Robots]]"
  - "[[Drones]]"
  - "[[Underwater_and_Space_Robots]]"
  - "[[Navigation]]"
  - "[[Localization]]"
  - "[[Mapping]]"
  - "[[SLAM]]"
  - "[[Perception]]"
  - "[[Path Planning Algorithms]]"
  - "[[Robot_Types_and_Applications]]"
---

# Mobile Robots

**Mobile Robots** are defined by their capacity for [[Locomotion|locomotion]] – the ability to move themselves through an environment. Unlike fixed [[Industrial_Arms|manipulators]] bolted to a factory floor, mobile robots can navigate potentially complex and unstructured spaces to perform tasks over extended areas. They represent a major branch of robotics, encompassing a wide variety of forms and application domains, unified by the core challenges of autonomous mobility.

---

## Key Components

A typical autonomous mobile robot integrates several essential subsystems:

* **Platform/Chassis:** The main structural body housing components.
* **[[Locomotion]] System:** The mechanism enabling movement (e.g., wheels, legs, tracks, propellers). Covered in detail in [[Locomotion]].
* **[[Sensor|Sensors]]:**
    * **Proprioceptive:** Measure internal state (e.g., [[IMU_Sensors]] for orientation/acceleration, [[Odometry|wheel encoders]] for distance traveled).
    * **Exteroceptive:** Measure properties of the external environment (e.g., [[LIDAR]] or [[Ultrasonic_Sensors]] for rangefinding, [[Camera_Systems]] for vision, [[GPS]] for global position).
* **Control System:** Onboard computing hardware and software responsible for:
    * **[[Perception]]:** Interpreting sensor data.
    * **[[Navigation]]:** The combination of [[Localization]] (determining position), [[Mapping]] (building environment representations), and [[Path Planning Algorithms|path planning]] (finding collision-free routes).
    * **[[Motion_Control]]:** Executing planned paths or reactive behaviors (like [[Obstacle Avoidance]]) by commanding the locomotion system.
    * **Decision Making:** Higher-level reasoning and task management ([[AI_and_Robot_Control]]).
* **Power Source:** Provides energy for operation (e.g., batteries, fuel cells, internal combustion engines).

---

## Types of Locomotion

Mobile robots are often categorized by their primary mode of locomotion:

1.  **[[Wheeled_Mobile_Robots]]:** The most common type, especially for operation on relatively flat surfaces like indoor floors or paved areas. They offer high energy efficiency and mechanical simplicity. Various drive configurations exist, offering different trade-offs in stability, maneuverability, and controllability, including [[Differential_Drive]], Ackermann steering, omnidirectional drives, and Synchro drives.
2.  **[[Legged_Robots]]:** Inspired by biological systems, these robots use articulated legs for locomotion. Examples include [[Bipedal Locomotion|bipeds]] ([[Humanoid_Robots]]), quadrupeds, and hexapods. They excel at traversing highly uneven terrain, discrete obstacles (like stairs), and gaps. However, they face significant challenges in control complexity (balance, gait generation), mechanical complexity, stability, and energy efficiency compared to wheeled robots.
3.  **Tracked Robots:** Use continuous tracks (like tanks or bulldozers) for locomotion. Provide excellent traction and stability on loose or very rough terrain (e.g., rubble, sand). Steering typically involves slip/skid mechanisms, which makes [[Odometry]] unreliable and control less precise. Often used in military, demining, or construction applications.
4.  **Flying Robots ([[Drones]] / UAVs):** Unmanned Aerial Vehicles use propellers or wings to achieve flight. Offer unique overhead perspectives for surveillance, mapping, inspection, and delivery. Key challenges include flight stability, limited endurance/payload, navigation in GPS-denied areas, and air traffic regulations. See [[Drones]].
5.  **[[Underwater_and_Space_Robots|Underwater Robots]] (AUVs/ROVs):** Autonomous Underwater Vehicles (AUVs) and Remotely Operated Vehicles (ROVs) are designed for subsea exploration, inspection, and intervention. They typically use propellers or thrusters. Major challenges include waterproofing, pressure resistance, limited communication bandwidth (acoustic), and navigation without GPS. ROVs are remotely operated, AUVs are autonomous. See [[Underwater_and_Space_Robots]].
6.  **[[Underwater_and_Space_Robots|Space Robots]] (Rovers):** Designed for exploring the surfaces of planets and moons (e.g., Mars rovers). Must cope with extreme temperatures, radiation, vacuum, communication delays, and challenging terrain. Often feature specialized wheel-leg hybrid suspension systems (like the rocker-bogie) for mobility. See [[Underwater_and_Space_Robots]].
7.  **Other:** Various specialized or bio-inspired forms exist, such as snake-like robots for navigating pipes or rubble, climbing robots, and swimming robots.

---

## Key Challenges and Capabilities

Developing autonomous mobile robots involves addressing several core challenges:

* **[[Navigation]]:** Enabling the robot to successfully plan and execute movement from a starting point to a goal location, encompassing [[Localization]], [[Mapping]], and [[Path Planning Algorithms|Path Planning]]. This is arguably the central problem in mobile robotics.
* **[[Perception]]:** Equipping the robot with sensors and algorithms to interpret its surroundings reliably, detect obstacles, identify landmarks, and build environmental models.
* **[[Localization]]:** Accurately estimating the robot's own position and orientation within its environment, often using [[Sensor_Fusion]] techniques to combine data from various sensors ([[SLAM]] addresses simultaneous mapping and localization).
* **[[Motion_Control]]:** Precisely controlling the robot's actuators to follow desired paths or execute specific maneuvers, considering the robot's [[Kinematics]] and [[Dynamics]].
* **[[Obstacle Avoidance]]:** Reacting to unexpected static or dynamic obstacles detected in real-time.
* **Power Management:** Operating efficiently to maximize mission duration given limited onboard energy storage.
* **Robustness & Reliability:** Functioning consistently in complex, dynamic, and potentially harsh real-world conditions.

---

## Applications

Mobile robots are used in a rapidly expanding range of applications:

* **Logistics & Warehousing:** Autonomous Mobile Robots (AMRs) and Automated Guided Vehicles (AGVs) for transporting goods.
* **Exploration:** Planetary rovers, underwater vehicles for oceanography or archaeology, robots for exploring hazardous environments (e.g., disaster sites, mines, nuclear facilities).
* **Service Robotics:** Delivery robots (hospitals, hotels, campuses), automated cleaning robots, security and surveillance robots, telepresence robots.
* **[[Agricultural_Robots]]:** Autonomous tractors, robotic weeders, monitoring drones, automated harvesting systems.
* **Military & Defense:** Reconnaissance (UAVs, UGVs), bomb disposal, logistics support.
* **Construction & Mining:** Automated excavation, material transport, site survey.
* **Personal & Domestic:** Robotic vacuum cleaners, lawnmowers, potential future assistants.
* **Research & Education:** Platforms like Pioneer, Khepera, TurtleBot are widely used for teaching and research.

The field continues to advance rapidly, driven by progress in sensing, computation, [[AI_and_Robot_Control|AI]], and battery technology, pushing mobile robots into increasingly diverse and challenging domains.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #robotics OR #automation WHERE contains(file.outlinks, [[Robots]])