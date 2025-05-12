---
title: Drones
description: Drones, or unmanned aerial vehicles (UAVs), are robotic systems designed to operate autonomously or semi-autonomously in the air, performing tasks such as surveillance, delivery, and environmental monitoring.
tags:
  - robotics
  - drones
  - unmanned-aerial-vehicles
  - aerial-robotics
  - engineering
  - glossary-term
  - manipulator-arm
  - mobile-robot
layout: default
category: robotics
author: Jordan_Smith_and_le_Chat
date: 2025-05-02
permalink: /drones/
related:
  - "[[Aerial_Robotics]]"
  - "[[Unmanned_Systems]]"
  - "[[Robot_Control]]"
  - "[[Autonomous_Systems]]"
  - "[[Machine_Learning]]"
---

# Drones

**Drones**, or unmanned aerial vehicles (UAVs), are robotic systems designed to operate autonomously or semi-autonomously in the air. They are equipped with advanced sensors, actuators, and control systems, enabling them to perform tasks such as surveillance, delivery, and environmental monitoring. Drones are widely used in various applications, including agriculture, search and rescue, and infrastructure inspection, providing capabilities that enhance efficiency, safety, and data collection.

---

## Key Concepts

### Autonomous Flight

Autonomous flight involves the drone's ability to navigate and perform tasks without human intervention, using advanced control systems and algorithms to interpret sensory inputs and make decisions.

### Sensory Systems

Sensory systems in drones include cameras, lidars, and other sensors that provide data about the environment, enabling tasks such as obstacle detection, mapping, and object recognition.

### Control Systems

Control systems in drones involve the algorithms and mechanisms that govern their behavior, ensuring that they perform tasks accurately and safely. This includes techniques such as feedback control, motion planning, and adaptive control, which enable the drones to operate effectively in dynamic and uncertain environments.

### Communication Systems

Communication systems in drones enable the exchange of data and commands between the drone and the ground station, facilitating tasks such as remote control, telemetry, and data transmission.

---

## Mathematical Formulation

### Feedback Control

Feedback control involves using the drone's sensory inputs to continuously adjust its actions, ensuring that it achieves its goals and adapts to changes in the environment. This includes techniques such as PID control, which adjusts the drone's behavior based on the error between the desired and actual states. The control signal $u(t)$ is given by:

$$
u(t) = K_p e(t) + K_i \int_0^t e(\tau) d\tau + K_d \frac{de(t)}{dt}
$$

where:
- $u(t)$ is the control signal.
- $e(t)$ is the error at time $t$.
- $K_p$, $K_i$, and $K_d$ are the proportional, integral, and derivative gains, respectively.

### Motion Planning

Motion planning involves determining the sequence of movements that a drone should execute to achieve a goal, such as navigating to a destination or avoiding obstacles. This includes techniques such as path planning and trajectory optimization, which enable the drone to perform tasks effectively in its environment.

### Example: Environmental Monitoring

Consider a drone designed for environmental monitoring. The drone's control system uses feedback from its sensors to adjust its flight path, ensuring that it covers the designated area and collects data effectively. The motion planning algorithm determines the optimal path for the drone, enabling it to navigate through the environment, avoid obstacles, and reach its destination efficiently.

---

## Applications in Robotics

- **Surveillance**: Drones are used to monitor and inspect areas, providing data and insights that enhance security and safety.
- **Delivery**: Enables drones to transport goods and packages, facilitating tasks such as logistics and supply chain management.
- **Environmental Monitoring**: Drones are used to collect data about the environment, enabling tasks such as wildlife tracking, pollution monitoring, and disaster assessment.
- **Agriculture**: Drones are used to monitor and manage crops, providing data and insights that enhance productivity and sustainability.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #robotics OR #aerial-robotics WHERE contains(file.outlinks, [[Drones]])
