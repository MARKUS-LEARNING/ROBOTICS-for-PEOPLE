---
title: Mapping
description: Mapping in robotics involves creating a representation of the environment, enabling robots to navigate, plan paths, and interact with their surroundings.
tags:
  - robotics
  - navigation
  - sensors
  - control
  - engineering
type: Robotic Concept
application: Creating environmental representations for navigation
layout: default
category: robotics
author: Jordan_Smith_and_le_Chat
date: 2025-04-29
permalink: /mapping/
related:
  - "[[Autonomous_Navigation]]"
  - "[[Robot_Design]]"
  - "[[SLAM]]"
  - "[[Localization]]"
  - "[[Sensor_Fusion]]"
  - "[[Occupancy_Grid]]"
  - "[[LIDAR]]"
  - "[[Path_Planning_Algorithms]]"
---

# Mapping

**Mapping** in robotics involves creating a representation of the environment, enabling robots to navigate, plan paths, and interact with their surroundings. It is a fundamental aspect of autonomous navigation, allowing robots to understand their spatial context and make informed decisions about movement and interaction. Mapping involves the use of sensors, algorithms, and data structures to build accurate and efficient representations of the environment.

---

## Key Concepts in Mapping

1. **Occupancy Grid**: A discrete representation of the environment where the space is divided into a grid of cells, each indicating the probability of being occupied by an obstacle. Occupancy grids are commonly used in 2D mapping for navigation and obstacle avoidance.

2. **Feature-Based Maps**: Maps that represent the environment using distinct features or landmarks, such as corners, edges, or objects. Feature-based maps are useful for localization and navigation in structured environments.

3. **Metric Maps**: Maps that provide precise geometric information about the environment, including distances, angles, and shapes. Metric maps are essential for tasks requiring high accuracy, such as path planning and object manipulation.

4. **Topological Maps**: Maps that represent the environment as a graph of nodes and edges, where nodes correspond to distinct locations or landmarks, and edges represent the connectivity between them. Topological maps are useful for high-level navigation and planning.

5. **SLAM (Simultaneous Localization and Mapping)**: A technique that allows a robot to build a map of its environment while simultaneously determining its location within that map. SLAM is crucial for autonomous navigation in unknown or dynamic environments.

---

## Key Equations

<br>

- **Occupancy Grid Update**:

  $$
  P(O_t | z_{1:t}) = \frac{P(z_t | O_t) P(O_t | z_{1:t-1})}{P(z_t | z_{1:t-1})}
  $$

  <br>

  where $P(O_t | z_{1:t})$ is the probability of a cell being occupied given the measurements up to time $t$, $P(z_t | O_t)$ is the likelihood of the measurement given the occupancy, $P(O_t | z_{1:t-1})$ is the prior probability, and $P(z_t | z_{1:t-1})$ is the normalization factor. This equation updates the occupancy probability based on sensor measurements.

  <br>

- **Feature Extraction**:

  $$
  f_i = h(z_i)
  $$

  <br>

  where $f_i$ is the extracted feature, $h(z_i)$ is the feature extraction function, and $z_i$ is the sensor measurement. This equation represents the process of extracting features from sensor data for use in feature-based maps.

  <br>

- **SLAM Measurement Model**:

  $$
  z_t = h(x_t) + v_t
  $$

  <br>

  where $z_t$ is the measurement at time $t$, $h(x_t)$ is the measurement function that relates the state $x_t$ to the measurement, and $v_t$ is the measurement noise. This equation is used in SLAM to relate the robot's state to the sensor measurements.

  <br>

---

## Impact on Robotics

- **Autonomous Navigation**: Mapping is essential for autonomous navigation, enabling robots to understand their environment and plan safe and efficient paths. It allows robots to operate in unknown or dynamic environments without human intervention.

- **Environmental Interaction**: Accurate maps enable robots to interact with their surroundings effectively, whether it's avoiding obstacles, manipulating objects, or navigating through complex spaces.

- **Path Planning**: Mapping provides the necessary spatial information for path planning, allowing robots to determine the best routes to their destinations while avoiding obstacles and hazards.

- **Localization**: Mapping is closely tied to localization, as it provides the environmental context needed for robots to determine their position and orientation accurately.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts
```dataview
LIST FROM #robotics OR #navigation WHERE contains(file.outlinks, [[Mapping]])
