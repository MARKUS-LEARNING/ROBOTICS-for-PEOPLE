---
title: Probabilistic Roadmaps
description: Describes Probabilistic Roadmaps (PRM), a sampling-based motion planning algorithm that constructs a roadmap of the configuration space using random samples.
tags:
  - robotics
  - motion-planning
  - path-planning
  - algorithms
  - sampling
  - probabilistic
layout: default
category: robotics
author: Jordan_Smith_and_le_Chat
date: 2025-05-03
permalink: /probabilistic_roadmaps/
related:
  - "[[Motion_Planning]]"
  - "[[Path_Planning_Algorithms]]"
  - "[[ROBOTICS-for-EVERYONE/03_Kinematics_and_Dynamics/03_Trajectory_Planning/Configuration_Space]]"
  - "[[Collision_Detection]]"
  - "[[Sampling-Based_Planning]]"
---

# Probabilistic Roadmaps (PRM)

**Probabilistic Roadmaps (PRM)** is a sampling-based motion planning algorithm that constructs a roadmap of the configuration space using random samples. This method is particularly effective for high-dimensional spaces and complex environments where traditional search methods may be inefficient.

---

## Key Concepts

* **Roadmap:** A graph where nodes represent randomly sampled configurations, and edges represent feasible paths between them.
* **Sampling:** The process of randomly selecting points in the configuration space to explore potential paths.
* **Connectivity:** The degree to which nodes in the roadmap are connected, affecting the completeness and efficiency of the pathfinding process.
* **Probabilistic Completeness:** The property that the probability of finding a solution approaches 1 as the number of samples increases.

---

## Methodology

### 1. Sampling the Configuration Space

* **Concept:** Randomly sample the configuration space to generate a set of nodes.
* **Steps:**
  1. Randomly select points $q$ in the configuration space $\mathcal{C}$:
     $$
     q \in \mathcal{C}
     $$
  2. Ensure that the sampled points are collision-free using collision detection algorithms.

### 2. Building the Roadmap

* **Concept:** Connect the sampled nodes to form a network of possible paths.
* **Steps:**
  1. For each sampled node, attempt to connect it to its nearest neighbors using a local planner.
  2. Use a collision detection algorithm to ensure that the edges (paths) between nodes are feasible.
  3. Construct a graph $( G = (V, E)$ where $V$ is the set of nodes and $E$ is the set of edges.

### 3. Querying the Roadmap

* **Concept:** Use the roadmap to find a path from the start to the goal configuration.
* **Steps:**
  1. Add the start and goal configurations to the roadmap as nodes.
  2. Use a graph search algorithm (e.g., A* or Dijkstra's) to find a path from the start to the goal node.
  3. If a path is found, return it; otherwise, continue sampling and updating the roadmap.

---

## Mathematical Representation

### Probabilistic Completeness

A PRM is probabilistically complete if the probability of finding a solution approaches 1 as the number of samples \( n \) approaches infinity:

$$
\lim_{n \to \infty} P(\text{solution found}) = 1
$$

### Path Feasibility

A path \( \pi \) is feasible if it does not intersect with any obstacle \( \mathcal{O} \):

$$
\pi \cap \mathcal{O} = \emptyset
$$

---

## Applications

Probabilistic Roadmaps are widely used in various robotic applications, including:

* **Mobile Robots:** Navigating through complex environments with obstacles.
* **Manipulator Arms:** Planning paths for tasks like assembly, welding, or pick-and-place operations.
* **Autonomous Vehicles:** Planning safe and efficient routes for self-driving cars, drones, or underwater vehicles.
* **Human-Robot Interaction:** Ensuring safe operation when robots interact with humans in shared workspaces.

---

## Challenges

* **Sampling Density:** Ensuring sufficient coverage of the configuration space with a limited number of samples.
* **Dynamic Environments:** Adapting to changes in the environment in real-time.
* **Optimality:** Finding not just any feasible path, but an optimal one in terms of length, time, or energy.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #robotics OR #motion-planning WHERE contains(file.outlinks, [[Probabilistic_Roadmaps]])
