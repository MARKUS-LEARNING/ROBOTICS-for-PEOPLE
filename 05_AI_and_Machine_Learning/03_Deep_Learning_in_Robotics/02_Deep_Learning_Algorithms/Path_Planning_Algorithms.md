---
title: Path Planning Algorithms
description: Describes Path Planning in robotics, the process of finding a collision-free geometric path from a start to a goal configuration, and common algorithmic approaches.
tags:
  - path-planning
  - motion-planning
  - AI
  - robotics
  - algorithms
  - navigation
  - configuration-space
  - obstacle-avoidance
  - search-algorithms
layout: default
category: robotics
author: Jordan_Smith_&_Gemini
date: 2025-04-28
permalink: /path_planning_algorithms/
related:
  - "[[Trajectory_Planning]]"
  - "[[Motion_Planning]]"
  - "[[ROBOTICS-for-EVERYONE/03_Kinematics_and_Dynamics/03_Trajectory_Planning/Configuration_Space]]"
  - "[[Workspace]]"
  - "[[Obstacle Avoidance]]"
  - "[[Graph Search Algorithms]]"
  - "[[RRT]]"
  - "[[PRM]]"
  - "[[Potential Fields]]"
  - "[[Bug Algorithms]]"
  - "[[Mobile_Robots]]"
  - "[[Manipulator_Arm_Types]]"
  - "[[AI_and_Robot_Control]]"
---

# Path Planning Algorithms

**Path Planning** in robotics refers to the computational problem of finding a feasible geometric path for a robot (e.g., its body for a mobile robot, or its end-effector for a manipulator) from a specified start configuration to a goal configuration, while avoiding collisions with known obstacles in the environment.

It is distinct from [[Trajectory_Planning]], which takes a geometric path and assigns timing information (velocities and accelerations) to create an executable trajectory. Path planning primarily focuses on the geometry of motion, ensuring feasibility and collision avoidance.

---

## Key Concepts

* **[[Configuration Space (C-space)|Configuration Space]] (C-space):** The space representing all possible configurations (positions and orientations) of the robot. Path planning algorithms often operate directly in C-space, where the robot is conceptually shrunk to a point, and obstacles are "grown" into C-space obstacles (C-obstacles).
* **Free Space ($C_{free}$):** The subset of the C-space where the robot is not in collision with any obstacles. The goal of path planning is to find a continuous path within $C_{free}$.
* **Obstacles:** Geometric representations of objects in the workspace that the robot must not intersect.
* **Completeness:** A desirable property of planning algorithms:
    * **Complete:** Guarantees finding a path if one exists and reporting failure if not. Often computationally expensive.
    * **Resolution Complete:** Guaranteed to find a path if one exists at the given discretization resolution (e.g., for grid-based methods).
    * **Probabilistically Complete:** The probability of finding an existing path approaches 1 as computation time increases (common for sampling-based methods).
* **Optimality:** Some algorithms aim to find paths that are optimal according to a specific cost function (e.g., shortest path length, maximum clearance from obstacles, minimum energy).
* **Holonomic vs. Nonholonomic Constraints:** Path planning must respect the robot's kinematic constraints. [[Nonholonomic Constraint|Nonholonomic robots]] (like car-like vehicles) cannot move instantaneously in all directions, making planning more complex than for holonomic robots.

---

## Major Classes of Path Planning Algorithms

### 1. Roadmap Methods

These methods first construct a graph (roadmap) capturing the connectivity of the free space, then search this graph for a path between the start and goal configurations connected to the roadmap.

* **Visibility Graph:** Nodes are the start/goal configurations and the vertices of the C-obstacles. Edges represent straight, collision-free line segments between mutually visible nodes. Finds the shortest paths in terms of path length, but paths tend to graze obstacles. Computationally expensive in higher dimensions.
* **Voronoi Diagram:** The roadmap consists of paths that are equidistant from the two nearest obstacles. Maximizes clearance from obstacles but paths are generally not optimal in length. Complex to compute in higher dimensions.
* **[[PRM|Probabilistic Roadmaps (PRM)]]:** A sampling-based approach. Randomly samples collision-free configurations (nodes) in C-space and connects nearby nodes with simple, collision-free local paths (edges). Efficient for high-dimensional C-spaces (many DoF). Probabilistically complete.

### 2. Cell Decomposition

These methods partition the free space into a collection of simple, non-overlapping cells. A connectivity graph is built representing adjacencies between cells, and this graph is searched for a path.

* **Exact Cell Decomposition:** Uses cells with boundaries defined precisely by obstacle geometry (e.g., trapezoidal decomposition in 2D, cylindrical algebraic decomposition for algebraic boundaries). Complete but often computationally complex.
* **Approximate Cell Decomposition (Grid-Based):** Overlays a regular grid (typically squares in 2D or cubes in 3D) onto the C-space or workspace. Each cell is classified as free, occupied, or mixed based on collisions. Pathfinding involves searching the grid graph using algorithms like Breadth-First Search, Dijkstra's, or A*. Examples include **Wavefront planners** (spreading potential values from the goal) and **Occupancy Grids**. Simple to implement but resolution-dependent and may fail to find paths through narrow passages.

### 3. [[Potential Fields|Artificial Potential Fields]]

* **Method:** Defines a potential function over C-space where the goal configuration exerts an attractive force and obstacles exert repulsive forces. The robot follows the path of steepest descent (negative gradient) of the potential field.
* **Pros:** Simple, computationally efficient, naturally reactive to the environment.
* **Cons:** Susceptible to getting trapped in local minima (configurations where attractive and repulsive forces balance, but which are not the goal). Oscillations can occur near obstacles or in narrow passages. Primarily used for local [[Obstacle Avoidance]] rather than global path planning.

### 4. Sampling-Based Planners (Incremental Search)

These methods build a search tree (or multiple trees) directly in C-space, incrementally exploring the free space from the start configuration(s) towards the goal.

* **[[RRT|Rapidly-exploring Random Trees (RRTs)]]:** Builds a tree by randomly sampling points in C-space and extending the nearest existing node in the tree towards the sample point by a small step. Biased to explore large, unexplored regions quickly. Very effective in high dimensions. Probabilistically complete. Bidirectional variants grow trees from both start and goal. **RRT*:** An optimal variant that rewires the tree connections to converge towards the shortest path solution.
* **Other Tree-Based Planners:** Include methods based on expansive spaces or path-directed subdivisions.

### 5. Search-Based Planners (on Graphs/Grids)

These apply efficient graph search algorithms to explicit graph representations of the C-space, such as grids derived from cell decomposition or precomputed roadmaps.

* **[[Graph Search Algorithms|A* Search]]:** An informed search algorithm that uses a heuristic function (estimating cost-to-go) to prioritize expanding nodes that appear closer to the goal. Finds optimal paths (e.g., shortest) if the heuristic is admissible.
* **[[Graph Search Algorithms|D* Lite]]:** An efficient incremental search algorithm optimized for replanning when edge costs change dynamically (e.g., due to newly detected obstacles). Widely used in mobile robot navigation.

### 6. [[Bug Algorithms]]

* **Method:** Simple, sensor-based algorithms (Bug1, Bug2, Tangent Bug) typically for point robots in 2D with limited sensing (e.g., contact or range). Combine two basic behaviors: move towards the goal and follow obstacle boundaries.
* **Pros:** Provably complete (guaranteed to find the goal if a path exists within the bounded workspace). Minimal computational requirements.
* **Cons:** Generally produce highly suboptimal paths. Limited applicability to simple robots/environments.

The choice of path planning algorithm depends heavily on the specific robot, the dimensionality of the C-space, the complexity of the environment, whether optimal paths are required, real-time constraints, and whether the environment is known beforehand or discovered online. Sampling-based methods like PRM and RRT are currently dominant for high-dimensional problems (e.g., manipulator arms), while grid-based search (like A* or D* Lite) and potential fields/Bug algorithms are common for mobile robot navigation.

