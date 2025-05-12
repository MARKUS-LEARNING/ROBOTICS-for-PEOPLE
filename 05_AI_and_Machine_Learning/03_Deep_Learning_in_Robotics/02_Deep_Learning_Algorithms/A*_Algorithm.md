---
title: A* Algorithm
description: "The A* Algorithm is a popular pathfinding and graph traversal algorithm used to find the shortest path between nodes in a graph, commonly used in robotics for path planning."
tags:
  - robotics
  - path-planning
  - algorithms
  - graph-traversal
  - navigation
  - control-systems
  - automation
layout: default
category: robotics
author: Jordan_Smith_&_le_Chat
date: 2025-05-02
permalink: /a_star_algorithm/
related:
  - "[[Path_Planning]]"
  - "[[Dijkstra's_Algorithm]]"
  - "[[Heuristics]]"
  - "[[Graph_Theory]]"
  - "[[Autonomous_Navigation]]"
---

# A* Algorithm

The **A\* Algorithm** is a popular pathfinding and graph traversal algorithm used to find the shortest path between nodes in a graph. It is widely used in robotics for path planning due to its efficiency and ability to incorporate heuristics to guide the search. The A\* algorithm combines the benefits of Dijkstra's algorithm and greedy best-first search, using a heuristic to estimate the cost to the goal and prioritize the exploration of promising paths.

---

## Key Concepts in the A* Algorithm

1. **Graph Representation**: The environment is represented as a graph, where nodes correspond to locations, and edges represent possible paths between locations.

2. **Cost Function**: A function that quantifies the cost of moving from one node to another. This cost can represent distance, time, energy, or other relevant metrics.

3. **Heuristic Function**: A function that estimates the cost from a node to the goal. The heuristic guides the search by prioritizing nodes that are expected to be closer to the goal.

4. **Open and Closed Sets**: The open set contains nodes that are candidates for exploration, while the closed set contains nodes that have already been evaluated.

5. **Optimality**: The A\* algorithm is optimal if the heuristic function is admissible, meaning it never overestimates the true cost to the goal.

---

## Mathematical Representation

### Cost Function

The cost function $f(n)$ for a node $n$ in the A\* algorithm is given by:

$$
f(n) = g(n) + h(n)
$$

where:
- $g(n)$ is the cost from the start node to the current node $n$.
- $h(n)$ is the heuristic estimate of the cost from $n$ to the goal.

The algorithm selects the node with the lowest $f(n)$ value for expansion, balancing the actual cost to reach the node and the estimated cost to the goal.

<br>

### Heuristic Function

The heuristic function $h(n)$ must be admissible to ensure the optimality of the A\* algorithm. A heuristic is admissible if:

$$
h(n) \leq h^*(n)
$$

where $h^*(n)$ is the true cost from node $n$ to the goal. Common heuristics include the Euclidean distance, Manhattan distance, and Chebyshev distance, depending on the problem domain.

<br>

### Algorithm Steps

1. **Initialization**:
   - Initialize the open set with the start node.
   - Initialize the closed set as empty.
   - Set $g(start) = 0$ and $f(start) = h(start)$.

2. **Node Selection**:
   - Select the node with the lowest $f$ value from the open set.
   - Move this node to the closed set.

3. **Path Expansion**:
   - For each neighbor of the selected node, calculate the tentative $g$ score:
     $$
     g(neighbor) = g(current) + c(current, neighbor)
     $$
     where $c(current, neighbor)$ is the cost to move from the current node to the neighbor.
   - If the neighbor is not in the open or closed set, or if the tentative $g$ score is lower than the previous $g$ score, update the neighbor's $g$ and $f$ values and add it to the open set.

4. **Termination**:
   - If the goal node is selected, reconstruct the path by following the parent pointers from the goal to the start.
   - If the open set is empty and the goal has not been reached, no path exists.

---

## Applications of the A* Algorithm

The A\* algorithm is used in various robotic applications:

- **Path Planning**: Finding the shortest or optimal path for robots to navigate through environments with obstacles.
- **Autonomous Navigation**: Enabling robots to plan and execute paths in real-time, avoiding collisions and reaching their destinations efficiently.
- **Game Development**: Used in video games for character pathfinding and AI behavior.
- **Logistics**: Optimizing routes for delivery and transportation systems.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #robotics OR #deep-learning WHERE contains(file.outlinks, [[Deep_Learning_in_Robotics]])
