---
title: Dijkstra's Algorithm
description: "Dijkstra's Algorithm is a classic algorithm used to find the shortest path between nodes in a graph, commonly used in robotics for path planning."
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
author: Jordan_Smith_&_Gemini
date: 2025-05-02
permalink: /dijkstras_algorithm/
related:
  - "[[Path_Planning]]"
  - "[[A*_Algorithm]]"
  - "[[Graph_Theory]]"
  - "[[Shortest_Path]]"
  - "[[Autonomous_Navigation]]"
---

# Dijkstra's Algorithm

**Dijkstra's Algorithm** is a classic algorithm used to find the shortest path between nodes in a graph, particularly when the graph has non-negative weights. It is widely used in robotics for path planning due to its simplicity and effectiveness in finding optimal paths in environments with uniform costs. The algorithm systematically explores all possible paths, ensuring that the shortest path is found.

---

## Key Concepts in Dijkstra's Algorithm

1. **Graph Representation**: The environment is represented as a graph, where nodes correspond to locations, and edges represent possible paths between locations, each with an associated cost or weight.

2. **Cost Function**: A function that quantifies the cost of moving from one node to another. This cost can represent distance, time, energy, or other relevant metrics.

3. **Priority Queue**: A data structure used to efficiently retrieve the node with the smallest tentative distance, guiding the exploration of the graph.

4. **Tentative Distance**: The current known shortest distance from the start node to each node in the graph, which is updated as the algorithm progresses.

5. **Optimality**: Dijkstra's algorithm guarantees the shortest path in graphs with non-negative weights, making it suitable for many path planning applications.

---

## Mathematical Representation

### Initialization

Dijkstra's algorithm initializes the tentative distance $d(n)$ for each node $n$ in the graph:

$$
d(start) = 0
$$
$$
d(n) = \infty \quad \text{for all other nodes } n
$$

The algorithm also initializes a priority queue with the start node and an empty set of visited nodes.

<br>

### Algorithm Steps

1. **Node Selection**:
   - Select the node with the smallest tentative distance from the priority queue.
   - Remove this node from the priority queue and add it to the set of visited nodes.

2. **Distance Update**:
   - For each neighbor of the selected node, calculate the tentative distance:
     $$
     d(neighbor) = \min(d(neighbor), d(current) + c(current, neighbor))
     $$
     where $c(current, neighbor)$ is the cost to move from the current node to the neighbor.
   - If the tentative distance is updated, add the neighbor to the priority queue if it is not already present.

3. **Termination**:
   - If the goal node is selected, the shortest path is found by tracing back from the goal to the start using the parent pointers.
   - If the priority queue is empty and the goal has not been reached, no path exists.

---

## Applications of Dijkstra's Algorithm

Dijkstra's algorithm is used in various robotic applications:

- **Path Planning**: Finding the shortest or optimal path for robots to navigate through environments with obstacles.
- **Autonomous Navigation**: Enabling robots to plan and execute paths in real-time, avoiding collisions and reaching their destinations efficiently.
- **Network Routing**: Optimizing data transmission paths in communication networks.
- **Logistics**: Planning efficient routes for delivery and transportation systems.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
list from "Path Planning" or "A* Algorithm" or "Graph Theory" or "Shortest Path"
