---
title: Shortest Path
description: "The shortest path problem is a fundamental concept in graph theory and algorithms, involving finding the shortest route between two points in a graph or network."
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
author: Jordan_Smith_and_le_Chat
date: 2025-04-29
permalink: /shortest_path/
related:
  - "[[Path_Planning]]"
  - "[[A*_Algorithm]]"
  - "[[Graph_Theory]]"
  - "[[Dijkstra's_Algorithm]]"
  - "[[Autonomous_Navigation]]"
---

# Shortest Path

The **shortest path problem** is a fundamental concept in graph theory and algorithms, involving finding the shortest route between two points in a graph or network. This note explores the key concepts, algorithms, and mathematical representations related to the shortest path problem.

---

## Key Concepts

1. **Graph**: A collection of nodes (vertices) and edges (connections between nodes).
   <br>

2. **Weighted Graph**: A graph where each edge has an associated weight or cost.
   <br>

3. **Shortest Path**: The path between two nodes with the smallest total weight.
   <br>

## Algorithms

Several algorithms are used to solve the shortest path problem, each with its own strengths and use cases:

1. **Dijkstra's Algorithm**:
   - Used for graphs with non-negative weights.
   - Finds the shortest path from a single source node to all other nodes.
   - Time complexity: $O((V + E) \log V)$, where $V$ is the number of vertices and $E$ is the number of edges.
   <br>

2. **Bellman-Ford Algorithm**:
   - Can handle graphs with negative weights.
   - Useful for detecting negative weight cycles.
   - Time complexity: $O(V \cdot E)$.
   <br>

3. **A* Algorithm**:
   - An informed search algorithm that uses heuristics to guide the search.
   - Often used in pathfinding and AI applications.
   - Time complexity: Depends on the heuristic used.
   <br>

## Mathematical Representations

The shortest path problem can be represented mathematically using various notations and equations. Here are some key representations:

- **Graph Representation**: A graph $G = (V, E)$ where $V$ is the set of vertices and $E$ is the set of edges.
  <br>

- **Distance Function**: $d(u, v)$ represents the shortest distance from vertex $u$ to vertex $v$.
  <br>

- **Relaxation Equation**: For an edge $(u, v)$ with weight $w(u, v)$, the relaxation step is given by:
  $$
  d(v) = \min(d(v), d(u) + w(u, v))
  $$
  <br>

## Dataview Plugin Features

To leverage the power of the Dataview plugin, you can use the following queries to explore related notes and concepts:

- **List all algorithms related to the shortest path problem**:
  ```dataview
  LIST FROM #Algorithms WHERE contains(file.outlinks, [[Shortest_Path]])
