---
title: Graph Theory
description: Graph Theory is a branch of mathematics studying graph structures, which are mathematical models of pairwise relations between objects.
tags:
  - mathematics
  - algorithms
  - networks
  - computer-science
  - discrete-mathematics
  - optimization
  - glossary-term
layout: default
category: mathematics
author: Jordan_Smith_and_le_Chat
date: 2025-04-29
permalink: /graph_theory/
related:
  - "[[Shortest_Path]]"
  - "[[Dijkstra's_Algorithm]]"
  - "[[A*_Algorithm]]"
  - "[[Network_Analysis]]"
  - "[[Combinatorial_Optimization]]"
---

# Graph Theory

**Graph Theory** is a branch of mathematics that studies graph structures, which are mathematical models of pairwise relations between objects. It provides a framework for analyzing complex networks and systems, making it a fundamental tool in various fields such as computer science, engineering, and social sciences. Graph theory is used to model and solve problems involving connectivity, paths, flows, and optimization.

---

## Key Concepts

1. **Graph**: A structure consisting of a set of vertices (or nodes) and a set of edges (or links) that connect pairs of vertices. Graphs are used to represent relationships and interactions between entities.
   <br>

2. **Vertex (Node)**: A fundamental unit of a graph representing an object or an entity. Vertices are the basic elements that form the structure of a graph.
   <br>

3. **Edge (Link)**: A connection between two vertices, representing a relationship or interaction between them. Edges can be directed or undirected and may have weights.
   <br>

4. **Directed Graph (Digraph)**: A graph in which edges have a direction, indicating a one-way relationship between vertices. Directed graphs are used to model systems with unidirectional flows or influences.
   <br>

5. **Undirected Graph**: A graph in which edges do not have a direction, representing mutual relationships between vertices. Undirected graphs are commonly used to model bidirectional interactions.
   <br>

6. **Weighted Graph**: A graph where each edge has an associated weight, representing a cost or value associated with the connection. Weighted graphs are essential for problems involving optimization and resource allocation.
   <br>

7. **Path**: A sequence of vertices and edges that connects a starting vertex to an ending vertex. Paths are fundamental in routing, navigation, and connectivity analysis.
   <br>

8. **Cycle**: A path that starts and ends at the same vertex and does not repeat any other vertices. Cycles are important in detecting loops and redundancies in networks.
   <br>

9. **Connected Graph**: A graph in which there is a path between every pair of vertices. Connected graphs are used to analyze the overall connectivity of a network.
   <br>

10. **Tree**: A connected graph with no cycles, often used to represent hierarchical structures. Trees are fundamental in algorithms for sorting, searching, and organizing data.
    <br>

---

## Mathematical Representations

Graph theory involves several mathematical representations and notations:

- **Adjacency Matrix**: A matrix representation of a graph where the element at row $i$ and column $j$ is 1 if there is an edge between vertex $i$ and vertex $j$, and 0 otherwise. The adjacency matrix $A$ for a graph with $n$ vertices is defined as:

$$
A_{ij} =
\begin{cases}
1 & \text{if there is an edge between vertex } i \text{ and vertex } j \\
0 & \text{otherwise}
\end{cases}
$$
<br>

- **Incidence Matrix**: A matrix representation of a graph where the rows correspond to vertices and the columns correspond to edges, with entries indicating the incidence of vertices with edges. The incidence matrix $B$ for a graph with $n$ vertices and $m$ edges is defined as:

$$
B_{ij} =
\begin{cases}
1 & \text{if vertex } i \text{ is incident to edge } j \\
0 & \text{otherwise}
\end{cases}
$$
<br>

- **Degree of a Vertex**: The number of edges connected to a vertex. For a vertex $v$, the degree is denoted as $\deg(v)$ and is calculated as:

$$
\deg(v) = \sum_{i=1}^{n} A_{vi}
$$

where $A$ is the adjacency matrix of the graph.
<br>

- **Graph Coloring**: An assignment of colors to the vertices of a graph such that no two adjacent vertices share the same color. Graph coloring is used in scheduling and resource allocation problems to minimize conflicts.
  <br>

---

## Shortest Path Algorithms in Robotics

Graph-based path planning is fundamental to mobile robotics. The workspace or configuration space is discretized into a graph (grid, lattice, or roadmap), and shortest-path algorithms find collision-free paths.

### Dijkstra's Algorithm

Dijkstra's algorithm finds the shortest path from a source vertex $s$ to all other vertices in a weighted graph $G = (V, E)$ with non-negative edge weights $w(u,v) \geq 0$.

**Algorithm:** Maintain a distance estimate $d[v]$ for every vertex, initialized to $\infty$ except $d[s] = 0$. Repeatedly extract the vertex $u$ with minimum $d[u]$ from a priority queue and relax all its neighbors:

$$
d[v] \leftarrow \min\bigl(d[v],\; d[u] + w(u, v)\bigr)
$$

**Complexity:** $O((|V| + |E|) \log |V|)$ with a binary heap.

**Robotics context:** Dijkstra's is used in grid-based planners (e.g., the `global_planner` package in ROS 2 Nav2) where the graph is an 8-connected occupancy grid. For a typical 100 m x 100 m map at 5 cm resolution, the grid has $2000 \times 2000 = 4 \times 10^6$ cells. Dijkstra's explores all of them, which motivates using A* instead.

### A* Algorithm

A* extends Dijkstra's by adding a heuristic $h(v)$ that estimates the cost from $v$ to the goal $t$. Each vertex is evaluated by:

$$
f(v) = g(v) + h(v)
$$

where $g(v)$ is the actual cost from $s$ to $v$ (same as Dijkstra's $d[v]$), and $h(v)$ is the heuristic estimate from $v$ to $t$.

**Optimality guarantee:** A* returns the shortest path if $h(v)$ is **admissible** (never overestimates the true cost) and **consistent** (satisfies the triangle inequality):

$$
h(u) \leq w(u, v) + h(v) \quad \forall (u,v) \in E
$$

**Complexity:** $O((|V| + |E|) \log |V|)$ worst case, but in practice A* expands far fewer nodes than Dijkstra's --- typically 10--100x fewer for grid-based robot navigation.

**Robotics context:** A* is the default global planner in the ROS 2 Nav2 stack (`NavfnPlanner`). Common heuristics for 2D grid planning include:
- Euclidean distance: $h(v) = \sqrt{(x_v - x_t)^2 + (y_v - y_t)^2}$ (admissible, good for any-angle movement)
- Manhattan distance: $h(v) = |x_v - x_t| + |y_v - y_t|$ (admissible for 4-connected grids)
- Diagonal/Chebyshev distance: $h(v) = \max(|x_v - x_t|, |y_v - y_t|)$ (admissible for 8-connected grids)

---

## Sampling-Based Planning and Graph Theory

Sampling-based planners construct graphs implicitly or explicitly in the robot's configuration space $\mathcal{C}$, which can be high-dimensional (6D for a rigid body, 7D for a 7-DOF arm, 12D+ for a dual-arm system).

### Probabilistic Roadmap (PRM)

PRM is a **multi-query** planner that constructs a graph (roadmap) in $\mathcal{C}$:

1. **Sampling phase:** Generate $N$ random configurations $q_i \in \mathcal{C}_{\text{free}}$ (these become vertices).
2. **Connection phase:** For each sample $q_i$, connect it to its $k$-nearest neighbors (or all neighbors within radius $r$) if the straight-line path in $\mathcal{C}$ is collision-free (these become edges).
3. **Query phase:** Given start $q_s$ and goal $q_g$, connect them to the roadmap and run A* or Dijkstra's on the resulting graph.

The connection radius for asymptotic optimality (PRM*) is:

$$
r_n = \gamma \left( \frac{\log n}{n} \right)^{1/d}
$$

where $d$ is the dimension of $\mathcal{C}$, $n$ is the number of samples, and $\gamma$ is a constant depending on the volume of $\mathcal{C}_{\text{free}}$.

**Practical note:** PRM is ideal when you need to plan many paths in the same static environment (e.g., a fixed manufacturing workcell). MoveIt 2 uses PRM variants from the OMPL library.

### Rapidly-exploring Random Tree (RRT)

RRT is a **single-query** planner that incrementally builds a tree (a special case of a graph with no cycles):

1. Sample a random configuration $q_{\text{rand}}$.
2. Find the nearest vertex $q_{\text{near}}$ in the tree.
3. Extend from $q_{\text{near}}$ toward $q_{\text{rand}}$ by a step size $\epsilon$ to get $q_{\text{new}}$.
4. If the path from $q_{\text{near}}$ to $q_{\text{new}}$ is collision-free, add the edge.

RRT* adds a rewiring step that reconnects nearby vertices through $q_{\text{new}}$ if this reduces their cost, guaranteeing asymptotic optimality.

**Robotics context:** RRT-Connect (bidirectional RRT) is the default planner in MoveIt 2 for manipulator motion planning. Typical parameters:
- Step size $\epsilon$: 0.05--0.2 rad in joint space
- Goal bias: 5--10% (probability of sampling $q_{\text{goal}}$ directly)
- Planning time budget: 1--5 seconds for a 7-DOF arm in a moderately cluttered environment

The key graph-theoretic insight: RRT builds a spanning tree of the reachable configuration space, while PRM builds a general graph. Both reduce the continuous motion planning problem to a discrete graph search problem.

---

## Applications of Graph Theory

Graph theory has a wide range of applications across various fields:

- **Computer Science**: Used in algorithm design, network analysis, and data structures. Examples include shortest path algorithms, network flow problems, and graph traversal techniques.
  <br>

- **Engineering**: Applied in circuit design, communication networks, and transportation systems to optimize connectivity and efficiency.
  <br>

- **Social Sciences**: Used to model social networks, analyze relationships, and study the spread of information or diseases.
  <br>

- **Operations Research**: Employed in optimization problems, such as scheduling, resource allocation, and supply chain management.
  <br>

- **Biology**: Used to model biological networks, such as metabolic pathways and genetic interactions.
  <br>

- **Robotics Path Planning**: Dijkstra's, A*, PRM, and RRT all rely on graph theory to find collision-free paths through configuration space or workspace.
  <br>

- **Task and Motion Planning (TAMP)**: Task-level planners use AND/OR graphs or constraint graphs to reason about object placements, grasps, and action sequences.
  <br>

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

- **List all related algorithms and concepts**:
  ```dataview
  LIST FROM #Algorithms OR #Mathematics WHERE contains(file.outlinks, [[Graph_Theory]])
  ```
