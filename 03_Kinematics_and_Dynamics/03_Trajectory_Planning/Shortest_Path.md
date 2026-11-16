---
title: Shortest Path
description: "The shortest-path problem: find the cheapest route between nodes of a weighted graph. Covers the problem variants (single-source, single-pair, all-pairs), the algorithms for each (BFS, Dijkstra, A*, Bellman–Ford, Floyd–Warshall) with their costs, how robots turn maps into graphs, and what 'cost' should mean for a robot."
aliases:
  - Shortest Path Problem
  - Shortest Paths
tags:
  - path-planning
  - graph-traversal
  - navigation
type: concept
level: beginner
subfield:
  - planning
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Search_Algorithms]]"
related:
  - "[[Dijkstra's_Algorithm]]"
  - "[[A*_Algorithm]]"
  - "[[Path_Planning_Algorithms]]"
  - "[[Configuration_Space]]"
key_takeaways:
  - "A shortest-path problem needs a graph (nodes and edges) and a cost on each edge; 'shortest' means lowest total cost, which need not be distance."
  - "Pick the algorithm by question and costs: BFS for equal costs, Dijkstra for non-negative costs from one start, A* for one goal with a good heuristic, Bellman–Ford for negative costs, Floyd–Warshall for all pairs on small graphs."
  - "For robots, the hard parts are building the graph (grids, roadmaps, lattices, route graphs) and choosing costs that capture time, risk, energy and comfort."
sources:
  - "Dijkstra, E. W. (1959). A note on two problems in connexion with graphs. https://link.springer.com/article/10.1007/BF01386390"
  - "Hart, P. E., Nilsson, N. J. and Raphael, B. (1968). A Formal Basis for the Heuristic Determination of Minimum Cost Paths. https://ieeexplore.ieee.org/document/4082128"
  - "LaValle, S. M. (2006). Planning Algorithms, chapter 2: Discrete Planning. http://lavalle.pl/planning/"
---

# Shortest Path

The **shortest-path problem** asks for the cheapest route between points of a **graph**: a set of nodes joined by edges, each edge with a cost. It is one of the most solved problems in computer science, and the core of most robot navigation.

> **Etymology.** The problem was studied in the 1950s for telephone routing and transport networks. Dijkstra's (1959), Bellman's and Ford's (1956–58) algorithms all date from that decade.

## Variants and the right algorithm

| Question | Edge costs | Algorithm | Time (V nodes, E edges) |
|---|---|---|---|
| Fewest steps from a start | All equal | Breadth-first search | $O(V + E)$ |
| Cheapest from one start to all nodes | Non-negative | [[Dijkstra's_Algorithm\|Dijkstra]] | $O((V+E)\log V)$ |
| Cheapest from one start to one goal | Non-negative, good heuristic | [[A*_Algorithm\|A*]] | Usually far below Dijkstra |
| From one start, some costs negative | Any (no negative cycles) | Bellman–Ford | $O(VE)$ |
| Between every pair | Any (no negative cycles) | Floyd–Warshall | $O(V^3)$ |

For scale: an occupancy grid of 50 m × 50 m at 5 cm resolution has $1000 \times 1000 = 10^6$ cells. Dijkstra or A* handle that in milliseconds to tens of milliseconds; Floyd–Warshall ($10^{18}$ operations) is out of the question.

## From a robot's world to a graph

| Graph | Nodes | Edges | Used by |
|---|---|---|---|
| **Occupancy grid** | Free cells | Neighbouring cells | 2D navigation (Nav2) |
| **Roadmap** | Random collision-free configurations | Collision-free straight connections | Arm planning ([[Probabilistic_Roadmaps]]) |
| **State lattice** | Discretised poses (x, y, θ) | Precomputed drivable motions | Cars, legged and omnidirectional bases |
| **Route / topological graph** | Places, junctions, docks | Lanes, corridors | Warehouses, campuses, fleets |
| **Scene graph** | Rooms, objects | Adjacency, containment | Language-driven tasks ("go to the kitchen table") |

## What "cost" should mean

Distance is the simplest edge cost, but rarely the right one:
- **time**, given speed limits in narrow or crowded areas;
- **risk**, through inflated costs near obstacles, so paths keep clearance;
- **energy**, from slopes, carpet, payload;
- **social cost**, avoiding passing between people or through busy doorways;
- **semantic cost**, from learned terrain or segmentation models, such as grass versus pavement for a delivery robot.

A planner returns the *cheapest* path under the costs it is given, so most "bad path" bugs are cost-design bugs.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Shortest_Path]])
```
