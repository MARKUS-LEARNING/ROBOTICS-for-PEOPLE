---
title: Dijkstra's Algorithm
description: "Dijkstra's algorithm finds the cheapest path from a start to every reachable node in a graph with non-negative edge costs. Covers the idea, a fully traced example, complexity, why negative costs break it, and where robots use it in 2026 (grid and lattice planners, Nav2's NavFn, route graphs, multi-goal cost maps)."
aliases:
  - Dijkstra
  - Dijkstra Algorithm
tags:
  - path-planning
  - graph-traversal
  - navigation
type: method
level: intermediate
subfield:
  - planning
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Shortest_Path]]"
related:
  - "[[A*_Algorithm]]"
  - "[[Path_Planning_Algorithms]]"
  - "[[Nav2_Navigation]]"
  - "[[Search_Algorithms]]"
key_takeaways:
  - "Dijkstra's algorithm settles nodes in order of distance from the start, always expanding the closest unsettled node; with non-negative costs, each settled distance is final."
  - "With a binary heap it runs in O((V + E) log V); it explores in every direction, which A* avoids by adding a heuristic."
  - "Robots use it where all distances are wanted (a cost-to-goal map for many starts), on route graphs, and as the core of grid planners such as Nav2's NavFn."
sources:
  - "Dijkstra, E. W. (1959). A note on two problems in connexion with graphs. https://link.springer.com/article/10.1007/BF01386390"
  - "LaValle, S. M. (2006). Planning Algorithms, chapter 2: Discrete Planning. http://lavalle.pl/planning/"
  - "Macenski, S., Martín, F., White, R. and Ginés Clavero, J. (2020). The Marathon 2: A Navigation System. https://arxiv.org/abs/2003.00368"
---

# Dijkstra's Algorithm

**Dijkstra's algorithm** computes the cheapest path from one start node to every other node of a graph whose edge costs are non-negative. For robots, the graph might be the cells of an occupancy grid, the lanes of a warehouse, or the waypoints of a building.

> **Etymology.** Named after Edsger W. Dijkstra, who designed it in 1956, reportedly in about twenty minutes at a café in Amsterdam, and published it in 1959.

## The idea

Keep a tentative distance for every node: 0 for the start, ∞ for the rest. Repeatedly:
1. take the **unsettled node with the smallest tentative distance** and mark it settled (its distance is now final);
2. for each neighbour, if going through this node is cheaper, lower the neighbour's tentative distance and remember this node as its parent.

Stop when every node is settled (or when the goal is, if only one path is needed). Following parents back from the goal gives the path.

**Why it is correct.** With non-negative costs, any other route to the settled node would have to leave through a node that is at least as far away, so it cannot be shorter.

## Worked example

Edges (directed, with costs): A→B 4, A→C 2, C→B 1, B→D 5, C→D 8, C→E 10, D→E 2.

| Step | Settle | Distance | Updates |
|---|---|---|---|
| 1 | A | 0 | B = 4, C = 2 |
| 2 | C | 2 | B = min(4, 2 + 1) = **3**; D = 2 + 8 = 10; E = 2 + 10 = 12 |
| 3 | B | 3 | D = min(10, 3 + 5) = **8** |
| 4 | D | 8 | E = min(12, 8 + 2) = **10** |
| 5 | E | 10 | — |

Shortest path to E: **A → C → B → D → E**, cost 10. The direct-looking A→C→E costs 12. The detour through B was found because B's distance improved from 4 to 3 when C was settled.

## Cost and limits

- **Complexity** $O((V + E)\log V)$ with a binary-heap priority queue. A 2D grid of $V$ cells has $E \approx 4V$ or $8V$ edges.
- **Explores in all directions.** It settles every node closer than the goal, so on an open map it does far more work than needed for one goal. [[A*_Algorithm|A*]] adds a heuristic to aim the search: on a 20×20 open grid, Dijkstra settled 260 cells where A* settled 16 (see that note).
- **Negative costs break it**: use Bellman–Ford instead (rare in robotics).

## Where robots use it (2026)

- **Cost-to-goal maps.** Run Dijkstra *backwards* from the goal once, and every cell knows its distance and direction to the goal. Useful for many robots or replanning (wavefront planners).
- **Nav2's NavFn planner** computes a navigation function over the costmap with Dijkstra (or A*).
- **Route graphs.** Nav2's Route Server and fleet managers plan on graphs of lanes and docks, where Dijkstra or A* run in microseconds.
- **Inside other planners.** Lattice planners and roadmap methods ([[Probabilistic_Roadmaps]]) end with a graph search.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Dijkstra's_Algorithm]])
```
