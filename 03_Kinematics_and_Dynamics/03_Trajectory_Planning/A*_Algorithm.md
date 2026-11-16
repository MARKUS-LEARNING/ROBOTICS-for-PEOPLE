---
title: A* Algorithm
description: "A* finds the cheapest path to one goal by combining the cost so far with a heuristic estimate of the cost to go. Covers f = g + h, admissible and consistent heuristics for robot grids, a computed comparison with Dijkstra (16 versus 260 cells expanded on an open grid), weighted and anytime variants, Hybrid-A* for car-like robots, and where A* runs on robots in 2026."
aliases:
  - A star
  - A-star
  - Hybrid A*
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
  - "[[Dijkstra's_Algorithm]]"
related:
  - "[[Path_Planning_Algorithms]]"
  - "[[Nav2_Navigation]]"
  - "[[Search_Algorithms]]"
  - "[[Configuration_Space]]"
key_takeaways:
  - "A* expands the node with the smallest f = g + h, where g is the cost so far and h estimates the remaining cost; with an admissible heuristic (never overestimating), the first path found to the goal is optimal."
  - "A good heuristic focuses the search on the goal: on a 20×20 open grid A* expanded 16 cells where Dijkstra expanded 260, both finding the same 15-step path."
  - "Variants fit robots: weighted A* trades optimality for speed, D* Lite replans as the map changes, and Hybrid-A* plans drivable paths for car-like vehicles (Nav2's Smac Hybrid-A*)."
sources:
  - "Hart, P. E., Nilsson, N. J. and Raphael, B. (1968). A Formal Basis for the Heuristic Determination of Minimum Cost Paths. https://ieeexplore.ieee.org/document/4082128"
  - "Koenig, S. and Likhachev, M. (2002). D* Lite. https://idm-lab.org/bib/abstracts/papers/aaai02b.pdf"
  - "LaValle, S. M. (2006). Planning Algorithms, chapter 2: Discrete Planning. http://lavalle.pl/planning/"
  - "Russell, S. and Norvig, P. (2020). Artificial Intelligence: A Modern Approach, 4th edition, chapter 3. https://aima.cs.berkeley.edu/"
  - "Macenski, S., Moore, T., Lu, D. V., Merzlyakov, A. and Ferguson, M. (2023). From the desks of ROS maintainers: A survey of modern & capable mobile robotics algorithms in the robot operating system 2. https://arxiv.org/abs/2307.15236"
---

# A* Algorithm

**A\*** (say "A-star") finds the cheapest path from a start to one goal by searching first where the goal probably is. It is [[Dijkstra's_Algorithm|Dijkstra's algorithm]] plus a **heuristic**: an estimate of how far each node still is from the goal.

> **Etymology.** Peter Hart, Nils Nilsson and Bertram Raphael created it in 1968 for **Shakey**, SRI's mobile robot. Earlier versions were called A1 and A2. The star marks the version that was proven *optimal*, a notation borrowed from statistics, where an asterisk marks the optimal estimate.

## The rule

For each node $n$:

$$f(n) = g(n) + h(n)$$

- $g(n)$ is the cost of the best path found so far from the start to $n$;
- $h(n)$ is the heuristic estimate of the cost from $n$ to the goal;
- A* always expands the open node with the smallest $f$.

With $h = 0$ it *is* Dijkstra. If $h$ is **admissible** (never overestimates the true remaining cost), the first time A* removes the goal from the queue, its path is optimal. If $h$ is also **consistent** ($h(n) \le c(n, n') + h(n')$), no node ever needs re-expanding.

## Heuristics for robot grids

| Movement | Admissible heuristic |
|---|---|
| 4-connected grid (up, down, left, right) | Manhattan distance $\lvert\Delta x\rvert + \lvert\Delta y\rvert$ |
| 8-connected grid (with diagonals of cost √2) | Octile distance $\max(\lvert\Delta x\rvert,\lvert\Delta y\rvert) + (\sqrt2-1)\min(\lvert\Delta x\rvert,\lvert\Delta y\rvert)$ |
| Any-angle motion | Euclidean distance |
| Car-like robot | Max of the obstacle-free Reeds–Shepp or Dubins path length and a 2D grid distance that accounts for obstacles |

## Computed comparison with Dijkstra

Searches on a 4-connected grid with unit costs, Manhattan heuristic, counting the cells settled (expanded):

| Map | Path length | Dijkstra expanded | A* expanded |
|---|---|---|---|
| Open 20×20, start (2, 10) → goal (17, 10) | 15 | 260 | 16 |
| 8×6 with a wall at x = 4 (gap only in the top row), (1, 2) → (6, 2) | 11 | 36 | 32 |

On the open map the heuristic is exact, so A* walks straight to the goal. Behind a wall the heuristic is misleading (it points through the wall), so A* explores almost as much as Dijkstra. The quality of $h$ decides how much A* saves. Both always return a shortest path.

## Variants used on robots

| Variant | Idea | Use |
|---|---|---|
| **Weighted A\*** | $f = g + w\,h$ with $w > 1$ | Faster, path at most $w$ times optimal |
| **Anytime (ARA\*)** | Start with a large $w$, then refine | A usable path at once, better ones as time allows |
| **D\* Lite** (2002) | Reuse earlier search when the map changes | Replanning as sensors reveal obstacles; used on Mars rovers in the D* family |
| **Theta\*** | Allow any-angle shortcuts across the grid | Shorter, smoother paths (a Nav2 planner) |
| **Hybrid-A\*** | Search over continuous (x, y, θ) states with motion primitives a car can drive | Car-like and Ackermann robots; Nav2's Smac Hybrid-A* |

## A* in 2026 robot stacks

Nav2's Smac planners (2D A*, Hybrid-A*, State Lattice) and NavFn plan on costmaps in milliseconds. Task planners use A* with domain heuristics. Learned components increasingly supply *costs* (semantic traversability from a segmentation network) or *heuristics* (learned cost-to-go), while A* keeps its guarantee of returning the cheapest path under those costs.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[A*_Algorithm]])
```
