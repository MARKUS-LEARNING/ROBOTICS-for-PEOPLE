---
title: Path Planning Algorithms
description: "A map of the algorithm families robots use to find collision-free paths: graph search on grids and lattices, sampling-based planners (PRM, RRT, RRT*), potential fields, trajectory optimisation (CHOMP, GPU-parallel cuRobo, MPC/MPPI), and learned planners (diffusion-based motion generation). Includes a computed example of why grids fail in high dimensions and a guide to choosing a planner in 2026."
aliases:
  - Path Planning Methods
  - Motion Planning Algorithms
tags:
  - path-planning
  - motion-planning
  - navigation
  - graph-traversal
type: overview
level: intermediate
subfield:
  - planning
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Path_Planning]]"
  - "[[Configuration_Space]]"
related:
  - "[[A*_Algorithm]]"
  - "[[Dijkstra's_Algorithm]]"
  - "[[Sampling-Based_Planning]]"
  - "[[Probabilistic_Roadmaps]]"
  - "[[MoveIt2]]"
  - "[[Nav2_Navigation]]"
key_takeaways:
  - "Low-dimensional problems (a mobile base on a map) are solved by graph search on grids or lattices; high-dimensional ones (a 7-joint arm) need sampling-based or optimisation-based planners, because grids grow exponentially with dimension."
  - "Sampling planners (PRM, RRT, RRT*) are probabilistically complete and, for the starred versions, asymptotically optimal; optimisation planners produce smooth trajectories fast but can get stuck in local minima, which GPU parallelism (cuRobo) largely overcomes."
  - "In 2026 learned components propose paths or costs (diffusion motion models, learned traversability), while classical planners and collision checkers verify them."
sources:
  - "LaValle, S. M. (2006). Planning Algorithms. http://lavalle.pl/planning/"
  - "Kavraki, L. E., Svestka, P., Latombe, J.-C. and Overmars, M. H. (1996). Probabilistic roadmaps for path planning in high-dimensional configuration spaces. https://ieeexplore.ieee.org/document/508439"
  - "LaValle, S. M. (1998). Rapidly-exploring random trees: A new tool for path planning. http://msl.cs.uiuc.edu/~lavalle/papers/Lav98c.pdf"
  - "Karaman, S. and Frazzoli, E. (2011). Sampling-based Algorithms for Optimal Motion Planning. https://arxiv.org/abs/1105.1186"
  - "Zucker, M. et al. (2013). CHOMP: Covariant Hamiltonian Optimization for Motion Planning. https://journals.sagepub.com/doi/10.1177/0278364913488805"
  - "Sundaralingam, B. et al. (2023). cuRobo: Parallelized Collision-Free Minimum-Jerk Robot Motion Generation. https://arxiv.org/abs/2310.17274"
  - "Carvalho, J. et al. (2023). Motion Planning Diffusion: Learning and Planning of Robot Motions with Diffusion Models. https://arxiv.org/abs/2308.01557"
---

# Path Planning Algorithms

A **path planner** finds a collision-free way from a start to a goal. Which algorithm works depends mostly on one number, the dimension of the robot's [[Configuration_Space|configuration space]] (2 or 3 for a mobile base, 6–7 for an arm, 20–40 for a humanoid), and on whether the result must be smooth, fast, optimal or simply found.

> **Etymology.** *Path* is from Old English *pæth*; *planning* from Latin *planum*, "flat surface", via the idea of laying out a design on a flat drawing. In robotics, a *path* is geometric (where to go), and a *trajectory* adds timing (when to be there).

## The families

| Family | Idea | Strengths | Weaknesses | Examples |
|---|---|---|---|---|
| **Graph search** | Discretise space into a graph and search it | Complete and optimal at the chosen resolution; predictable | Exponential in dimension | [[Dijkstra's_Algorithm\|Dijkstra]], [[A*_Algorithm\|A*]], D* Lite, state lattices |
| **Sampling-based** | Sample random collision-free configurations and connect them | Scales to high dimensions; only needs a collision checker | Paths are jerky until smoothed; no guarantee in finite time | [[Probabilistic_Roadmaps\|PRM]], RRT, RRT-Connect, RRT*, BIT* (OMPL) |
| **Potential fields** | Goal attracts, obstacles repel; follow the gradient | Very fast, reactive | Local minima and oscillations | Reactive avoidance layers |
| **Optimisation** | Treat the trajectory as variables; minimise cost subject to collision and limits | Smooth, dynamically feasible | Local minima; needs a good start | CHOMP, TrajOpt, STOMP, cuRobo, MPC, MPPI |
| **Learned** | Learn to propose paths or costs from data | Fast proposals; captures preferences | No guarantees on its own | Diffusion motion planners, learned heuristics and costmaps |

## Worked example: why grids fail for arms

Discretise each joint into 100 values:

| Robot | Dimensions | Grid cells |
|---|---|---|
| Mobile base (x, y) | 2 | $100^2 = 10^4$ |
| Base with heading (x, y, θ) | 3 | $10^6$ |
| 7-joint arm | 7 | $100^7 = 10^{14}$ |

Even checking one cell per nanosecond, $10^{14}$ cells take about 28 hours, so an arm cannot be planned on a grid. Sampling-based planners avoid building the grid at all: they sample a few thousand configurations, keep the collision-free ones, and connect them. Their guarantee is **probabilistic completeness**: if a path exists, the probability of finding it approaches 1 as samples grow. RRT* and PRM* (Karaman and Frazzoli, 2011) add **asymptotic optimality**: the path cost converges to the optimum.

## Choosing a planner (2026)

| Problem | Good default |
|---|---|
| Mobile robot on a 2D map | A* / Smac planners or NavFn in [[Nav2_Navigation\|Nav2]]; MPPI or Regulated Pure Pursuit to follow |
| Car-like vehicle | Hybrid-A* or state lattice, then MPC |
| Robot arm, static scene | OMPL's RRT-Connect through [[MoveIt2]], then time-parameterisation; or cuRobo on a GPU |
| Arm in a changing scene | GPU-parallel trajectory optimisation (cuRobo), replanning at high rate |
| Humanoid whole body | Optimisation and MPC over contacts, often with a learned policy for locomotion |
| Contact-rich manipulation | Plan transit with a planner; use a learned policy for the contact phase |

## Learned planning (2023–2026)

**Diffusion models** trained on many good trajectories can generate collision-free motions in one shot (Motion Planning Diffusion, 2023), or warm-start an optimiser. Learned traversability turns camera images into costs for graph search. Large robot policies increasingly produce motion directly (see [[Diffusion_Policy]]). The common pattern is that learning proposes and classical methods check: collision checking, joint limits and dynamics constraints stay explicit.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Path_Planning_Algorithms]])
```
