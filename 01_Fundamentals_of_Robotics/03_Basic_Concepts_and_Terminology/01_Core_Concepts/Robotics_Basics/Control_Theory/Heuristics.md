---
title: Heuristics
description: Heuristics are strategies or rules of thumb designed to produce good, though not necessarily optimal, solutions to problems, often used in decision-making and problem-solving.
tags:
  - algorithms
  - decision-making
  - problem-solving
  - artificial-intelligence
  - optimization
  - robotics
  - path-planning
layout: default
category: algorithms
author: Jordan_Smith
date: 2025-04-29
permalink: /heuristics/
related:
  - "[[A*_Algorithm]]"
  - "[[Search_Algorithms]]"
  - "[[Optimization_Techniques]]"
  - "[[Decision_Making]]"
  - "[[Problem_Solving]]"
---

# Heuristics

## Intuition: The "Crow Flies" Analogy

Imagine you are in a city trying to reach a destination. You do not have a map, but you can see the destination's general direction. A reasonable strategy is: *at every intersection, pick the road that points most directly toward the destination* — in other words, follow the direction "as the crow flies."

This is a heuristic. It does not guarantee the shortest path (a straight-line road may not exist, or a highway in the "wrong" direction may actually be faster), but it dramatically narrows the search compared to trying every possible route. In robotics path planning, the straight-line (Euclidean) distance to the goal is the most common heuristic — it tells the planner which directions are *probably* better, so it can focus its search there instead of exploring the entire map.

---

**Heuristics** are strategies or rules of thumb designed to produce good, though not necessarily optimal, solutions to problems. They are widely used in decision-making and problem-solving, particularly in situations where finding an optimal solution is computationally expensive or impractical. Heuristics simplify the problem-solving process by providing approximate solutions quickly, making them valuable in fields such as artificial intelligence, optimization, and game theory.

---

## Key Concepts

1. **Approximation**: Heuristics provide approximate solutions to problems, balancing the trade-off between solution quality and computational efficiency.
   <br>

2. **Rule of Thumb**: A simple, experience-based principle that offers guidance in decision-making processes.
   <br>

3. **Admissibility**: A property of heuristics ensuring that they never overestimate the cost to reach the goal, which is crucial for algorithms like A* to guarantee optimal solutions.
   <br>

4. **Greedy Algorithms**: Algorithms that make the locally optimal choice at each step, hoping that these local solutions will lead to a global solution.
   <br>

5. **Search Algorithms**: Heuristics are often used in search algorithms to guide the search process by prioritizing more promising paths or solutions.
   <br>

6. **Evaluation Function**: A function used in heuristic search algorithms to estimate the cost or desirability of a particular state or path.
   <br>

---

## Mathematical Representations

Heuristics can be mathematically represented in various ways, depending on the problem and the algorithm used:

- **Heuristic Function**: In search algorithms, a heuristic function $h(n)$ estimates the cost from a node $n$ to the goal. For example, in A* search:

$$
f(n) = g(n) + h(n)
$$
<br>
  
  where $f(n)$ is the estimated total cost from the start to the goal through node $n$, $g(n)$ is the cost from the start to $n$, and $h(n)$ is the heuristic estimate from $n$ to the goal.
  <br><br>
- **Greedy Choice**: In greedy algorithms, the choice at each step is made to maximize or minimize a local criterion:

$$
\text{Choose } x \text{ such that } f(x) \text{ is minimized or maximized}
$$
<br>
  
  where $f(x)$ is the local objective function.
  <br>

---

## Admissible vs. Inadmissible Heuristics

The distinction between admissible and inadmissible heuristics determines whether an algorithm like A* will find the optimal solution.

### Admissible Heuristics

A heuristic $h(n)$ is **admissible** if it never overestimates the true cost $h^*(n)$ to reach the goal:

$$
h(n) \leq h^*(n) \quad \forall\, n
$$

**Proof that A* with an admissible heuristic finds the optimal path:**

Suppose A* returns a suboptimal goal node $G'$ with cost $f(G') = g(G') > g^*(G)$ (where $G$ is the optimal goal). There must exist an unexpanded node $n$ on the optimal path. For this node: $f(n) = g(n) + h(n) \leq g(n) + h^*(n) = g^*(G) < g(G') = f(G')$. But A* would have expanded $n$ before $G'$ since $f(n) < f(G')$ --- contradiction. Therefore A* must return the optimal path.

### Consistent (Monotone) Heuristics

A stronger condition: $h$ is **consistent** if for every node $n$ and every successor $n'$:

$$
h(n) \leq c(n, n') + h(n') \quad \text{(triangle inequality)}
$$

$$
h(G) = 0 \quad \text{(goal condition)}
$$

Consistency implies admissibility (but not vice versa). With a consistent heuristic, A* never re-expands nodes, which is important for efficiency.

### Inadmissible Heuristics

An inadmissible heuristic may overestimate costs. This means A* is no longer guaranteed to find the optimal path, but it often finds good paths much faster.

**Weighted A*** uses an inflated heuristic $f(n) = g(n) + w \cdot h(n)$ with $w > 1$. This produces a path at most $w$ times the optimal cost, but can be orders of magnitude faster. Typical values: $w = 1.5$--$3.0$.

**Robotics application:** The `sbpl` (Search-Based Planning Lab) library used in ROS implements Anytime Repairing A* (ARA*), which starts with a large $w$ for a fast initial solution, then decreases $w$ iteratively to improve the solution. This is ideal for real-time replanning on mobile robots.

---

## Common Heuristics in Robotics

### Workspace Heuristics (Mobile Robots)

| Heuristic | Formula | Grid Type | Properties |
|---|---|---|---|
| Euclidean distance | $h = \sqrt{\Delta x^2 + \Delta y^2}$ | Any | Admissible, consistent |
| Manhattan distance | $h = |\Delta x| + |\Delta y|$ | 4-connected | Admissible, consistent |
| Diagonal (Octile) | $h = \max(|\Delta x|, |\Delta y|) + (\sqrt{2}-1)\min(|\Delta x|, |\Delta y|)$ | 8-connected | Admissible, consistent |
| Dubins path length | Shortest path with bounded curvature | Continuous | Admissible for car-like robots |

### Configuration-Space Heuristics (Manipulators)

For a robot arm with joint configuration $q \in \mathbb{R}^n$:

- **Joint-space $L^2$ distance:** $h(q, q_{\text{goal}}) = \|q - q_{\text{goal}}\|_2 = \sqrt{\sum_{i=1}^{n}(q_i - q_{i,\text{goal}})^2}$. Admissible if each joint can move independently at unit cost. Fast to compute but ignores obstacles.

- **Weighted joint-space distance:** $h(q, q_{\text{goal}}) = \sqrt{\sum_{i=1}^{n} w_i (q_i - q_{i,\text{goal}})^2}$, where $w_i$ reflects the cost of moving joint $i$ (e.g., base joints are heavier and more expensive to move). Used in MoveIt 2's OMPL interface.

- **Task-space distance:** $h = \|FK(q) - FK(q_{\text{goal}})\|_2$ where $FK$ is the forward kinematics. Admissible when the robot can move at least as fast in joint space as the end-effector moves in task space.

---

## Computational Complexity Trade-offs

The choice of heuristic directly impacts planning speed:

| Algorithm + Heuristic | Time Complexity | Nodes Expanded (typical 1000x1000 grid) | Optimality |
|---|---|---|---|
| Dijkstra (no heuristic, $h = 0$) | $O((V+E) \log V)$ | ~1,000,000 | Optimal |
| A* + Euclidean | $O((V+E) \log V)$ | ~10,000--100,000 | Optimal |
| A* + Weighted ($w=2$) | $O((V+E) \log V)$ | ~1,000--10,000 | $\leq 2 \times$ optimal |
| Greedy Best-First ($g=0$) | $O((V+E) \log V)$ | ~1,000--5,000 | No guarantee |

**Practical guidance:**
- For real-time mobile robot navigation (10--20 Hz replanning), weighted A* with $w = 1.5$--$2.0$ is the standard choice. The suboptimality is acceptable and planning is 10--50x faster than optimal A*.
- For manipulator motion planning, heuristics are less useful because the configuration space is high-dimensional (6--7D) and cluttered. Sampling-based planners (RRT, PRM) are preferred over grid-based search.
- **Memory matters:** A* on a 3D voxel grid (e.g., 200 x 200 x 200 at 5 cm resolution) requires ~32 GB of memory for the open/closed lists. Use hierarchical planning (coarse-to-fine) or lattice-based planners to manage memory.

---

## Advanced Heuristic Techniques

### Precomputed Backward Dijkstra (Perfect Heuristics)

The strongest possible heuristic is the *true* shortest-path distance to the goal — a **perfect heuristic** where $h(n) = h^*(n)$. With a perfect heuristic, A* expands only the nodes on the optimal path (zero wasted expansions).

**How to compute it:** Run Dijkstra's algorithm *backward* from the goal over the full graph and store the resulting distances. This is $O((V+E) \log V)$ as a one-time cost, but every subsequent A* query with that goal is nearly instantaneous.

**When it is practical:**
- The goal is fixed or changes infrequently (e.g., a docking station, charging pad, or home position).
- The map is static or changes slowly (the precomputed distances remain valid).
- Memory is available to store the distance table ($4N$ bytes for $N$ cells with 32-bit floats).

For a 2000 x 2000 grid (typical indoor map at 5 cm resolution), the distance table is ~16 MB — easily fits in memory. The backward Dijkstra takes ~100 ms on modern hardware.

### Pattern Databases

For high-dimensional planning (manipulator arms, multi-robot systems), precomputing a full backward Dijkstra is infeasible. **Pattern databases** store heuristic values for *projections* of the state space onto lower-dimensional subspaces.

For example, a 7-DOF arm's configuration space is 7-dimensional. A pattern database might project onto joints 1--3 only (a 3D subspace), precompute shortest paths in that subspace, and use those distances as a heuristic for the full 7D problem. Multiple pattern databases (different joint subsets) can be combined by taking their maximum — the resulting heuristic is still admissible and often much tighter than Euclidean distance.

**Robotics context:** Pattern databases are used in the SBPL (Search-Based Planning Lab) library for high-dimensional lattice planning, and in multi-robot path planning where the joint state space grows exponentially with the number of robots.

---

## Applications of Heuristics

Heuristics are applied in various fields to solve complex problems efficiently:

- **Artificial Intelligence**: Used in search algorithms, decision-making processes, and game playing to find solutions quickly.
  <br>

- **Optimization**: Employed in optimization problems to find near-optimal solutions when exact solutions are computationally infeasible.
  <br>

- **Game Theory**: Used to model decision-making in strategic situations where players use heuristics to make moves based on incomplete information.
  <br>

- **Robotics**: Applied in path planning and navigation to guide robots through complex environments using approximate methods. A* with Euclidean heuristics is the backbone of most 2D mobile robot navigation stacks.
  <br>

- **Operations Research**: Utilized in scheduling, resource allocation, and logistics to make practical decisions under uncertainty.
  <br>

- **Multi-Robot Coordination**: Conflict-Based Search (CBS) uses heuristics to plan collision-free paths for fleets of warehouse robots (e.g., Amazon Kiva/Proteus systems with 1000+ robots).
  <br>

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

- **List all related algorithms and techniques**:
  ```dataview
  LIST FROM #Algorithms OR #Optimization WHERE contains(file.outlinks, [[Heuristics]])
  ```
