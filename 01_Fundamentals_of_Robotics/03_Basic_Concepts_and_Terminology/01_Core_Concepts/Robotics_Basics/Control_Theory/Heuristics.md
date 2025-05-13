---
title: Heuristics
description: Heuristics are strategies or rules of thumb designed to produce good, though not necessarily optimal, solutions to problems, often used in decision-making and problem-solving.
tags:
  - algorithms
  - decision-making
  - problem-solving
  - artificial-intelligence
  - optimization
  - glossary-term
layout: default
category: algorithms
author: Jordan_Smith_and_le_Chat
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

## Applications of Heuristics

Heuristics are applied in various fields to solve complex problems efficiently:

- **Artificial Intelligence**: Used in search algorithms, decision-making processes, and game playing to find solutions quickly.
  <br>

- **Optimization**: Employed in optimization problems to find near-optimal solutions when exact solutions are computationally infeasible.
  <br>

- **Game Theory**: Used to model decision-making in strategic situations where players use heuristics to make moves based on incomplete information.
  <br>

- **Robotics**: Applied in path planning and navigation to guide robots through complex environments using approximate methods.
  <br>

- **Operations Research**: Utilized in scheduling, resource allocation, and logistics to make practical decisions under uncertainty.
  <br>

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

- **List all related algorithms and techniques**:
  ```dataview
  LIST FROM #Algorithms OR #Optimization WHERE contains(file.outlinks, [[Heuristics]])
