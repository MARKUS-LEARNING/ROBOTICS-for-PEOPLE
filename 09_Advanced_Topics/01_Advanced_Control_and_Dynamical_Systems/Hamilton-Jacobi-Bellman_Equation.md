---
title: Hamilton-Jacobi-Bellman Equation
description: "The Hamilton–Jacobi–Bellman (HJB) equation is the partial differential equation the optimal cost-to-go satisfies in continuous-time optimal control. Covers where it comes from (Bellman's principle of optimality), a solved scalar example, how it reduces to the Riccati equation for LQR, why it is hard (curse of dimensionality), and its discrete-time cousin, the Bellman equation that reinforcement learning approximates."
aliases:
  - HJB Equation
  - HJB
  - Bellman Equation
tags:
  - optimal-control
  - reinforcement-learning
  - control
type: concept
level: research
subfield:
  - control
  - mathematics
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Optimal_Control]]"
related:
  - "[[Pontryagin's_Minimum_Principle]]"
  - "[[Reinforcement_Learning]]"
  - "[[Markov_Decision_Process]]"
key_takeaways:
  - "The HJB equation states that the optimal cost-to-go V(x) balances the best instantaneous cost against the rate of change of V along the dynamics: 0 = min_u [ℓ(x, u) + ∇V·f(x, u)] (infinite horizon)."
  - "For ẋ = u with cost ∫(x² + u²) dt, V(x) = x² and the optimal control is u = −x, which can be checked by substitution."
  - "Solving HJB on a grid is exponential in the state dimension; reinforcement learning sidesteps this by approximating V or the policy with neural networks trained on samples."
sources:
  - "Bertsekas, D. P. (2017). Dynamic Programming and Optimal Control, 4th edition. https://www.athenasc.com/dpbook.html"
  - "Liberzon, D. (2011). Calculus of Variations and Optimal Control Theory: A Concise Introduction, chapter 5. http://liberzon.csl.illinois.edu/teaching/cvoc.pdf"
  - "Tedrake, R. (2026). Underactuated Robotics, chapter: Dynamic Programming. https://underactuated.csail.mit.edu/dp.html"
  - "Sutton, R. S. and Barto, A. G. (2018). Reinforcement Learning: An Introduction, 2nd edition. http://incompleteideas.net/book/the-book-2nd.html"
---

# Hamilton-Jacobi-Bellman Equation

The **Hamilton–Jacobi–Bellman (HJB) equation** is the central equation of continuous-time optimal control. Its solution, the **value function** $V(\mathbf x)$, gives the minimum cost achievable from every state, and from it the optimal feedback controller follows immediately.

> **Etymology.** It joins two traditions: William Rowan Hamilton's and Carl Jacobi's 19th-century equation from classical mechanics, and Richard Bellman's dynamic programming (1950s). Bellman chose the name "dynamic programming" partly, he later wrote, because it sounded impressive and hard to object to.

## From Bellman's principle

**Principle of optimality**: the remainder of an optimal path is itself optimal. Over a short interval $dt$, the optimal cost from $\mathbf x$ is the cost of the next instant plus the optimal cost from wherever that leads. Letting $dt \to 0$ gives, for an infinite-horizon problem with dynamics $\dot{\mathbf x} = f(\mathbf x, \mathbf u)$ and running cost $\ell$:

$$0 = \min_{\mathbf u}\ \Big[\, \ell(\mathbf x, \mathbf u) + \nabla V(\mathbf x)^\top f(\mathbf x, \mathbf u) \,\Big]$$

(Finite-horizon problems add $\partial V/\partial t$ and a terminal condition.) The minimising $\mathbf u$ at each state *is* the optimal policy.

## Worked example

Dynamics $\dot x = u$, cost $\int_0^\infty (x^2 + u^2)\, dt$. Guess $V(x) = p x^2$. Then

$$0 = \min_u \big[x^2 + u^2 + 2px\,u\big].$$

Setting the derivative in $u$ to zero gives $u^* = -px$. Substituting: $0 = x^2 + p^2x^2 - 2p^2x^2 = (1 - p^2)x^2$, so $p = 1$ (taking the positive root, since cost is positive).

**Result:** $V(x) = x^2$ and $u^* = -x$. Check from $x_0 = 2$: the closed loop $\dot x = -x$ gives $x = 2e^{-t}$, and the cost is $\int_0^\infty (4e^{-2t} + 4e^{-2t})\,dt = 4 = V(2)$. The value function correctly predicts the cost.

For general linear-quadratic problems, $V = \mathbf x^\top P \mathbf x$ and HJB reduces to the Riccati equation behind LQR (see [[Optimal_Control]]).

## Why it is hard

For nonlinear systems HJB rarely has a closed-form solution. Solving it numerically on a grid needs a number of points exponential in the state dimension: 100 points per dimension for a 12-dimensional quadruped state is $10^{24}$ points. This "curse of dimensionality" (Bellman's own phrase) limits exact methods to systems with a handful of states. Grid-based solvers are still used for low-dimensional safety analysis (reachability).

## The discrete cousin: Bellman's equation and RL

In discrete time and with randomness, the same principle gives the **Bellman equation**:

$$V(s) = \max_a\ \Big[r(s, a) + \gamma\, \mathbb E\, V(s')\Big]$$

the heart of [[Reinforcement_Learning]] and [[Markov_Decision_Process|MDPs]]. Value-based RL (Q-learning, the critics in actor-critic methods) approximates its solution with neural networks from sampled experience, which is how robot policies escape the curse of dimensionality without a model.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Hamilton-Jacobi-Bellman_Equation]])
```
