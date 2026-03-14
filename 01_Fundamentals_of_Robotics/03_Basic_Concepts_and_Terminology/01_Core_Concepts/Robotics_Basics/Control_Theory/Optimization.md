---
title: Optimization
description: Optimization is the process of finding the best solution from all feasible solutions, often involving mathematical techniques to maximize or minimize an objective function subject to constraints.
tags:
  - mathematics
  - operations-research
  - algorithms
  - decision-making
  - efficiency
  - modeling
  - computational-mathematics
layout: default
category: mathematics
author: Jordan_Smith_and_le_Chat
date: 2025-05-09
permalink: /optimization/
related:
  - "[[Mathematical_Modeling]]"
  - "[[Operations_Research]]"
  - "[[Algorithms]]"
  - "[[Decision_Making]]"
  - "[[Efficiency]]"
  - "[[Linear_Programming]]"
  - "[[Nonlinear_Programming]]"
  - "[[Constraint_Analysis]]"
  - "[[Computational_Mathematics]]"
  - "[[Optimization_Techniques]]"
  - "[[Objective_Function]]"
  - "[[Feasibility_Analysis]]"
  - "[[Gradient_Descent]]"
  - "[[Convex_Optimization]]"
---

# Optimization

**Optimization** is the process of finding the best solution from all feasible solutions, often involving mathematical techniques to maximize or minimize an objective function subject to constraints. It is widely used in various fields such as engineering, economics, and computer science to improve efficiency, reduce costs, and enhance performance.

---
![[Realtime-Robotics-Optimization.jpg]]
<font size=1>*source: https://www.therobotreport.com/realtime-robotics-optimization-system-workcell-motion-planning-wins-iera-recognition/*</font>
---

## Key Components of Optimization

1. **Objective Function**: A mathematical function that defines the quantity to be maximized or minimized, representing the goal of the optimization problem.
   <br>

2. **Constraints**: Limitations or restrictions that define the feasible region within which the solution must lie.
   <br>

3. **Decision Variables**: Variables that can be controlled or changed to find the optimal solution.
   <br>

4. **Feasibility Analysis**: The process of determining whether a solution satisfies all the constraints of the problem.
   <br>

5. **Optimization Techniques**: Methods and algorithms used to find the optimal solution, such as linear programming, nonlinear programming, and gradient descent.
   <br>

6. **Computational Mathematics**: The use of mathematical models and algorithms to solve optimization problems efficiently.
   <br>

---

## Mathematical Representations

### Linear Programming

Linear programming is a method to achieve the best outcome in a mathematical model whose requirements are represented by linear relationships. The standard form of a linear programming problem is:

$$
\begin{align*}
\text{Maximize } & c^T x \\
\text{Subject to } & A x \leq b \\
& x \geq 0
\end{align*}
$$

where $c$ is the cost vector, $x$ is the decision variable vector, $A$ is the constraint matrix, and $b$ is the resource vector.

<br>

### Nonlinear Programming

Nonlinear programming involves the optimization of a nonlinear objective function subject to nonlinear constraints. The general form is:

$$
\begin{align*}
\text{Minimize } & f(x) \\
\text{Subject to } & g_i(x) \leq 0, \quad i = 1, 2, \ldots, m \\
& h_j(x) = 0, \quad j = 1, 2, \ldots, p
\end{align*}
$$

where $f(x)$ is the objective function, $g_i(x)$ are inequality constraints, and $h_j(x)$ are equality constraints.

<br>

### Gradient Descent

Gradient descent is an iterative optimization algorithm used to find the minimum of a function. The update rule for gradient descent is:

$$
x_{k+1} = x_k - \alpha \nabla f(x_k)
$$

where $x_k$ is the current solution, $\alpha$ is the step size (learning rate), and $\nabla f(x_k)$ is the gradient of the objective function at $x_k$.

---

## Convex vs. Non-Convex Optimization in Robotics

Understanding the distinction between convex and non-convex problems is critical for choosing the right solver and knowing what guarantees you can expect.

### Convex Optimization

A problem is **convex** if the objective function is convex and the feasible set is a convex set. Convex problems have a single global minimum, and efficient solvers can find it in polynomial time.

**Robotics examples of convex problems:**
- **Quadratic programming (QP) for whole-body control:** Given a desired end-effector acceleration, solve for joint torques that minimize effort while satisfying contact constraints. This is a QP solved at 1 kHz on humanoids like the NASA Valkyrie.
- **Minimum-energy trajectory generation:** Given via-points and time allocation, finding a minimum-jerk or minimum-snap polynomial trajectory is a convex QP.
- **Model Predictive Control (MPC) with linear models:** When the robot dynamics are linearized, the MPC problem becomes a QP.

### Non-Convex Optimization

Most real robotics problems are **non-convex** due to nonlinear dynamics, obstacle avoidance constraints, or orientation representations. Non-convex problems may have multiple local minima, and solvers can only guarantee convergence to a local optimum.

**Robotics examples of non-convex problems:**
- **Inverse kinematics** with joint limits and obstacle avoidance (multiple valid configurations).
- **Motion planning** through cluttered environments (the free configuration space is non-convex).
- **Simultaneous Localization and Mapping (SLAM)** as a nonlinear least-squares problem.
- **Trajectory optimization** with full nonlinear dynamics ($M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q) = \tau$).

**Practical implication:** For non-convex problems, always run the solver from multiple initial guesses (multi-start), or use a global method (genetic algorithm, CMA-ES) for initialization followed by a local solver for refinement.

---

## Joint-Space Trajectory Optimization

A central problem in robotics: find a trajectory $q(t)$ that moves the robot from configuration $q_0$ to $q_f$ while minimizing cost and satisfying constraints.

### General Formulation

$$
\min_{q(\cdot), \tau(\cdot)} \int_0^T \mathcal{L}(q(t), \dot{q}(t), \tau(t)) \, dt
$$

subject to:

$$
M(q)\ddot{q} + C(q, \dot{q})\dot{q} + g(q) = \tau \quad \text{(dynamics)}
$$

$$
q(0) = q_0, \quad q(T) = q_f \quad \text{(boundary conditions)}
$$

$$
q_{\min} \leq q(t) \leq q_{\max} \quad \text{(joint limits)}
$$

$$
|\dot{q}(t)| \leq \dot{q}_{\max} \quad \text{(velocity limits)}
$$

$$
|\tau(t)| \leq \tau_{\max} \quad \text{(torque limits)}
$$

$$
d(q(t), \mathcal{O}) \geq d_{\text{safe}} \quad \text{(collision avoidance)}
$$

Common choices for the cost functional $\mathcal{L}$:
- **Minimum time:** $\mathcal{L} = 1$ (minimize $T$)
- **Minimum effort:** $\mathcal{L} = \|\tau\|_2^2$
- **Minimum jerk:** $\mathcal{L} = \|\dddot{q}\|_2^2$ (produces smooth, human-like motion)

For a typical 6-DOF industrial arm, the joint limits might be:
- $\dot{q}_{\max}$: 180--360 deg/s per joint (KUKA KR 6 R900: 360 deg/s on axes 4--6)
- $\tau_{\max}$: 10--300 Nm depending on the joint (larger for shoulder/elbow, smaller for wrist)
- $d_{\text{safe}}$: typically 5--50 mm clearance from obstacles

### Direct Collocation Method

In practice, the continuous problem is discretized into $N$ knot points. Using direct collocation (used by Drake, CasADi, and Trajopt):

$$
\min_{\mathbf{q}_1, \ldots, \mathbf{q}_N, \boldsymbol{\tau}_1, \ldots, \boldsymbol{\tau}_N} \sum_{k=1}^{N} \mathcal{L}(\mathbf{q}_k, \dot{\mathbf{q}}_k, \boldsymbol{\tau}_k) \, \Delta t
$$

subject to collocation constraints (e.g., Hermite-Simpson) that enforce dynamics between knot points. Typical values: $N = 20$--$100$ knot points, $\Delta t = 0.01$--$0.1$ s.

---

## Practical Solver Recommendations

| Solver | Type | Best For | License | Typical Solve Time |
|---|---|---|---|---|
| **OSQP** | Convex QP | MPC, whole-body QP control | Apache 2.0 | <1 ms for small QPs |
| **IPOPT** | Nonlinear (NLP) | Trajectory optimization, NLP | EPL | 10 ms -- 10 s |
| **SNOPT** | Nonlinear (SQP) | Trajectory optimization (sparse) | Commercial | 10 ms -- 5 s |
| **ECOS** | Conic (SOCP) | Convex relaxations, fast embedded | GPL | <1 ms |
| **Gurobi** | LP/QP/MIP | Mixed-integer planning (MICP) | Commercial (free academic) | 1 ms -- 60 s |
| **CasADi** | NLP framework | Automatic differentiation + IPOPT/SNOPT | LGPL | Depends on backend |
| **Drake (Mathematical Program)** | Multi-solver interface | Research prototyping | BSD | Depends on backend |
| **CVXPY** | Modeling language | Rapid prototyping of convex problems | Apache 2.0 | Depends on backend |

**Practitioner guidance:**
- For real-time MPC on a robot (1 kHz control loop), use OSQP or qpOASES --- they are designed for hot-starting and can solve moderate QPs in microseconds.
- For offline trajectory optimization, use IPOPT through CasADi or Drake --- the automatic differentiation saves enormous development time.
- For mixed-integer problems (e.g., contact-mode scheduling in manipulation), use Gurobi with a warm-start strategy.
- Always provide analytical gradients if possible --- finite-difference gradients are 10--100x slower and less accurate.

---

## Applications of Optimization

Optimization is applied in various fields, including:

- **Engineering Design**: Optimizing the design of structures, machines, and systems to meet performance and safety constraints.
  <br>

- **Economics**: Maximizing profits, minimizing costs, and optimizing resource allocation in economic models.
  <br>

- **Computer Science**: Improving the efficiency of algorithms, data structures, and computational processes.
  <br>

- **Operations Research**: Solving complex decision-making problems in logistics, supply chain management, and production planning.
  <br>

- **Finance**: Optimizing investment portfolios, managing risks, and ensuring compliance with regulatory constraints.
  <br>

- **Robotics Trajectory Planning**: Computing time-optimal or energy-optimal joint trajectories subject to dynamics, joint limits, and obstacle avoidance constraints.
  <br>

- **Robot Grasping**: Optimizing contact forces subject to friction cone constraints (a second-order cone program).
  <br>

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #mathematics OR #operations-research WHERE contains(file.outlinks, [[Optimization]])
```
