---
title: Optimization for Robotics
description: The mathematics of picking the best option from a continuum. Underpins trajectory planning, model-predictive control, inverse kinematics, machine learning, SLAM back-ends, and nearly every modern robot autonomy stack.
tags:
  - mathematics
  - optimization
  - trajectory-optimization
  - control
  - robotics
  - numerical-methods
layout: default
category: mathematics
author: Jordan_Smith
date: 2025-05-02
permalink: /optimization_for_robotics/
related:
  - "[[Calculus_for_Robotics]]"
  - "[[Linear_Algebra_for_Robotics]]"
  - "[[Model_Predictive_Control]]"
  - "[[Trajectory_Optimization]]"
  - "[[Gradient_Descent]]"
---

# Optimization for Robotics

**Optimization** is the mathematical discipline of finding the input to a function that makes the output as small (or as large) as possible, subject to constraints. In robotics it is the common machinery behind trajectory planning, model-predictive control, inverse kinematics, localization, SLAM back-ends, controller synthesis, and every neural-network-based perception or policy.

> **Etymology.** *Optimize* is from Latin *optimus*, "best." *Minimize* from Latin *minimus*, "least." *Maximize* from *maximus*, "greatest."

---

## The canonical form

$$
\min_{\mathbf{x} \in \mathcal{X}} f(\mathbf{x}) \quad \text{subject to} \quad g_i(\mathbf{x}) \leq 0,\; h_j(\mathbf{x}) = 0
$$

- $\mathbf{x}$ — decision variables (joint angles, motor torques, waypoints, neural weights)
- $f$ — objective / cost function (travel time, energy, error)
- $g_i$ — inequality constraints (joint limits, collision avoidance, actuator bounds)
- $h_j$ — equality constraints (dynamics, kinematic closure, task targets)

Every robotics problem posed as "find the best X subject to Y" fits this mold.

---

## Taxonomy

| Class | Objective | Constraints | Standard solvers |
|---|---|---|---|
| **Linear programming (LP)** | Linear | Linear | simplex, interior-point |
| **Quadratic programming (QP)** | Quadratic | Linear | OSQP, qpOASES, CVXOPT |
| **Second-order cone programming (SOCP)** | Linear | Conic | ECOS, MOSEK |
| **Semidefinite programming (SDP)** | Linear | Matrix PSD | SDPA, MOSEK |
| **Nonlinear programming (NLP)** | Nonlinear | Nonlinear | IPOPT, SNOPT, Ceres |
| **Mixed-integer (MIP/MINLP)** | Any | Any, integer vars | Gurobi, CPLEX, SCIP |
| **Convex (any of the above with convex $f$, $g$)** | Convex | Convex | CVXPY is the front-end |

**The convex / non-convex split is the single most important dividing line.** A convex problem has one global minimum that any descent method will find; a non-convex problem may have many local minima, and initialization matters enormously.

---

## Unconstrained gradient methods

The foundation. For smooth $f : \mathbb{R}^n \to \mathbb{R}$:

### Gradient descent

$$
\mathbf{x}_{k+1} = \mathbf{x}_k - \alpha \nabla f(\mathbf{x}_k)
$$

- **Pros:** simple, scales to millions of variables, no Hessian needed.
- **Cons:** linear convergence, requires well-tuned step size $\alpha$, struggles with ill-conditioned problems.

Variants: momentum (Polyak heavy-ball), Nesterov accelerated gradient, RMSProp, Adam. Every neural-network training loop uses one of these.

### Newton's method

$$
\mathbf{x}_{k+1} = \mathbf{x}_k - H^{-1}(\mathbf{x}_k) \nabla f(\mathbf{x}_k)
$$

- Quadratic convergence near the minimum.
- Requires forming and inverting the Hessian $H = \nabla^2 f$.
- Line search or trust region needed for global convergence.

### Quasi-Newton (BFGS, L-BFGS)

Approximate the Hessian with cheap rank-one or rank-two updates based on gradient history. L-BFGS is memory-efficient and a common default for mid-size unconstrained problems.

### Gauss-Newton (least-squares only)

For $f(\mathbf{x}) = \tfrac{1}{2} \lVert \mathbf{r}(\mathbf{x}) \rVert^2$:

$$
\mathbf{x}_{k+1} = \mathbf{x}_k - (J^T J)^{-1} J^T \mathbf{r}, \quad J = \frac{\partial \mathbf{r}}{\partial \mathbf{x}}
$$

Approximates Newton by ignoring second-order terms in the Hessian. Fast and cheap; the dominant method in SLAM back-ends.

### Levenberg-Marquardt

Damped Gauss-Newton:

$$
(J^T J + \lambda I) \Delta \mathbf{x} = -J^T \mathbf{r}
$$

Interpolates between Gauss-Newton ($\lambda \to 0$) and gradient descent ($\lambda \to \infty$). Robust when the Jacobian is ill-conditioned. Default in Ceres, g2o, GTSAM.

---

## Constrained optimization

### Karush-Kuhn-Tucker (KKT) conditions

Necessary conditions for a local minimum of a constrained problem. The Lagrangian:

$$
\mathcal{L}(\mathbf{x}, \boldsymbol{\lambda}, \boldsymbol{\mu}) = f(\mathbf{x}) + \sum_i \lambda_i g_i(\mathbf{x}) + \sum_j \mu_j h_j(\mathbf{x})
$$

At a minimum: $\nabla_{\mathbf{x}} \mathcal{L} = 0$, primal feasibility, dual feasibility ($\lambda_i \geq 0$), and complementary slackness ($\lambda_i g_i = 0$). Every constrained solver is an algorithm to drive these residuals to zero.

### Interior-point methods

Replace inequality constraints with a barrier function that blows up at the boundary, then follow a central path as the barrier weight decreases. Default for convex optimization at scale; IPOPT is an interior-point NLP solver used widely in trajectory optimization.

### Sequential quadratic programming (SQP)

At each iterate, solve a QP that approximates the Lagrangian and constraints. SNOPT is an industrial-grade SQP. Often faster than IPOPT on problems with fewer constraints and good warm-starts — which is why it is popular for real-time MPC.

### Augmented Lagrangian and ADMM

Hybrid methods for large-scale, structured problems. ADMM (Alternating Direction Method of Multipliers) is popular for distributed optimization.

---

## Trajectory optimization

The canonical robotics NLP: find a state-and-control trajectory that minimizes a cost and satisfies dynamics + constraints.

$$
\min_{\mathbf{x}(t),\mathbf{u}(t)} \; \phi(\mathbf{x}(t_f)) + \int_{t_0}^{t_f} L(\mathbf{x}, \mathbf{u}, t)\,dt
$$

subject to $\dot{\mathbf{x}} = f(\mathbf{x}, \mathbf{u})$, $\mathbf{g}(\mathbf{x}, \mathbf{u}) \leq 0$.

**Transcription methods** turn this infinite-dimensional problem into a finite NLP:

| Method | Approach | Tradeoff |
|---|---|---|
| **Direct shooting** | Parameterize $\mathbf{u}(t)$, simulate to get $\mathbf{x}(t)$, optimize | Small variable count, shooting instability |
| **Multiple shooting** | Break into intervals, match states at knots | More variables, much better conditioning |
| **Direct collocation** | Variables = state + control at collocation points | Sparse KKT system, standard for modern solvers |
| **Differential dynamic programming (DDP / iLQR)** | Dynamic-programming-style sweeps | Very fast for unconstrained, local |

Tools: CasADi + IPOPT, Drake's trajectory optimizer, Crocoddyl, Altro.

---

## Model-predictive control (MPC)

Solve a short-horizon trajectory optimization at every control tick, apply only the first control input, re-solve next cycle. Standard formulation at each tick:

$$
\min_{\mathbf{u}_0, \ldots, \mathbf{u}_{N-1}} \sum_{k=0}^{N-1} \ell(\mathbf{x}_k, \mathbf{u}_k) + \phi(\mathbf{x}_N)
$$

subject to $\mathbf{x}_{k+1} = f(\mathbf{x}_k, \mathbf{u}_k)$, input/state constraints. For linear dynamics and quadratic cost this is a QP — solvable at kHz rates (OSQP, qpOASES). For nonlinear dynamics, sequential QP with warm-starting keeps real-time rates feasible.

See [[Model_Predictive_Control]].

---

## Inverse kinematics as optimization

Non-closed-form IK is an NLP:

$$
\min_{\mathbf{q}} \lVert f(\mathbf{q}) - \mathbf{x}_{\text{target}} \rVert^2 + \lambda \lVert \mathbf{q} - \mathbf{q}_{\text{rest}} \rVert^2
$$

subject to joint limits. Solved with Gauss-Newton, Levenberg-Marquardt, or gradient descent on the twist error. See [[Inverse_Kinematics]].

---

## Convex optimization in one picture

A function $f$ is *convex* if the line segment between any two graph points lies above the graph. Convex problems are special because:

- Every local minimum is a global minimum.
- Strong duality holds.
- Polynomial-time solvers exist with worst-case guarantees.

**Recognizing convexity pays off in robotics:** whenever a problem can be cast as a QP (MPC with linear dynamics, distance-to-collision penalty, control effort penalty), we get real-time solvers. Whenever it can't (six-DoF IK over joint limits), we use local NLP and accept local minima.

Tools: CVXPY (Python), CVX (MATLAB) for prototyping; OSQP, ECOS, qpOASES, MOSEK for production.

---

## Practical tooling

| Tool | Language | Strength |
|---|---|---|
| **CVXPY** | Python | Prototyping; automatic convexity checks |
| **CasADi** | C++/Python | Automatic differentiation + NLP front-end |
| **Ceres Solver** | C++ | Nonlinear least-squares (SLAM, calibration) |
| **GTSAM** | C++/Python | Factor graphs for estimation |
| **IPOPT** | C++/Fortran | Industrial interior-point NLP |
| **SNOPT** | Fortran | Industrial SQP |
| **OSQP** | C | Embedded QP, sub-ms solve times |
| **Drake** | C++/Python | Systems-level framework with optimization built in |
| **PyTorch / JAX** | Python | Stochastic gradient on learned models |

---

## References

- Boyd & Vandenberghe, *Convex Optimization* (free PDF)
- Nocedal & Wright, *Numerical Optimization*
- Betts, *Practical Methods for Optimal Control and Estimation Using Nonlinear Programming*
- Tedrake, *Underactuated Robotics* (MIT open course) — applications
- Rawlings, Mayne & Diehl, *Model Predictive Control: Theory, Computation, and Design*

---

## Dataview

```dataview
LIST FROM #optimization OR #trajectory-optimization WHERE contains(file.outlinks, [[Optimization_for_Robotics]])
```
