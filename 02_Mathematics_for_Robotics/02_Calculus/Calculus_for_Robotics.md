---
title: Calculus for Robotics
description: The mathematics of change. Calculus lets a robot reason about how its state evolves through time, how small changes in input ripple to outputs, and how to pick the best action from an infinite continuum of options.
tags:
  - mathematics
  - calculus
  - differential-equations
  - optimization
  - dynamics
  - robotics
  - foundations
layout: default
category: mathematics
author: Jordan_Smith
date: 2025-05-02
permalink: /calculus_for_robotics/
related:
  - "[[Differential_Equations]]"
  - "[[Optimization]]"
  - "[[Jacobian_Matrix]]"
  - "[[Manipulator_Dynamics]]"
  - "[[Linear_Algebra_for_Robotics]]"
---

# Calculus for Robotics

**Calculus** is the mathematics of *change*. Where linear algebra describes static structure — a vector, a rotation, a coordinate frame — calculus describes how those structures evolve: velocities, accelerations, sensitivities, and rates. It is the language in which a robot's dynamics are written and the optimization problems it solves are posed.

> **Etymology.** *Calculus* is Latin for a small pebble — ancient Romans used pebbles to count and compute. The word generalized to any systematic method of calculation, and Newton and Leibniz (independently, ca. 1670) attached it to their new theory of differentials and integrals, which was a *calculus* — a method — for reasoning about quantities that change continuously.

---

## Why calculus owns robotics dynamics

A robot exists in continuous time. Its state — joint positions, velocities, and accelerations — satisfies differential equations derived from Newton's second law. Even when you ultimately run everything on a discrete digital computer at 1 kHz, the underlying model is continuous, and every competent controller, estimator, and planner is derived from a continuous-time analysis and then discretized.

The three core questions calculus lets a robot ask:

1. **How is the state changing right now?** (differentiation — velocity, acceleration, Jacobian)
2. **Where will the state be after time $\Delta t$?** (integration — forward simulation, dead reckoning)
3. **What input makes some cost as small as possible?** (optimization — trajectory optimization, MPC, gradient descent)

---

## Differentiation

The **derivative** of $f : \mathbb{R} \to \mathbb{R}$ at a point is the instantaneous rate of change:

$$
f'(x) = \lim_{h \to 0} \frac{f(x+h) - f(x)}{h}
$$

For multivariable $f : \mathbb{R}^n \to \mathbb{R}^m$, the generalization is the **Jacobian matrix** of partial derivatives:

$$
J(\mathbf{x}) = \frac{\partial f}{\partial \mathbf{x}} = \begin{bmatrix}
\frac{\partial f_1}{\partial x_1} & \cdots & \frac{\partial f_1}{\partial x_n} \\
\vdots & \ddots & \vdots \\
\frac{\partial f_m}{\partial x_1} & \cdots & \frac{\partial f_m}{\partial x_n}
\end{bmatrix} \in \mathbb{R}^{m \times n}
$$

This single object threads through the entire robotics stack.

### Where the Jacobian shows up

| Context | Jacobian of | Interpretation |
|---|---|---|
| Manipulator kinematics | end-effector pose w.r.t. joints | Maps $\dot{\mathbf{q}}$ to end-effector twist $\mathbf{v}$ |
| Dynamics linearization | $f(\mathbf{x}, \mathbf{u})$ around $(\mathbf{x}^*, \mathbf{u}^*)$ | Local state-space matrices $A$ and $B$ |
| [[Extended_Kalman_Filter]] | nonlinear dynamics/measurement | Propagation of covariance |
| Gradient descent | loss w.r.t. parameters | Direction of steepest descent |
| Backpropagation | network output w.r.t. weights | Chain-ruled through the graph |

**Chain rule** is the single most important theorem in applied calculus. For $f = g \circ h$:

$$
\frac{\partial f}{\partial \mathbf{x}} = \frac{\partial g}{\partial \mathbf{h}} \frac{\partial h}{\partial \mathbf{x}}
$$

Every deep learning framework (PyTorch, JAX, TensorFlow) is fundamentally a well-engineered chain-rule machine.

### Second derivatives and the Hessian

For $f : \mathbb{R}^n \to \mathbb{R}$ the **Hessian** is the matrix of second partials:

$$
H_{ij} = \frac{\partial^2 f}{\partial x_i \, \partial x_j}
$$

The Hessian captures local curvature. At a minimum of $f$, the gradient is zero and the Hessian is positive semi-definite. Newton's method for optimization uses the Hessian directly; quasi-Newton methods (BFGS, L-BFGS) build cheap approximations.

---

## Integration

The **integral** is the inverse of the derivative. Geometrically, $\int_a^b f(x)\,dx$ is the signed area under the curve; physically, it accumulates an instantaneous rate into a total quantity.

### Numerical integration for robots

Almost no robotics problem has a closed-form integral. Numerical schemes trade accuracy against cost:

| Method | Order | Use case |
|---|---|---|
| Forward Euler | 1 | Fast prototyping; unstable for stiff systems |
| Midpoint / RK2 | 2 | Good balance for most on-board control loops |
| Runge-Kutta 4 | 4 | Standard for offline simulation |
| Adams-Bashforth | multi-step | Long trajectories with smooth dynamics |
| Symplectic integrators | varies | Rigid-body mechanics — conserve energy |

For a dynamics $\dot{\mathbf{x}} = f(\mathbf{x}, \mathbf{u})$:

$$
\mathbf{x}_{k+1} = \mathbf{x}_k + \Delta t \cdot f(\mathbf{x}_k, \mathbf{u}_k) \quad \text{(forward Euler)}
$$

At 1 kHz control rates ($\Delta t = 1\,\text{ms}$) forward Euler is often fine; at lower rates you typically want RK4. Physics simulators like MuJoCo use specialized implicit integrators to stay stable at larger time steps.

---

## Ordinary differential equations (ODEs)

The manipulator equation is the defining ODE of robot dynamics:

$$
M(\mathbf{q}) \ddot{\mathbf{q}} + C(\mathbf{q}, \dot{\mathbf{q}}) \dot{\mathbf{q}} + \mathbf{g}(\mathbf{q}) = \boldsymbol{\tau}
$$

where $M$ is the inertia matrix, $C$ captures Coriolis/centrifugal effects, $\mathbf{g}$ is gravity, and $\boldsymbol{\tau}$ is the applied joint torque. This is a second-order nonlinear ODE — second order in $\mathbf{q}$, nonlinear because $M$, $C$, and $\mathbf{g}$ depend on $\mathbf{q}$.

**Forward dynamics** solves for $\ddot{\mathbf{q}}$ given $\boldsymbol{\tau}$ — used in simulation. **Inverse dynamics** solves for $\boldsymbol{\tau}$ given a desired $\ddot{\mathbf{q}}$ — used in computed-torque and feedforward control. Inverse dynamics is computationally cheaper (no matrix inverse) and enables the Recursive Newton-Euler Algorithm (RNEA) running in $O(n)$ on an $n$-joint chain.

### State-space form

A controller usually lives in first-order form. Let $\mathbf{x} = [\mathbf{q}^T, \dot{\mathbf{q}}^T]^T$:

$$
\dot{\mathbf{x}} = \begin{bmatrix} \dot{\mathbf{q}} \\ M^{-1}(\boldsymbol{\tau} - C\dot{\mathbf{q}} - \mathbf{g}) \end{bmatrix} = f(\mathbf{x}, \boldsymbol{\tau})
$$

Linearize around $(\mathbf{x}^*, \boldsymbol{\tau}^*)$ and you get:

$$
\delta \dot{\mathbf{x}} = A \, \delta \mathbf{x} + B \, \delta \boldsymbol{\tau}
$$

which is the setup for LQR, pole-placement, and eigenvalue stability analysis.

---

## Calculus of variations and optimal control

Trajectory optimization asks: among all admissible trajectories $\mathbf{x}(t)$, find the one minimizing a cost functional. The canonical form:

$$
\min_{\mathbf{x}(t), \mathbf{u}(t)} \quad \phi(\mathbf{x}(t_f)) + \int_{t_0}^{t_f} L(\mathbf{x}(t), \mathbf{u}(t), t)\,dt
$$

subject to $\dot{\mathbf{x}} = f(\mathbf{x}, \mathbf{u})$ and path constraints $\mathbf{g}(\mathbf{x}, \mathbf{u}) \leq \mathbf{0}$.

Pontryagin's Minimum Principle gives necessary conditions via a Hamiltonian and costate equations. In practice, for robotics, the problem is **transcribed** into a finite-dimensional nonlinear program (direct collocation, multiple shooting, direct transcription) and solved with solvers like IPOPT or SNOPT.

- **Model Predictive Control (MPC)** solves a short-horizon version of this problem at each time step, applies the first control, and re-plans.
- **iLQR / DDP** are specialized methods that exploit the dynamics structure for fast local optimization.

---

## Gradient-based optimization

For a smooth cost $J : \mathbb{R}^n \to \mathbb{R}$:

$$
\mathbf{x}_{k+1} = \mathbf{x}_k - \alpha \nabla J(\mathbf{x}_k)
$$

This simple update, plus its countless variants (momentum, Adam, RMSProp), trains every neural network on Earth. The robotics applications:

- **Policy gradient RL:** $\nabla_\theta \mathbb{E}[\text{return}]$ under a stochastic policy $\pi_\theta$
- **Inverse kinematics:** minimize $\lVert f(\mathbf{q}) - \mathbf{x}_{\text{desired}} \rVert^2$ over $\mathbf{q}$
- **Bundle adjustment / pose-graph SLAM:** Gauss-Newton on reprojection error
- **Trajectory smoothing:** CHOMP, TrajOpt, GPMP

**Newton's method** uses second-order information:

$$
\mathbf{x}_{k+1} = \mathbf{x}_k - H^{-1} \nabla J
$$

Fast local convergence (quadratic) but expensive to form and invert the Hessian; Gauss-Newton and Levenberg-Marquardt approximate it cheaply for least-squares costs.

---

## Stability: the calculus of Lyapunov

**Lyapunov's direct method** is calculus turned toward proving that a dynamical system settles down to equilibrium. Given $\dot{\mathbf{x}} = f(\mathbf{x})$ with equilibrium at the origin, find a positive-definite function $V(\mathbf{x})$ (a "Lyapunov function" — think of an energy) whose derivative along trajectories,

$$
\dot{V}(\mathbf{x}) = \nabla V(\mathbf{x}) \cdot f(\mathbf{x}),
$$

is negative definite. Then the origin is asymptotically stable.

This one idea anchors:

- Stability proofs for PID controllers
- Convergence arguments for model-reference adaptive control
- Control-Lyapunov functions for constructive controller design
- Barrier functions for safety-critical control

---

## Calculus in perception and learning

- **Image gradients** detect edges (Sobel, Scharr, Canny)
- **Optical flow** estimates pixel velocities via the brightness constancy constraint $\frac{\partial I}{\partial t} + \nabla I \cdot \mathbf{v} = 0$
- **Backpropagation** propagates loss gradients through millions of parameters via reverse-mode automatic differentiation
- **Diffusion models** for robot policy learning are built on stochastic differential equations

---

## Recommended references

- James Stewart, *Calculus* — the canonical undergraduate text
- Michael Spivak, *Calculus on Manifolds* — rigorous multivariable; essential for SE(3)
- Jorge Nocedal & Stephen Wright, *Numerical Optimization* — the standard reference
- Russ Tedrake, *Underactuated Robotics* (MIT online) — calculus of variations for real robots

---

## Dataview Plugin Features

### List of related concepts

```dataview
LIST FROM #mathematics OR #calculus WHERE contains(file.outlinks, [[Calculus_for_Robotics]])
```
