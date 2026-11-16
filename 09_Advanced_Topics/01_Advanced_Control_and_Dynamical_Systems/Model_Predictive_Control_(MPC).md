---
title: Model Predictive Control (MPC)
description: "Model predictive control repeatedly solves a short-horizon optimal-control problem with an explicit model and constraints, applies the first command, and re-solves. Covers the receding-horizon idea, the standard formulation, a simulated example of how actuator limits change the motion, solvers (QP, nonlinear, sampling-based MPPI), MPC for legged and humanoid robots, and MPC with learned policies."
aliases:
  - MPC
  - Model Predictive Control
  - Receding Horizon Control
tags:
  - mpc
  - optimal-control
  - control
  - optimization
type: method
level: advanced
subfield:
  - control
  - planning
status: keystone
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[State_Space_Representation]]"
  - "[[Optimization_and_Loss_Functions]]"
related:
  - "[[Optimal_Control]]"
  - "[[Nav2_Navigation]]"
  - "[[Legged_Robots]]"
  - "[[Humanoid_Robots]]"
  - "[[Reinforcement_Learning_for_Robots]]"
key_takeaways:
  - "MPC plans the next N steps by optimisation, executes only the first, then plans again from the new measured state: feedback through repeated planning."
  - "Constraints are first-class: with acceleration limited to 1 m/s², a simulated 1 m move takes 1.9 s instead of 1.3 s but never exceeds the limit; at 0.3 m/s² it takes 4.3 s."
  - "Fast QP and nonlinear solvers (OSQP, acados) and sampling methods (MPPI) run MPC at 100–1,000 Hz on robots; in 2026 it is the standard for legged and humanoid whole-body control alongside RL policies."
sources:
  - "Rawlings, J. B., Mayne, D. Q. and Diehl, M. M. (2020). Model Predictive Control: Theory, Computation, and Design, 2nd edition. https://sites.engineering.ucsb.edu/~jbraw/mpc/"
  - "Stellato, B. et al. (2017). OSQP: An Operator Splitting Solver for Quadratic Programs. https://arxiv.org/abs/1711.08013"
  - "Kim, D. et al. (2019). Highly Dynamic Quadruped Locomotion via Whole-Body Impulse Control and Model Predictive Control. https://arxiv.org/abs/1909.06586"
  - "Williams, G., Aldrich, A. and Theodorou, E. (2015). Model Predictive Path Integral Control using Covariance Variable Importance Sampling. https://arxiv.org/abs/1509.01149"
  - "acados contributors (2026). acados: fast embedded solvers for nonlinear optimal control. https://github.com/acados/acados"
---

# Model Predictive Control (MPC)

**Model predictive control** decides what to do now by planning ahead. At every control step it uses a model to predict how the system will evolve over a short horizon, chooses the sequence of commands that best achieves the goal **while respecting limits**, applies only the first command, and then repeats from the newly measured state.

> **Etymology.** *Predictive* because a model predicts the future; *receding horizon* because the planning window slides forward with time. MPC was developed in the process industries in the late 1970s (Dynamic Matrix Control at Shell, 1979), where slow chemical plants left plenty of time to optimise. Faster computers and solvers brought it to robots in the 2010s.

## The formulation

At time $t$, from the measured state $\mathbf x_t$, solve

$$\min_{\mathbf u_0, \dots, \mathbf u_{N-1}} \sum_{k=0}^{N-1} \Big(\lVert \mathbf x_k - \mathbf x^{ref}_k\rVert^2_Q + \lVert \mathbf u_k\rVert^2_R\Big) + \lVert \mathbf x_N - \mathbf x^{ref}_N\rVert^2_P$$

subject to:
- the model, $\mathbf x_{k+1} = f(\mathbf x_k, \mathbf u_k)$, with $\mathbf x_0 = \mathbf x_t$;
- input limits (torques, speeds, accelerations), $\mathbf u_{min} \le \mathbf u_k \le \mathbf u_{max}$;
- state limits (joint ranges, obstacle avoidance, friction cones), $\mathbf x_k \in \mathcal X$.

Apply $\mathbf u_0$, measure, and solve again. With a linear model and quadratic cost this is a **quadratic program (QP)**, solvable in microseconds to milliseconds.

## Worked example: what constraints do

A mass is modelled as a double integrator (position and velocity, acceleration as input), sampled at 0.1 s. MPC (horizon 20 steps = 2 s; weights $Q = \mathrm{diag}(10, 1)$, $R = 0.1$) moves it from 0 to 1 m. Simulated in closed loop:

| Acceleration limit | Peak acceleration used | Peak speed | Settles within 2 cm | Overshoot |
|---|---|---|---|---|
| 10 m/s² (effectively unlimited) | 7.6 m/s² | 1.28 m/s | 1.3 s | 0.5% |
| 1 m/s² | 1.0 m/s² | 0.93 m/s | 1.9 s | 0.4% |
| 0.3 m/s² | 0.3 m/s² | 0.55 m/s | 4.3 s | 5.3% |

With a strict limit, MPC plans a slower motion that **never violates** the limit, accelerating at exactly the maximum and braking in time. A PID controller would instead command more than the motor can give, saturate, and overshoot (windup). Handling limits explicitly is MPC's main advantage.

## Solvers

| Problem | Solver family | Examples |
|---|---|---|
| Linear model, quadratic cost | QP (active set, interior point, ADMM) | OSQP, qpOASES, HPIPM |
| Nonlinear model | Sequential quadratic programming, real-time iteration | acados, CasADi, Crocoddyl |
| Non-smooth, nonconvex costs | Sampling-based | MPPI (see [[Nav2_Navigation]]), CEM |

Warm-starting from the previous solution, and stopping early with a good-enough answer (real-time iteration), are what make MPC fast enough for robots.

## MPC on robots

- **Legged robots and humanoids**: convex MPC on simplified centroidal dynamics plans contact forces over the next ~0.5 s; a whole-body controller tracks them at 1 kHz. This is the approach of the MIT Cheetah line (Kim et al., 2019) and many humanoids.
- **Drones**: MPC handles thrust limits and aggressive manoeuvres.
- **Mobile robots and cars**: path tracking with steering and speed limits; MPPI in Nav2.
- **Manipulators**: trajectory optimisation with collision constraints (GPU-parallel in cuRobo).

## MPC and learning (2026)

MPC and RL are complementary, and increasingly combined:
- RL policies are often *trained* with an MPC or whole-body controller as the low level, or imitate an MPC expert, which gives fast inference with MPC-quality behaviour;
- learned models (neural dynamics, world models) replace or correct the MPC model;
- learned value functions serve as MPC's terminal cost, extending its effective horizon;
- humanoids commonly run an RL locomotion policy and an MPC or whole-body QP for manipulation and contact-rich tasks.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Model_Predictive_Control_(MPC)]])
```
