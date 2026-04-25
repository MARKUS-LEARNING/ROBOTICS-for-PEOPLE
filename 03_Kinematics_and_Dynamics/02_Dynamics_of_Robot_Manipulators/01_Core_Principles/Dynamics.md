---
title: Dynamics
description: The study of motion and the forces that cause it. Dynamics adds Newton's second law to a kinematic skeleton — turning a geometry problem into a physics problem with mass, inertia, gravity, and friction.
tags:
  - robotics
  - dynamics
  - mechanics
  - physics
  - manipulator-dynamics
  - lagrangian
  - newton-euler
layout: default
category: robotics
author: Jordan_Smith
date: 2025-05-02
permalink: /dynamics/
related:
  - "[[Kinematics]]"
  - "[[Newton-Euler_Equations]]"
  - "[[Lagrangian_Dynamics]]"
  - "[[Manipulator_Dynamics]]"
  - "[[Inertia]]"
  - "[[Rigid_Body_Dynamics]]"
  - "[[Torque]]"
  - "[[Friction]]"
  - "[[Stability]]"
---

# Dynamics

**Dynamics** studies the relationship between motion and the forces and torques that produce it. Where [[Kinematics|kinematics]] asks "where is the robot and how is it moving?", dynamics asks "what forces caused that motion?" — adding mass, inertia, gravity, and Coriolis effects to the picture.

> **Etymology.** From the Greek *dynamis* (δύναμις), "power" or "force," from *dynasthai* (δύνασθαι), "to be able." The same root gives us *dynamic*, *dynamo*, and *dynamite*. Coined as a scientific term by Leibniz in *Specimen dynamicum* (1695) for the new science of force-driven motion that grew out of Newton's *Principia* (1687).

---

## The two halves of mechanics

| | **Kinematics** | **Dynamics** |
|---|---|---|
| Scope | Geometry of motion | Causes of motion |
| Variables | Position, velocity, acceleration | Force, torque, mass, inertia |
| Equation | $T = f(\boldsymbol{\theta})$ | $\boldsymbol{\tau} = M(\boldsymbol{\theta}) \ddot{\boldsymbol{\theta}} + C(\boldsymbol{\theta}, \dot{\boldsymbol{\theta}}) \dot{\boldsymbol{\theta}} + \mathbf{g}(\boldsymbol{\theta})$ |
| Required for | Motion planning, IK | Torque-level control, simulation, MPC |

Pure kinematics is enough for a slow positioning task. Dynamics is required the moment payloads vary, motion is fast, gravity matters, or contact forces enter the picture.

---

## Newton's three laws — the foundation

1. **Inertia:** A body at rest stays at rest, a body in motion stays in motion, unless acted on by a net force.
2. **F = ma:** The acceleration of a body is proportional to the net force on it, inversely proportional to its mass: $\mathbf{F} = m \mathbf{a}$.
3. **Action-reaction:** Every force has an equal-and-opposite reaction.

For rotation, Euler extended this to the angular form: $\boldsymbol{\tau} = I \boldsymbol{\alpha}$, where $\boldsymbol{\tau}$ is torque, $I$ the moment of inertia, $\boldsymbol{\alpha}$ angular acceleration. See [[Inertia]] and [[Rotational_Dynamics]].

---

## Inverse vs. forward dynamics

Robotics distinguishes two directions of the dynamics problem:

| Problem | Given | Find | Used for |
|---|---|---|---|
| **Inverse dynamics** | $\boldsymbol{\theta}, \dot{\boldsymbol{\theta}}, \ddot{\boldsymbol{\theta}}$ | $\boldsymbol{\tau}$ | Torque-level feedforward control, computed-torque, RNEA |
| **Forward dynamics** | $\boldsymbol{\theta}, \dot{\boldsymbol{\theta}}, \boldsymbol{\tau}$ | $\ddot{\boldsymbol{\theta}}$ | Simulation, ABA |

Inverse dynamics is *cheap and exact* — Recursive Newton-Euler Algorithm (RNEA) computes it in $O(n)$ time. Forward dynamics needs to invert the mass matrix or use the Articulated-Body Algorithm (ABA), also $O(n)$.

Both are at the heart of every modern simulator (MuJoCo, Drake, Pinocchio, Bullet, Isaac Sim).

---

## The manipulator equation

For an $n$-link serial manipulator, the equation of motion takes a canonical form:

$$
M(\boldsymbol{\theta}) \ddot{\boldsymbol{\theta}} + C(\boldsymbol{\theta}, \dot{\boldsymbol{\theta}}) \dot{\boldsymbol{\theta}} + \mathbf{g}(\boldsymbol{\theta}) + \boldsymbol{\tau}_{\text{friction}} = \boldsymbol{\tau} + J^T \mathbf{F}_{\text{ext}}
$$

Each piece has a physical meaning:

- $M(\boldsymbol{\theta})$ — **mass / inertia matrix** ($n \times n$, symmetric, positive-definite). The configuration-dependent rotational and translational inertia of all the links combined. See [[Inertia]].
- $C(\boldsymbol{\theta}, \dot{\boldsymbol{\theta}}) \dot{\boldsymbol{\theta}}$ — **Coriolis and centrifugal terms.** Velocity-coupled terms that arise from the rotating-frame nature of multi-DoF motion.
- $\mathbf{g}(\boldsymbol{\theta})$ — **gravity vector.** Joint torques required just to hold the robot up against gravity.
- $\boldsymbol{\tau}_{\text{friction}}$ — joint friction. See [[Friction]].
- $\boldsymbol{\tau}$ — actuator torques (the control input).
- $J^T \mathbf{F}_{\text{ext}}$ — joint torques produced by external wrenches at the end-effector (via the [[Jacobian_Matrix|Jacobian]] transpose).

This equation appears verbatim in *Lynch & Park*, *Spong*, *Siciliano*, and *Murray-Li-Sastry*. Memorize its shape.

---

## Two routes to derive it

### Newton-Euler (force-balance)

Walk the robot link by link, applying $\mathbf{F} = m \mathbf{a}$ and $\boldsymbol{\tau} = I \boldsymbol{\alpha}$ to each rigid body, propagating velocities outward and forces inward. The Recursive Newton-Euler Algorithm (RNEA) is the systematic version — $O(n)$ inverse dynamics. See [[Newton-Euler_Equations]].

### Lagrangian (energy-based)

Form the **Lagrangian** $L = T - V$ (kinetic minus potential energy) and apply the Euler-Lagrange equation:

$$
\frac{d}{dt}\!\left(\frac{\partial L}{\partial \dot{q}_i}\right) - \frac{\partial L}{\partial q_i} = \tau_i
$$

The math is heavier per step but coordinate-invariant — the same procedure works in any choice of generalized coordinates. See [[Lagrangian_Dynamics]].

The two routes give the **same equation of motion**. Newton-Euler is the workhorse for fast computation; Lagrangian is the cleaner pedagogy and the right tool for non-trivial constraints.

---

## Spatial vector / screw-theoretic formulation

Modern dynamics libraries (Featherstone's *Rigid Body Dynamics Algorithms*, Pinocchio, Drake) use a **6D spatial-vector** formulation that bundles linear and angular quantities into single screw-axis vectors. The result:

- $O(n)$ inverse dynamics (RNEA)
- $O(n)$ forward dynamics (ABA)
- $O(n^2)$ mass matrix
- Clean Lie-group treatment of rotations

If you've seen $M(\boldsymbol{\theta})$, $C$, $\mathbf{g}$ written with screws and adjoints, that's spatial-vector form. See [[Manipulator_Dynamics]] and [[Rigid_Body_Dynamics]].

---

## What dynamics buys you

### Computed-torque control

Instead of treating each joint as a black-box PID, use the model:

$$
\boldsymbol{\tau} = M(\boldsymbol{\theta}) \ddot{\boldsymbol{\theta}}_{\text{ref}} + C(\boldsymbol{\theta}, \dot{\boldsymbol{\theta}}) \dot{\boldsymbol{\theta}}_{\text{ref}} + \mathbf{g}(\boldsymbol{\theta})
$$

This *exactly* cancels the nonlinear coupling, leaving a linearized system the outer loop can stabilize with PID. The basis of every modern arm controller from KUKA Sunrise to Franka Panda.

### Simulation

Forward dynamics integrated with an RK4/RK45 ODE solver produces physically faithful motion. Modern simulators (MuJoCo, Drake, Isaac Sim, PyBullet, Gazebo) all rest on this.

### Trajectory optimization & MPC

[[Optimization_for_Robotics|Optimization]]-based controllers like iLQR, DDP, and nonlinear MPC need fast, differentiable forward and inverse dynamics. Pinocchio and Drake provide analytical derivatives for exactly this reason.

### Reinforcement learning

The "physics" inside Gym, Isaac Lab, and Brax simulators is forward dynamics integrated forward in time. A policy that wins in simulation depends on dynamics being faithful to the real robot — *the* crux of sim-to-real transfer.

### Force / impedance / admittance control

Whenever a robot must regulate contact (assembly, polishing, surgery, humanoid balance), pure kinematic control is not enough. The end-effector wrench couples to joint torques through $\boldsymbol{\tau} = J^T \mathbf{F}$, and a dynamic model is essential.

---

## Mobile and floating-base dynamics

Beyond serial arms:

- **Mobile robots** carry rolling constraints (nonholonomic) — see [[Locomotion]] and [[Robot_Gaits]].
- **Quadrupeds and humanoids** are *floating-base* — the base is unactuated and the dynamics couple base motion to limb torques. The mass matrix becomes $(n+6) \times (n+6)$.
- **Underactuated** systems (cart-pole, acrobot, quadrotor) have fewer actuators than DoF, making control much harder. Tedrake's *Underactuated Robotics* is the standard reference.

---

## Practical tooling

| Library | Language | Strength |
|---|---|---|
| **Pinocchio** | C++/Python | Fast analytical RNEA, ABA, and derivatives |
| **Drake** | C++/Python | Multibody dynamics + trajectory optimization in one stack |
| **MuJoCo** | C/Python | Contact-rich simulation, GPU-friendly |
| **Bullet / PyBullet** | C++/Python | General-purpose simulation, used in OpenAI Gym |
| **Featherstone's RBDL** | C++ | Reference implementation of *Rigid Body Dynamics Algorithms* |
| **Gazebo / Ignition** | C++ | ROS-integrated simulator with ODE/Bullet/DART backends |
| **Isaac Sim / Lab** | Python | NVIDIA's GPU-parallel sim for RL |

---

## Recommended reading

- Roy Featherstone, *Rigid Body Dynamics Algorithms* — the canonical algorithmic reference
- Lynch & Park, *Modern Robotics*, Ch. 8 — modern PoE-style derivation
- Spong, Hutchinson, Vidyasagar, *Robot Modeling and Control*, Ch. 7 — Lagrangian derivation worked end-to-end
- Murray, Li, Sastry, *A Mathematical Introduction to Robotic Manipulation*, Ch. 4 — screw-theoretic treatment
- Tedrake, *Underactuated Robotics* (free MIT course) — dynamics-aware control for non-fully-actuated systems

---

## Dataview

```dataview
LIST FROM #dynamics OR #manipulator-dynamics WHERE contains(file.outlinks, [[Dynamics]])
```
