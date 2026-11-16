---
title: Physics Engines
description: "The numerical core of a robot simulator: equations of motion, time stepping and integrators, collision detection, and the contact models (complementarity, soft constraints, penalty, position-based) that decide how simulated objects push, slide and stick. Compares MuJoCo, PhysX, Newton, Bullet, Drake and Genesis as of 2026."
aliases:
  - Physics Engine
  - Contact Dynamics Simulation
tags:
  - simulation
  - dynamics
  - collision-detection
  - numerical-methods
  - friction
type: concept
level: advanced
subfield:
  - software
  - kinematics-dynamics
status: keystone
author: Jordan_Smith
date: 2026-11-15
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Robot_Simulation]]"
  - "[[Rigid_Body_Dynamics]]"
related:
  - "[[MuJoCo]]"
  - "[[GPU-Parallel_Simulation]]"
  - "[[Sim-to-Real_Transfer]]"
  - "[[Friction]]"
  - "[[Collision_Detection]]"
key_takeaways:
  - "A physics engine solves M(q)q̈ + c(q, q̇) = τ + Jᵀλ each step, where the hard part is finding the contact forces λ."
  - "Engines differ mainly in how they model contact: hard complementarity, soft constraints (MuJoCo), penalties, or position-based methods for deformables."
  - "Stiff contact limits the time step; a light, stiff object can make a simulation explode unless the step is small enough."
sources:
  - "Todorov, E., Erez, T. and Tassa, Y. (2012). MuJoCo: A physics engine for model-based control. https://homes.cs.washington.edu/~todorov/papers/TodorovIROS12.pdf"
  - "Google DeepMind (2026). MuJoCo documentation: Computation. https://mujoco.readthedocs.io/en/stable/computation/index.html"
  - "Google DeepMind (2026). MuJoCo 4: Studio & Filament, Deformables, Speculative Contacts. https://github.com/google-deepmind/mujoco/discussions/3773"
  - "Newton contributors (2026). Newton release v1.0.0. https://github.com/newton-physics/newton/releases/tag/v1.0.0"
  - "Collins, J. et al. (2021). A Review of Physics Simulators for Robotic Applications. https://ieeexplore.ieee.org/document/9386154"
---

# Physics Engines

A **physics engine** is the numerical core of a simulator: the code that takes the current positions, velocities and motor commands and computes where everything will be a fraction of a second later. Most of what makes simulated robots feel real (or fake) comes down to one question every engine must answer: when two things touch, what forces stop them passing through each other, and how much do they slide?

> **Etymology.** *Physics* comes from Greek *phusikē* (*epistēmē*), "knowledge of nature", from *phusis*, "nature". *Engine* is from Latin *ingenium*, "innate quality, cleverness, a clever device" (the same root as *ingenious* and *engineer*). Game developers popularised "physics engine" in the late 1990s for the reusable module that moved objects realistically; robotics adopted it as simulators split into an engine and the tools around it.

## The equation every engine solves

For a robot with generalised coordinates $q$ (joint angles plus base pose):

$$M(q)\,\ddot q + c(q, \dot q) = \tau + J_c(q)^\top \lambda$$

- $M(q)$: the mass (inertia) matrix.
- $c(q,\dot q)$: Coriolis, centrifugal and gravity terms.
- $\tau$: actuator torques.
- $J_c^\top \lambda$: contact forces $\lambda$ mapped into joint space by the contact Jacobian $J_c$.

Without contact this is ordinary [[Rigid_Body_Dynamics|rigid-body dynamics]], solved efficiently by recursive algorithms such as Featherstone's articulated-body algorithm in $O(n)$ for $n$ joints. **Contact makes it hard**: $\lambda$ is unknown, depends on the motion, and must satisfy physical rules.

## Contact: the rules and the models

Real rigid contact obeys three rules: bodies do not interpenetrate (gap $\phi \ge 0$); contact can only push, never pull ($\lambda_n \ge 0$); and force only acts when touching ($\phi\,\lambda_n = 0$). Friction adds Coulomb's cone: $\lVert \lambda_t \rVert \le \mu\,\lambda_n$. Engines model this in different ways:

| Model | Idea | Used by | Trade-off |
|---|---|---|---|
| **Complementarity (LCP/NCP)** | Solve the rules exactly as a complementarity problem | ODE, Bullet, PhysX (variants) | Physically crisp; solving can be hard, friction cones approximated |
| **Soft constraints / convex** | Allow tiny, controlled violations; solve a convex optimisation | [[MuJoCo]], MuJoCo Warp | Stable, smooth, invertible; contact is slightly "soft" |
| **Penalty (springs)** | Push bodies apart with a stiff spring proportional to penetration | Many early simulators; some differentiable engines | Simple; stiff springs need tiny steps |
| **Position-based (XPBD, VBD)** | Correct positions directly to satisfy constraints | Cloth, cables, soft bodies; Newton's VBD solver | Fast and robust for deformables; less exact for rigid robots |
| **Particle / continuum (MPM)** | Represent material as particles on a background grid | Sand, fluids, dough; Newton's MPM solver, Genesis | Handles large deformation; expensive |

## Time stepping and integrators

The engine advances in steps of $\Delta t$ (often 1–5 ms for robots). Choices:

- **Semi-implicit (symplectic) Euler**: update velocity first, then position with the new velocity. Cheap and stable for most robot simulation.
- **Implicit integrators**: treat stiff forces (damping, soft-body elasticity) implicitly so larger steps stay stable; MuJoCo 4 added implicit integration for soft solids, shells and rods.
- **Substeps**: the policy may act at 50 Hz while physics steps at 200–1000 Hz underneath.

## Worked example: why a light, stiff object explodes

Model contact as a spring of stiffness $k$ pressing on a mass $m$. Its natural frequency is $\omega = \sqrt{k/m}$, and semi-implicit Euler is stable only if $\omega\,\Delta t < 2$. For a 50-gram cube on a contact of stiffness $k = 10^5$ N/m:

$$\omega = \sqrt{10^5 / 0.05} = \sqrt{2 \times 10^6} \approx 1414 \text{ rad/s} \quad\Rightarrow\quad \Delta t < 2/1414 \approx 1.4 \text{ ms}$$

At the common default of 2 ms the cube's contact is unstable: it jitters, gains energy and may fly off. Halving its mass shrinks the safe step by a factor of $\sqrt{2}$. The fixes are a smaller step (0.5 ms), softer contact parameters, or an implicit treatment. This is exactly the failure that makes small objects "explode" in robot-manipulation simulations.

## Fast and thin objects

A fast object can pass through a thin wall between two steps without ever overlapping it ("tunnelling"). Remedies are smaller steps, thicker collision margins, or **continuous / speculative collision detection**, which anticipates contacts within the step. MuJoCo 4.0 (November 2026) added speculative contacts for exactly this.

## Engines in 2026

| Engine | Contact approach | Runs on | Notes |
|---|---|---|---|
| **MuJoCo 4** | Soft convex constraints | CPU, multi-world CPU batches, GPU/TPU via MJX, NVIDIA GPUs via MuJoCo Warp, browser | Apache-2.0; deformables, plasticity, speculative contacts, system-identification toolbox |
| **PhysX 5** | Complementarity-style rigid solver | CPU / GPU | Engine of Isaac Sim; available standalone as OVPhysX in Isaac Lab 3.0 |
| **Newton** | Several solvers: MuJoCo Warp, Featherstone, XPBD, VBD, MPM | NVIDIA GPUs (Warp) | Apache-2.0, Linux Foundation project started by Disney Research, Google DeepMind and NVIDIA; v1.0 released April 2026 |
| **Bullet / PyBullet** | Complementarity | CPU | Widely used in older research; PyBullet is no longer actively developed |
| **Drake** | Hydroelastic and point contact | CPU | Toyota Research Institute; built for planning, control and verification |
| **Genesis** | Multiple solvers (rigid, FEM, MPM, PBD, SPH) | GPU | Apache-2.0; aims at multi-physics robot learning |
| **DART, ODE** | Complementarity | CPU | Physics back ends used by Gazebo |

## What to check when choosing

1. Contact quality for *your* task: test the hardest contact (a light object, a high-gear joint, a foot on a slope).
2. Speed: steps per second, and whether it batches on a GPU.
3. Deformables if you need them (cloth, cables, food).
4. Model formats: URDF, MJCF, USD.
5. Determinism and differentiability, if you need gradients or exact replay.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Physics_Engines]])
```
