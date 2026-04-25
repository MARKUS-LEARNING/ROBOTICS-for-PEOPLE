---
title: Inverse Kinematics (IK)
description: Given a desired end-effector pose, find joint angles that achieve it. The hard half of kinematics — nonlinear, often non-unique, sometimes unsolvable. The bridge between task-space goals and joint-space commands.
tags:
  - kinematics
  - inverse-kinematics
  - manipulator
  - control
  - jacobian
  - optimization
layout: default
category: robotics
author: Jordan_Smith
date: 2025-04-28
permalink: /inverse_kinematics/
related:
  - "[[Kinematics]]"
  - "[[Forward_Kinematics]]"
  - "[[Jacobian_Matrix]]"
  - "[[Singularities]]"
  - "[[Workspace]]"
  - "[[Manipulability]]"
  - "[[Configuration_Space]]"
  - "[[Analytical_IK_Solutions]]"
  - "[[Numerical_IK_Solutions]]"
  - "[[Optimization_for_Robotics]]"
---

# Inverse Kinematics (IK)

**Inverse Kinematics (IK)** is the problem of finding joint angles $\boldsymbol{\theta}$ that place the end-effector at a desired pose $T_{\text{desired}} \in SE(3)$. Mathematically:

$$
\text{find } \boldsymbol{\theta} \quad \text{such that} \quad f(\boldsymbol{\theta}) = T_{\text{desired}}
$$

where $f$ is the [[Forward_Kinematics|forward-kinematics]] map. Conceptually it asks: *given where I want the gripper, what should the joints do?*

> **Etymology.** *Inverse* — Latin *inversus*, "turned upside down," past participle of *invertere*, "to turn over." IK is FK turned around: same equation, opposite direction of solving. *Kinematics* from Greek *kinēma*, "movement."

---

## Why IK is hard and FK is easy

[[Forward_Kinematics|FK]] is a *function* — one input, one output, computable by chaining matrix multiplies. IK is the *inverse problem*, and inverses of nonlinear functions inherit four difficulties:

1. **Existence.** A target outside the [[Workspace|workspace]] has *no* solutions.
2. **Uniqueness.** A typical 6-R arm has up to 16 distinct joint vectors that produce the same end-effector pose ("elbow up/down," "wrist flip," etc.).
3. **Closed form is rare.** Only specific kinematic structures (spherical wrist, three intersecting axes) admit analytical IK. Most real robots use numerical iteration.
4. **Singularities.** At [[Singularities|singular configurations]], multiple inputs map to the same output and the inverse becomes ill-defined.

Symbolically, FK is "evaluate a polynomial." IK is "find all roots of a coupled nonlinear system."

---

## Two main families of solvers

### Analytical (closed-form) IK

Direct algebraic or trigonometric solutions that compute *all* valid $\boldsymbol{\theta}$ in finite steps.

- **Pros:** sub-microsecond evaluation, all branches enumerated, deterministic.
- **Cons:** only exists for kinematically "nice" robots — typically those satisfying the **Pieper criterion** (three consecutive joints intersect at a point or three consecutive axes are parallel).
- **Examples:** UR5/UR10 (closed-form IK exists), KUKA KR series, classical 6-DoF industrial arms designed *specifically* to admit closed-form IK.
- **Tools:** `IKFast` (OpenRAVE) generates C++ closed-form IK at compile time from a URDF.

See [[Analytical_IK_Solutions]].

#### Geometric decomposition (spherical wrist)

When the last three axes intersect at a single "wrist center" point $\mathbf{p}_w$, the problem splits cleanly:

1. **Inverse position:** find $\theta_1, \theta_2, \theta_3$ such that the wrist center lands at $\mathbf{p}_w$. Reduces to planar trigonometry (law of cosines on the arm triangle).
2. **Inverse orientation:** with the first three angles fixed, solve $\theta_4, \theta_5, \theta_6$ such that the wrist matches the desired orientation. This is an Euler-angle extraction.

The total of $2 \times 2 \times 2 = 8$ branches comes from sign choices at each step (shoulder, elbow, wrist).

### Numerical (iterative) IK

Converge to *a* solution by minimizing pose error from an initial guess $\boldsymbol{\theta}_0$.

- **Pros:** works for any kinematic structure, including 7-DoF redundant arms.
- **Cons:** finds *one* solution (the one nearest the seed), local-minima risk, slower than closed-form.

#### Jacobian-pseudoinverse (resolved-motion-rate)

At each iteration, compute pose error and step:

$$
\Delta \mathbf{x} = \log(T_{\text{desired}} \cdot T_{\text{current}}^{-1}) \in \mathbb{R}^6
$$

$$
\Delta \boldsymbol{\theta} = J^\dagger \Delta \mathbf{x}
$$

$$
\boldsymbol{\theta} \leftarrow \boldsymbol{\theta} + \alpha \Delta \boldsymbol{\theta}
$$

where $J^\dagger$ is the Moore-Penrose pseudoinverse of the [[Jacobian_Matrix|Jacobian]] and $\log : SE(3) \to \mathbb{R}^6$ is the matrix logarithm (the twist-error). For square invertible $J$, $J^\dagger = J^{-1}$. For redundant or singular $J$, the pseudoinverse is the right generalization.

This is **Newton's method** on the pose-error map, with step size $\alpha$.

#### Damped least squares (Levenberg-Marquardt)

Pure pseudoinverse blows up near singularities — the smallest singular value of $J$ approaches zero, $J^\dagger$ approaches infinity. The fix is **damping**:

$$
\Delta \boldsymbol{\theta} = (J^T J + \lambda^2 I)^{-1} J^T \Delta \mathbf{x}
$$

$\lambda$ trades off accuracy against stability. Small $\lambda$ ⇒ accurate but unstable near singularities; large $\lambda$ ⇒ stable but slow. Adaptive $\lambda$ schemes (e.g., scale by smallest singular value) are standard. This is the **Levenberg-Marquardt algorithm** applied to IK and the dominant choice in modern solvers (KDL, TRAC-IK, Pinocchio's `inverse-kinematics` examples).

See [[Numerical_IK_Solutions]] and [[Optimization_for_Robotics]].

#### Optimization-based IK

Cast IK as an [[Optimization_for_Robotics|optimization]]:

$$
\min_{\boldsymbol{\theta}} \; \lVert \log(T_{\text{desired}} \cdot f(\boldsymbol{\theta})^{-1}) \rVert^2 + \lambda \lVert \boldsymbol{\theta} - \boldsymbol{\theta}_{\text{rest}} \rVert^2
$$

subject to joint limits, collision avoidance, manipulability constraints, etc. Solved with SQP (SNOPT, IPOPT) or trust-region methods. Used when IK is part of a bigger problem (e.g., trajectory optimization, whole-body humanoid balance).

---

## Redundancy

A manipulator with $n > 6$ joints is **kinematically redundant** for 6-DoF tasks. Examples: 7-DoF arms (Franka Panda, KUKA iiwa, Baxter), human arms.

Redundancy means an *infinite* set of joint configurations produces the same end-effector pose — the IK has a one-or-more-dimensional **null space**. The null space can be exploited to:

- Avoid joint limits
- Avoid singularities (maximize manipulability)
- Avoid obstacles
- Stay close to a comfortable "rest" configuration

The redundancy resolution rule:

$$
\Delta \boldsymbol{\theta} = J^\dagger \Delta \mathbf{x} + (I - J^\dagger J) \boldsymbol{\theta}_0
$$

The first term satisfies the task; $(I - J^\dagger J)$ projects $\boldsymbol{\theta}_0$ into the null space, so the second term moves the joints in directions that *don't* affect the end-effector.

This is the **secondary-task** framework — the basis of operational-space control and stacked-tasks frameworks (Stack-of-Tasks, OSC, Mahalanobis-weighted IK).

---

## Singularities and IK

Near a [[Singularities|singularity]] the Jacobian loses rank. Three symptoms in IK:

1. **Infinite joint velocities** — small Cartesian motions in the singular direction require huge joint motions.
2. **Solution discontinuity** — the IK answer can flip between branches as you cross a singularity.
3. **Pseudoinverse explosion** — without damping, $J^\dagger$ blows up.

Damped least squares is the standard remedy. Better: avoid the singularity by adding manipulability to the cost (see [[Manipulability]]) or by exploiting redundancy.

---

## When IK fails

Common failure modes and remedies:

| Failure | Cause | Fix |
|---|---|---|
| No convergence | Poor initial guess; far from valid configuration | Multi-start with random seeds; warm-start from previous solve |
| Joint-limit violation | Solver ignores limits | Add limit constraints (clamp + project, or NLP with bounds) |
| Self-collision | Solver ignores geometry | Add a separating-axis penalty or solve via MoveIt's `IKWithCollision` |
| Drifts to singular pose | No manipulability term | Add $-\det(JJ^T)$ penalty or null-space term |
| Wrong branch (e.g., elbow flips) | Newton converges to nearest local solution | Seed the solver with the previous timestep's $\boldsymbol{\theta}$ |

---

## IK in practice — the production stacks

| Tool | What it does | Notes |
|---|---|---|
| **KDL** | Newton + damped LS | Classical ROS solver, brittle near singularities |
| **TRAC-IK** | KDL + SQP fallback | Better convergence rate; standard MoveIt default |
| **IKFast** | Closed-form codegen | Microsecond solves; best for known geometries |
| **Pinocchio** | Newton + damped LS in `pinocchio.computeForwardKinematics` + manual loop | Programmable; integrates with autodiff |
| **Drake `InverseKinematics`** | NLP via SNOPT/IPOPT | Constraint-rich, slower per solve |
| **MoveIt 2** | Pluggable IK + collision-aware planning | Production end-to-end |
| **Bio-IK** | Genetic + iterative hybrid | Robust on humanoids, slower |

**Rule of thumb:** if IKFast can build closed-form code for your robot, use it. Otherwise, TRAC-IK is the practical default. For optimization-rich problems, Drake or Pinocchio with manual SQP.

---

## Mobile robots

For [[Locomotion|wheeled platforms]], "inverse kinematics" usually means computing wheel velocities from a desired chassis velocity. The relations from [[Forward_Kinematics|FK]] invert directly:

### Differential drive
$$
\dot\phi_R = \frac{2v + b\omega}{2r}, \quad \dot\phi_L = \frac{2v - b\omega}{2r}
$$

### Ackermann (bicycle)
$$
\delta = \arctan\!\left(\frac{L \omega}{v}\right), \quad \dot\phi = v / r
$$

These are closed-form and trivial. The hard part for mobile robots is *path planning* under [[Nonholonomic_Constraint|nonholonomic constraints]], not IK itself.

---

## Recommended reading

- Lynch & Park, *Modern Robotics*, Ch. 6 — modern PoE-style IK derivation and numerical methods
- Spong, Hutchinson, Vidyasagar, *Robot Modeling and Control*, Ch. 4 — geometric and algebraic IK
- Buss, *Introduction to Inverse Kinematics with Jacobian Transpose, Pseudoinverse and Damped Least Squares Methods* (2009 tutorial) — best free reference for numerical IK
- Sugihara, *Solvability-Unconcerned Inverse Kinematics by the Levenberg-Marquardt Method* (2011) — TRAC-IK foundation

---

## Dataview

```dataview
LIST FROM #kinematics OR #inverse-kinematics WHERE contains(file.outlinks, [[Inverse_Kinematics]])
```
