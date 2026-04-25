---
title: Singularities
description: Configurations where the Jacobian loses rank and the manipulator loses one or more directions of instantaneous motion. The places where IK breaks, joint velocities explode, and force control becomes ill-posed.
tags:
  - robotics
  - kinematics
  - singularities
  - jacobian
  - manipulator
  - control
type: Robotic Concept
layout: default
category: robotics
author: Jordan_Smith
date: 2025-04-29
permalink: /singularities/
related:
  - "[[Kinematics]]"
  - "[[Forward_Kinematics]]"
  - "[[Inverse_Kinematics]]"
  - "[[Jacobian_Matrix]]"
  - "[[Manipulability]]"
  - "[[Workspace]]"
  - "[[Linear_Algebra_for_Robotics]]"
---

# Singularities

A **singularity** is a configuration $\boldsymbol{\theta}^*$ where the manipulator's [[Jacobian_Matrix|Jacobian]] $J(\boldsymbol{\theta}^*)$ loses rank — the matrix mapping joint velocities to end-effector twists becomes rank-deficient. At a singularity, certain directions of end-effector motion become *instantaneously impossible* regardless of how the joints move, and the inverse mapping (joint velocities from end-effector velocities) becomes ill-defined.

> **Etymology.** From Latin *singularis*, "single" or "individual," from *singulus*, "alone." In mathematics a *singularity* is a point where a function "goes singular" — undefined, infinite, or otherwise misbehaved. In robotics the relevant misbehavior is rank loss in the Jacobian.

---

## Why singularities matter

Three problems arise simultaneously at a singularity:

1. **Some end-effector motions become unreachable.** In the singular direction(s), no choice of $\dot{\boldsymbol{\theta}}$ produces motion.
2. **Inverse kinematics blows up.** $\dot{\boldsymbol{\theta}} = J^{-1} \mathcal{V}$ requires infinite joint velocities to produce finite Cartesian velocity in the lost direction(s).
3. **Force-control fails.** External wrenches in the singular direction are absorbed without producing reaction torques in the joints — the robot cannot resist them.

The robot is structurally weaker, controllably worse, and numerically explosive — all at the same configuration.

---

## The mathematical definition

For an $m \times n$ Jacobian:

- **Singular configuration** $\boldsymbol{\theta}^*$ ⇔ $\text{rank}(J(\boldsymbol{\theta}^*)) < \min(m, n)$.
- For square Jacobians ($m = n = 6$): equivalent to $\det J = 0$.
- For non-square Jacobians: detected by SVD — singular when the smallest singular value $\sigma_n$ approaches zero.

The **manipulability index** quantifies distance to a singularity:

$$
w(\boldsymbol{\theta}) = \sqrt{\det(J J^T)} = \sigma_1 \sigma_2 \cdots \sigma_m
$$

$w = 0$ exactly when $J$ is singular. See [[Manipulability]].

---

## Three flavors

### 1. Boundary singularity

Occurs at the workspace boundary — the arm is fully extended or fully folded. The lost direction is *along* the radial direction of reach.

**Example:** A 2R planar arm with $\det J = L_1 L_2 \sin\theta_2$ is singular when $\theta_2 = 0$ (arm straight) or $\theta_2 = \pm\pi$ (arm folded).

These are unavoidable structural features of the robot's geometry — every reachable workspace has a boundary.

### 2. Interior singularity

Occurs *inside* the workspace where joint axes happen to align. The lost direction does *not* coincide with the workspace boundary.

**Example: wrist singularity.** A 6-DoF arm with a [[DH_Parameters|spherical wrist]] (last three axes intersect) becomes singular when the second-wrist joint reaches $\theta_5 = 0$ — joints 4 and 6 align, and rotation about that axis becomes redundant. Roll commands at that instant produce no motion in the lost direction.

**Example: shoulder singularity.** Joints 1 and 4 align such that joint 1 rotation has no effect on the wrist position.

**Example: elbow singularity.** Same as the boundary case for the inner triangle of the arm.

### 3. Algorithmic singularity

A singularity introduced by *the way the inverse problem is solved*, not by the geometry itself. Common in redundancy resolution: a chosen secondary task can drive the null-space projection to be rank-deficient even though the primary task is not.

---

## Worked example — 6-R arm wrist singularity

Most industrial 6-R arms have a spherical wrist (joints 4-5-6 axes intersect at a point). Their Jacobian factors as:

$$
\det J = \det J_p \cdot \det J_o = (\text{position-block determinant}) \cdot (\sin\theta_5)
$$

Wrist singularity: $\theta_5 = 0$. The first and third wrist axes align. Cartesian motions perpendicular to the aligned axis are lost.

Practically, a UR5 driving through the "elbow flip" pose at $\theta_5 = 0$ exhibits a momentary controller stutter as TRAC-IK or KDL detect the singular Jacobian and re-seed the IK from a nearby branch.

---

## Detection

**For a square Jacobian:** monitor $\det J$. Sign changes flag a singularity crossing.

**For any Jacobian:** monitor the smallest singular value $\sigma_n$ of $J$ via SVD or its proxy:

$$
\sigma_{\text{min}}(J) = \sqrt{\lambda_{\text{min}}(J^T J)}
$$

Or use the manipulability index $w(\boldsymbol{\theta}) = \sqrt{\det(J J^T)}$. Industrial controllers typically alarm when $\sigma_{\text{min}}$ falls below a threshold (e.g., $10^{-3}$).

The **condition number** $\kappa(J) = \sigma_{\text{max}}/\sigma_{\text{min}}$ is the standard quantitative measure of how anisotropic the Jacobian is — large $\kappa$ means near-singular.

---

## Mitigation

### Damped least squares (Levenberg-Marquardt)

The dominant practical fix. Replace $J^\dagger$ with

$$
J^\dagger_\lambda = J^T (J J^T + \lambda^2 I)^{-1}
$$

The damping factor $\lambda$ trades accuracy for stability. Adaptive schemes set $\lambda$ proportional to a singularity-distance metric — $\lambda \propto 1/\sigma_n$ ramps up smoothly as the manipulator approaches a singularity. See [[Inverse_Kinematics]] and [[Optimization_for_Robotics]].

### Singularity-robust pseudoinverse (Nakamura-Hanafusa)

Modify the SVD pseudoinverse by replacing small singular values with damped versions. Equivalent to DLS in the limit but smoother near transitions.

### Null-space avoidance

For redundant arms ($n > 7$), use the null-space term to push away from singularities:

$$
\dot{\boldsymbol{\theta}} = J^\dagger \mathcal{V} + (I - J^\dagger J) \nabla w(\boldsymbol{\theta})
$$

The secondary task is to *increase* manipulability while satisfying the primary task. Standard in the Stack-of-Tasks framework.

### Trajectory planning

Plan paths in [[Configuration_Space|configuration space]] that *don't pass through* known singular regions. Off-line planners (TrajOpt, CHOMP, STOMP) can include manipulability as a soft constraint.

### Mechanical design

Some manipulators are designed to push singularities to the workspace edges. The **Pieper criterion** (three intersecting joints or three parallel joints) ensures closed-form IK while keeping interior singularities tractable. Most industrial arms (UR, KUKA, ABB) follow this pattern.

---

## Singularity-free is impossible for most manipulators

A theorem from differential topology: any smooth map from a compact $n$-dimensional joint space to a higher-dimensional or topologically nontrivial task space *must* have singularities. Concretely:

- A 6-R arm cannot avoid all interior singularities — its Jacobian determinant changes sign across them.
- Only specific compositions (e.g., orthogonal-axis 7-DoF arms with redundancy resolution) can stay reasonably far from singularity over their workspace.
- Spherical wrists *will* have wrist singularities; this is a structural feature, not a flaw.

The engineering goal is therefore not to eliminate singularities but to keep workspace useful regions *far enough* from them.

---

## Special case: parallel mechanisms

In a [[Parallel_Mechanisms_and_Robots|parallel manipulator]] (Stewart platform, Delta), singularities have a richer taxonomy:

- **Forward / direct singularity:** platform gains an uncontrolled DoF — *very dangerous* (small applied force causes large platform motion).
- **Inverse singularity:** platform loses a DoF — analogous to the serial-arm case.
- **Combined singularity:** both happen simultaneously, only on specific geometries.

See [[Parallel_Mechanisms_and_Robots]] and [[Singularities_Dynamic_Effects]].

---

## Recommended reading

- Lynch & Park, *Modern Robotics*, §5.3 — singularities in body and space Jacobians
- Spong, Hutchinson, Vidyasagar, *Robot Modeling and Control*, §4.6 — geometric classification
- Nakamura & Hanafusa (1986), *Inverse kinematic solutions with singularity robustness for robot manipulator control* — the SVD-damped pseudoinverse paper
- Sciavicco & Siciliano, *Modelling and Control of Robot Manipulators* — exhaustive worked examples on industrial arms
- Park & Brockett (1994), *Kinematic dexterity of robotic mechanisms* — manipulability theory

---

## Dataview

```dataview
LIST FROM #singularities OR #kinematics WHERE contains(file.outlinks, [[Singularities]])
```
