---
title: Manipulability
description: A geometric measure of how well a manipulator can move and apply force in different directions at a given configuration. Encoded by the singular values of the Jacobian and visualized as the manipulability ellipsoid.
tags:
  - kinematics
  - manipulability
  - jacobian
  - dexterity
  - manipulator
  - singularities
layout: default
category: robotics
author: Jordan_Smith
date: 2025-05-02
permalink: /manipulability/
related:
  - "[[Kinematics]]"
  - "[[Jacobian_Matrix]]"
  - "[[Singularities]]"
  - "[[Workspace]]"
  - "[[Linear_Algebra_for_Robotics]]"
  - "[[Inverse_Kinematics]]"
---

# Manipulability

**Manipulability** quantifies how isotropically a manipulator can move and apply force in different directions at a given configuration. Introduced by Tsuneo Yoshikawa in 1985, it is a configuration-dependent scalar (or geometric ellipsoid) extracted from the singular values of the [[Jacobian_Matrix|Jacobian]].

> **Etymology.** *Manipulate* — Latin *manipulus*, "a handful," from *manus* (hand) + *plere* (to fill). Originally a *manipulus* was a fistful of grain. By the 18th century *manipulate* had drifted to mean "to handle skillfully." *Manipulability* is the abstract noun: the *capacity to manipulate*. In robotics it is shorthand for "how dextrous is the arm in this pose?"

---

## Why we need this number

The Jacobian alone tells you whether the robot is at a singularity (rank-deficient) or not (full rank). But "not singular" is binary — and the practical question is *how close* are we to a singularity, and *which directions* are we losing dexterity in.

Manipulability measures answer both:

1. **Manipulability index** $w$ — a scalar. How much "volume" of motion is available?
2. **Manipulability ellipsoid** — a 6-D ellipsoid in twist space. *Where* (in which directions) the manipulator is dextrous vs. weak.

---

## The manipulability ellipsoid

Take a unit-norm joint-velocity vector $\dot{\boldsymbol{\theta}}$ with $\lVert \dot{\boldsymbol{\theta}} \rVert = 1$. The set of resulting end-effector twists $\mathcal{V} = J \dot{\boldsymbol{\theta}}$ traces out an ellipsoid in $\mathbb{R}^6$ — the **manipulability ellipsoid**:

$$
\mathcal{V}^T (J J^T)^{-1} \mathcal{V} \leq 1
$$

The principal axes are the *left singular vectors* of $J$; the axis lengths are the singular values $\sigma_1 \geq \sigma_2 \geq \cdots \geq \sigma_m$.

- **Long axis (largest $\sigma$)** — direction in which the arm can move *fast* with bounded joint velocity.
- **Short axis (smallest $\sigma$)** — direction in which the arm can barely move; near zero, the manipulator is approaching a singularity.
- **Aspect ratio (condition number)** — $\kappa = \sigma_1 / \sigma_m$. Aspect ratio $1$ means perfectly isotropic motion; $\infty$ means rank-deficient.

The ellipsoid is *configuration-dependent*: it changes shape and size as $\boldsymbol{\theta}$ changes. Useful visualization for path planning — drawing the ellipsoid along a candidate trajectory tells you exactly where the arm gets weak.

---

## The Yoshikawa manipulability index

Yoshikawa's scalar measure is the volume of the manipulability ellipsoid (up to a constant):

$$
w(\boldsymbol{\theta}) = \sqrt{\det(J(\boldsymbol{\theta}) \, J^T(\boldsymbol{\theta}))} = \sigma_1 \sigma_2 \cdots \sigma_m
$$

Properties:

- $w \geq 0$, with $w = 0$ exactly at a [[Singularities|singularity]].
- For a square Jacobian: $w = |\det J|$.
- Smooth in $\boldsymbol{\theta}$ — differentiable, suitable as a cost function in optimization.
- Configuration-dependent: maximized at a "comfortable" interior pose, falls off near workspace boundaries and at interior singularities.

This is by far the most common single-number manipulability measure, but it has a drawback: $w$ is the product of all singular values, so a *single* small singular value drags the index down regardless of how dextrous the rest of the directions are. For tasks that only care about a subset of directions, restrict $J$ to those rows before computing $w$.

---

## Velocity vs. force manipulability

The Jacobian's duality (velocities forward, forces backward via $J^T$) creates two complementary ellipsoids:

| | **Velocity ellipsoid** | **Force ellipsoid** |
|---|---|---|
| Definition | $\mathcal{V} = J \dot{\boldsymbol{\theta}}$, unit $\dot{\boldsymbol{\theta}}$ | $\boldsymbol{\tau} = J^T \mathcal{F}$, unit $\boldsymbol{\tau}$ |
| Shape | $J J^T$ | $(J J^T)^{-1}$ |
| Long where | Easy to move fast | Easy to apply force |
| Short where | Hard to move fast | Hard to apply force |

The two ellipsoids are *reciprocals*: long axes of velocity coincide with short axes of force, and vice versa. *A manipulator that moves fast in a direction is mechanically weak in that direction.* This is why precision-machining robots have *short* moment arms — you trade speed for force.

The force manipulability index:

$$
w_f(\boldsymbol{\theta}) = 1/w(\boldsymbol{\theta}) = \frac{1}{\sigma_1 \sigma_2 \cdots \sigma_m}
$$

(when $w > 0$). Diverges at singularities — meaning at a singular configuration, an *unbounded* end-effector wrench can be balanced by zero joint torque (the manipulator gives up that direction structurally).

---

## What manipulability is used for

### 1. Singularity-distance diagnostics

The smallest singular value $\sigma_m$ is the most informative single number — it directly bounds how close the configuration is to losing rank. Most damped-least-squares IK schedules use $\sigma_m$ to set the damping factor.

### 2. Redundancy resolution

For a 7-DoF redundant arm, the null-space term in inverse kinematics can be steered toward configurations of higher manipulability:

$$
\dot{\boldsymbol{\theta}} = J^\dagger \mathcal{V} + (I - J^\dagger J) \nabla w(\boldsymbol{\theta})
$$

The robot satisfies the primary task while simultaneously moving toward a more dextrous pose. Standard in operational-space control. See [[Inverse_Kinematics]].

### 3. Trajectory optimization

Manipulability appears as a soft cost in trajectory optimization — penalize $-w$ to encourage paths that stay far from singularities. CHOMP, TrajOpt, and Drake's `IKWithCollision` all support this.

### 4. Robot design

Picking link lengths and joint axes to *maximize the volume of the manipulability ellipsoid over the workspace* is a classical design problem. See Park and Brockett's *Kinematic dexterity of robotic mechanisms* (1994).

### 5. Workspace evaluation

Plotting $w$ over the reachable workspace gives a "dexterity map" that distinguishes the high-manipulability *dexterous* workspace from the low-manipulability fringes. See [[Workspace]] and [[Workspace_Analysis]].

---

## Variants

| Measure | Formula | Property |
|---|---|---|
| **Yoshikawa index** | $w = \sqrt{\det(J J^T)}$ | Volume of the velocity ellipsoid |
| **Smallest singular value** | $\sigma_{\min}(J)$ | Tightest singularity-distance bound |
| **Condition number** | $\kappa = \sigma_{\max}/\sigma_{\min}$ | Anisotropy ratio (1 = isotropic, $\infty$ = singular) |
| **Inverse condition number** | $1/\kappa$ | Bounded in $[0, 1]$ — convenient for normalized cost |
| **Task-restricted manipulability** | Above measures on row-subset of $J$ | Useful when only some directions matter |

For mixed translational/rotational tasks, a unit consistency issue arises: the rotation rows of $J$ have units rad/s, the translation rows have m/s, so the determinant mixes units. Practical fix: weight rows to make them dimensionally consistent, or compute separate manipulability indices for the position and orientation blocks.

---

## Worked example — 2R planar arm

Recall the 2R Jacobian: $\det J = L_1 L_2 \sin\theta_2$.

Manipulability: $w = |L_1 L_2 \sin\theta_2|$. Maximized at $\theta_2 = \pm\pi/2$ — a right-angle elbow. Zero at $\theta_2 = 0$ (extended) and $\theta_2 = \pm\pi$ (folded). The "best" pose is independent of $\theta_1$ (radial symmetry), which matches the geometric intuition.

This explains why every textbook draws the canonical 2R example with the elbow at 90°: it is precisely the maximum-manipulability configuration.

---

## Recommended reading

- Yoshikawa, T. (1985), *Manipulability of robotic mechanisms* — the original
- Park, F. C., & Brockett, R. W. (1994), *Kinematic dexterity of robotic mechanisms* — geometric interpretation
- Lynch & Park, *Modern Robotics*, §5.4 — manipulability in the PoE formulation
- Sciavicco & Siciliano, *Modelling and Control of Robot Manipulators*, Ch. 3 — extensive worked examples on industrial arms
- Murray, Li, Sastry, *A Mathematical Introduction to Robotic Manipulation*, Ch. 3 — connection to screw-system rank

---

## Dataview

```dataview
LIST FROM #manipulability OR #kinematics WHERE contains(file.outlinks, [[Manipulability]])
```
