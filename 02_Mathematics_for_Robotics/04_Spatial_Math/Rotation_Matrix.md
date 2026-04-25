---
title: Rotation Matrix
description: A 3x3 orthogonal matrix with determinant +1 that represents a rotation in 3D space. The canonical first-principles representation of orientation in robotics.
tags:
  - mathematics
  - spatial-math
  - rotation
  - SO3
  - linear-algebra
  - kinematics
  - robotics
layout: default
category: mathematics
author: Jordan_Smith
date: 2025-05-02
permalink: /rotation_matrix/
related:
  - "[[Linear_Algebra_for_Robotics]]"
  - "[[Euler_Angles]]"
  - "[[Quaternions]]"
  - "[[Axis_Angle]]"
  - "[[Homogeneous_Transformation]]"
  - "[[Lie_Groups]]"
---

# Rotation Matrix

A **rotation matrix** is a $3 \times 3$ real matrix $R$ satisfying two conditions:

$$
R^T R = I, \quad \det(R) = +1
$$

The first condition makes $R$ **orthogonal** — it preserves lengths and angles. The second picks out *proper* rotations (the $+1$ determinant excludes reflections, which have $\det = -1$). The set of all such matrices forms the **special orthogonal group** $SO(3)$.

> **Etymology.** *Rotation* comes from Latin *rotare*, "to turn a wheel." *Matrix* — Latin for "womb" or "source," later a rectangular array. *Orthogonal* comes from Greek *orthos* ("right, straight") + *gonia* ("angle") — a right-angle-preserving map.

---

## Why 3×3 orthogonal with determinant +1?

This characterization is not arbitrary — it is forced by what a rotation has to do.

1. A rotation must preserve **distances**: $\lVert R\mathbf{v} \rVert = \lVert \mathbf{v} \rVert$ for every $\mathbf{v}$.
2. A rotation must preserve **angles**: the dot product $(R\mathbf{u}) \cdot (R\mathbf{v}) = \mathbf{u} \cdot \mathbf{v}$.
3. A rotation must not **flip handedness**: right-handed frames stay right-handed.

Conditions (1) and (2) together force $R^T R = I$ (orthogonality). Condition (3) forces $\det R = +1$ (rules out reflections).

---

## Using a rotation matrix

Let $R^a_b$ be the rotation matrix expressing frame $b$'s orientation in frame $a$'s coordinates. It acts in two dual ways:

### (a) Re-expressing a vector in a new frame

If $\mathbf{v}^b$ is a vector's coordinates in frame $b$, its coordinates in frame $a$ are:

$$
\mathbf{v}^a = R^a_b \, \mathbf{v}^b
$$

### (b) Rotating a vector within a single frame

The same matrix, applied to a vector in one frame, rotates that vector:

$$
\mathbf{v}' = R \, \mathbf{v}
$$

Both interpretations coexist. Sanity-check with a drawing whenever the code is new.

---

## The columns of R have meaning

$R = [\hat{\mathbf{x}}_b \mid \hat{\mathbf{y}}_b \mid \hat{\mathbf{z}}_b]$ — the columns are the basis vectors of frame $b$ expressed in frame $a$. This is a fast way to build $R$ by inspection: know where the child frame's axes point in the parent frame, stack them as columns.

---

## Composition and inversion

**Composition.** If a vector moves through three frames $a \leftarrow b \leftarrow c$:

$$
R^a_c = R^a_b \, R^b_c
$$

Rotation composition is matrix multiplication — non-commutative in general. $R_1 R_2 \neq R_2 R_1$.

**Inversion.** Because $R$ is orthogonal:

$$
R^{-1} = R^T
$$

This makes inverting a rotation free (just transpose) — a major reason rotation matrices are convenient for computation.

---

## Standard elementary rotations

Rotations about the principal axes:

$$
R_x(\theta) = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos\theta & -\sin\theta \\ 0 & \sin\theta & \cos\theta \end{bmatrix}
$$

$$
R_y(\theta) = \begin{bmatrix} \cos\theta & 0 & \sin\theta \\ 0 & 1 & 0 \\ -\sin\theta & 0 & \cos\theta \end{bmatrix}
$$

$$
R_z(\theta) = \begin{bmatrix} \cos\theta & -\sin\theta & 0 \\ \sin\theta & \cos\theta & 0 \\ 0 & 0 & 1 \end{bmatrix}
$$

Any $R \in SO(3)$ factors into a product of three of these — that is what [[Euler_Angles]] are.

---

## Axis-angle (Rodrigues) form

Any rotation can also be parameterized by a unit axis $\hat{\boldsymbol{\omega}} \in \mathbb{R}^3$ and angle $\theta$:

$$
R = I + \sin\theta \, [\hat{\boldsymbol{\omega}}]_\times + (1 - \cos\theta) \, [\hat{\boldsymbol{\omega}}]_\times^2
$$

where $[\hat{\boldsymbol{\omega}}]_\times$ is the $3 \times 3$ skew-symmetric matrix

$$
[\hat{\boldsymbol{\omega}}]_\times = \begin{bmatrix} 0 & -\omega_z & \omega_y \\ \omega_z & 0 & -\omega_x \\ -\omega_y & \omega_x & 0 \end{bmatrix}
$$

This is **Rodrigues' rotation formula**. It is the exponential map $\exp([\hat{\boldsymbol{\omega}}]_\times \theta)$ of the Lie algebra $\mathfrak{so}(3)$ — see [[Lie_Groups]].

---

## Comparison of rotation representations

| Representation | Parameters | Pros | Cons |
|---|---|---|---|
| Rotation matrix | 9 (6 constraints) | Composition = mat-mul; no singularities | Redundant; drifts under integration |
| Euler angles (e.g. XYZ) | 3 | Intuitive; compact | [[Gimbal_Lock]] singularities; convention-dependent |
| Axis-angle | 4 (or 3 if $\theta$ folded in) | Compact; no singularities | Composition is complicated |
| [[Quaternions]] | 4 (1 constraint) | No singularities; smooth interpolation; cheap composition | Double cover of SO(3); less intuitive |

**Rule of thumb for robotics code:** store orientation as a unit quaternion (4 floats, stable), convert to rotation matrix when you need to rotate vectors, and use Euler angles only for human display.

---

## Orthogonality drift and renormalization

Numerical integration ($R_{k+1} \approx R_k + \Delta t \cdot \dot{R}_k$) slowly violates $R^T R = I$. Two standard fixes:

1. **SVD projection.** Compute $R = U \Sigma V^T$, replace with $U V^T$. Gives the closest rotation matrix in Frobenius norm.
2. **Quaternion re-normalize.** If using quaternions, divide by norm each step.

Symplectic or exponential-map integrators avoid drift by construction and are preferred for long simulations.

---

## Derivative of a rotation

The time derivative of an orientation satisfies

$$
\dot{R} = [\boldsymbol{\omega}]_\times \, R
$$

where $\boldsymbol{\omega}$ is the angular velocity in the world frame. If $\boldsymbol{\omega}$ is expressed in the body frame, use $\dot{R} = R \, [\boldsymbol{\omega}^b]_\times$ instead. Getting this left-vs-right wrong is the classical source of sign errors in attitude estimators.

---

## Where rotation matrices appear

- Coordinate-frame transformations at every level of a [[TF]] tree in ROS 2
- [[Forward_Kinematics]] composition of joint rotations
- Camera extrinsics in multi-view geometry
- IMU attitude estimation
- Solid-body simulation

---

## Dataview

```dataview
LIST FROM #spatial-math OR #rotation WHERE contains(file.outlinks, [[Rotation_Matrix]])
```
