---
title: Lie Groups (SO(3), SE(3))
description: Smooth manifolds that are also groups under multiplication. The natural home of rigid-body rotation (SO(3)) and rigid-body motion (SE(3)), and the mathematical framework behind modern state estimation and control on manifolds.
tags:
  - mathematics
  - spatial-math
  - lie-groups
  - SO3
  - SE3
  - manifolds
  - robotics
  - estimation
layout: default
category: mathematics
author: Jordan_Smith
date: 2025-05-02
permalink: /lie_groups/
related:
  - "[[Rotation_Matrix]]"
  - "[[Quaternions]]"
  - "[[Homogeneous_Transformation]]"
  - "[[Linear_Algebra_for_Robotics]]"
  - "[[Kalman_Filter]]"
---

# Lie Groups (SO(3), SE(3))

A **Lie group** is a smooth manifold that is also a group, where multiplication and inversion are smooth maps. In robotics, two specific Lie groups carry almost all the weight:

- **SO(3)** — the group of 3D rotations ($3 \times 3$ orthogonal matrices with $\det = +1$)
- **SE(3)** — the group of 3D rigid-body transformations (rotation + translation)

Named after Sophus Lie (Norwegian, 1842–1899), who studied continuous transformation groups in the 1870s.

> **Etymology.** Pronounced "Lee." Sophus Lie's work created *Lie theory*, which unifies continuous symmetries in mathematics and physics. The deep idea: a group whose elements can be continuously deformed has an associated linear space — its *Lie algebra* — that captures all the local structure.

---

## Why robotics needs Lie groups

Rotations and poses don't live in Euclidean space. You cannot meaningfully "add" two rotation matrices or average them coordinate-wise. Doing so produces a matrix that is not even a rotation. Standard linear tools — Kalman filters, least-squares estimators, interpolation — assume a vector-space structure that rotations lack.

Lie-group theory gives us two linked objects:

1. **The group $G$** (the manifold of valid orientations or poses)
2. **The Lie algebra $\mathfrak{g}$** (the tangent space at the identity — a vector space)

and a pair of maps, $\exp$ and $\log$, that move between them. The algebra is where we do linear-algebra-style math; the group is where physical objects actually live.

---

## SO(3) — the rotation group

**Elements:** $3 \times 3$ matrices $R$ with $R^T R = I$ and $\det R = +1$.

**Dimension:** 3 (rotations have 3 DoF).

**Lie algebra $\mathfrak{so}(3)$:** $3 \times 3$ skew-symmetric matrices. Every skew-symmetric matrix corresponds to a 3-vector $\boldsymbol{\omega} \in \mathbb{R}^3$ (the "axis-angle" vector) via the hat operator:

$$
[\boldsymbol{\omega}]_\times = \begin{bmatrix} 0 & -\omega_z & \omega_y \\ \omega_z & 0 & -\omega_x \\ -\omega_y & \omega_x & 0 \end{bmatrix}
$$

**Exponential map** $\exp : \mathfrak{so}(3) \to SO(3)$: Rodrigues' formula.

$$
\exp([\boldsymbol{\omega}]_\times) = I + \frac{\sin\theta}{\theta}[\boldsymbol{\omega}]_\times + \frac{1 - \cos\theta}{\theta^2}[\boldsymbol{\omega}]_\times^2
$$

where $\theta = \lVert \boldsymbol{\omega} \rVert$. Geometrically, $\exp$ takes a tangent-space "velocity" vector and integrates it forward by one unit of time to reach a rotation.

**Logarithm** $\log : SO(3) \to \mathfrak{so}(3)$: the inverse. Given $R$, extract axis $\hat{\mathbf{u}}$ and angle $\theta$, return $\theta \hat{\mathbf{u}}$.

$$
\theta = \arccos\!\left(\tfrac{\text{tr}(R) - 1}{2}\right), \quad [\boldsymbol{\omega}]_\times = \tfrac{\theta}{2\sin\theta}(R - R^T)
$$

(Handle $\theta \to 0$ with a Taylor series to avoid division-by-zero.)

---

## SE(3) — the rigid-body group

**Elements:** $4 \times 4$ homogeneous transformation matrices.

$$
T = \begin{bmatrix} R & \mathbf{t} \\ \mathbf{0}^T & 1 \end{bmatrix}, \quad R \in SO(3), \; \mathbf{t} \in \mathbb{R}^3
$$

**Dimension:** 6 (3 for rotation + 3 for translation).

**Lie algebra $\mathfrak{se}(3)$:** $4 \times 4$ matrices of the form

$$
\hat{\boldsymbol{\xi}} = \begin{bmatrix} [\boldsymbol{\omega}]_\times & \mathbf{v} \\ \mathbf{0}^T & 0 \end{bmatrix}
$$

parameterized by a 6-vector $\boldsymbol{\xi} = [\mathbf{v}^T, \boldsymbol{\omega}^T]^T \in \mathbb{R}^6$ — a **twist**, combining linear and angular velocity.

**Exponential map** for SE(3):

$$
\exp(\hat{\boldsymbol{\xi}}) = \begin{bmatrix} \exp([\boldsymbol{\omega}]_\times) & V(\boldsymbol{\omega}) \mathbf{v} \\ \mathbf{0}^T & 1 \end{bmatrix}
$$

where $V(\boldsymbol{\omega})$ is the "left Jacobian of SO(3)" — a specific $3 \times 3$ matrix depending on $\boldsymbol{\omega}$.

---

## The $\boxplus$ and $\boxminus$ operators (on-manifold perturbations)

For state estimation on manifolds, the standard trick is to write state as $\mathbf{X} \in G$ (group element) and do Gaussian perturbations in the tangent space:

$$
\mathbf{X} \boxplus \boldsymbol{\delta} = \mathbf{X} \cdot \exp(\boldsymbol{\delta})
$$

$$
\mathbf{X} \boxminus \mathbf{Y} = \log(\mathbf{Y}^{-1} \mathbf{X})
$$

This is how modern estimators handle rotations cleanly:

- **On-manifold EKF**: covariance is $6 \times 6$ in the tangent space (not $16 \times 16$ in flattened $T$); updates use $\boxplus$.
- **Pose-graph optimization** (iSAM2, GTSAM, Ceres): Jacobians are taken w.r.t. tangent-space perturbations.
- **Preintegrated IMU factors**: integrate between keyframes in $SE(3)$ properly, not coordinate-by-coordinate.

---

## Why quaternions live here too

The unit quaternions form a Lie group themselves: $S^3 \cong \text{Spin}(3)$, the *double cover* of SO(3). They're a different parameterization of the same underlying manifold. In Lie-group terms, quaternions are a convenient chart on SO(3) whose exponential map is numerically well-behaved (no gimbal lock).

---

## Right vs. left perturbation conventions

A major source of sign errors: perturbations can be applied on the left or the right:

$$
\mathbf{X} \boxplus_r \boldsymbol{\delta} = \mathbf{X} \exp(\boldsymbol{\delta}) \quad\text{(right)}
$$

$$
\mathbf{X} \boxplus_l \boldsymbol{\delta} = \exp(\boldsymbol{\delta}) \mathbf{X} \quad\text{(left)}
$$

GTSAM, Sophus, and most vision libraries use **right** perturbations (body-frame). Be explicit about which you chose and stay consistent.

---

## Adjoint map

The **adjoint** $\text{Ad}_{\mathbf{X}}$ maps tangent vectors between different basepoints:

$$
\exp(\text{Ad}_{\mathbf{X}} \boldsymbol{\delta}) = \mathbf{X} \exp(\boldsymbol{\delta}) \mathbf{X}^{-1}
$$

For SE(3):

$$
\text{Ad}_T = \begin{bmatrix} R & [\mathbf{t}]_\times R \\ \mathbf{0} & R \end{bmatrix}
$$

Used constantly in dynamics (screw theory, spatial vectors) and estimation (transforming covariance between frames).

---

## Practical libraries

| Library | Language | Highlights |
|---|---|---|
| **Sophus** | C++ | Clean SO(3), SE(3), Sim(3) templates; Eigen-based |
| **GTSAM** | C++ (Python bindings) | Factor graphs, iSAM2, SLAM |
| **manif** | C++ / Python | Lie-theoretic types with explicit $\boxplus$/$\boxminus$ |
| **SE3 module in SciPy** | Python | Rotation class with exp/log |
| **Peter Corke's Robotics Toolbox** | Python/MATLAB | SE3, SO3 classes with extensive methods |

---

## What to read next

- Timothy Barfoot, *State Estimation for Robotics* — excellent, approachable
- Joan Solà et al., *A micro Lie theory for state estimation in robotics* (arXiv 1812.01537) — the tutorial everyone recommends
- Kevin Lynch & Frank Park, *Modern Robotics*, Chapter 3 — screw theory from the Lie-group perspective
- Frank Dellaert & Michael Kaess, *Factor Graphs for Robot Perception* — how these tools power SLAM

---

## Dataview

```dataview
LIST FROM #lie-groups OR #manifolds WHERE contains(file.outlinks, [[Lie_Groups]])
```
