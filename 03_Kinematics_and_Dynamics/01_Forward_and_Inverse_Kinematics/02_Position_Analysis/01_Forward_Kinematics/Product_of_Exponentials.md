---
title: Product of Exponentials (PoE)
description: A coordinate-free formulation of forward kinematics using screw axes and the matrix exponential on SE(3). Geometrically transparent, free of per-link frame conventions, and the modern alternative to Denavit-Hartenberg.
tags:
  - kinematics
  - product-of-exponentials
  - forward-kinematics
  - screw-theory
  - lie-groups
  - SE3
layout: default
category: robotics
author: Jordan_Smith
date: 2025-05-02
permalink: /product_of_exponentials/
related:
  - "[[Kinematics]]"
  - "[[Forward_Kinematics]]"
  - "[[DH_Parameters]]"
  - "[[Lie_Groups]]"
  - "[[Homogeneous_Transformation]]"
  - "[[Jacobian_Matrix]]"
  - "[[Joints]]"
---

# Product of Exponentials (PoE)

The **Product of Exponentials (PoE)** formula expresses the [[Forward_Kinematics|forward kinematics]] of a serial-chain manipulator as a product of matrix exponentials on the Lie group [[Lie_Groups|SE(3)]]:

$$
T_{\text{ee}}(\boldsymbol{\theta}) = e^{[\mathcal{S}_1] \theta_1} \, e^{[\mathcal{S}_2] \theta_2} \cdots e^{[\mathcal{S}_n] \theta_n} \, M
$$

where $\mathcal{S}_i \in \mathbb{R}^6$ is the screw axis of joint $i$ (in the base frame, at the home configuration), $[\mathcal{S}_i] \in \mathfrak{se}(3)$ is its $4 \times 4$ matrix form, and $M \in SE(3)$ is the home-configuration end-effector pose.

> **Etymology.** *Exponential* — Latin *exponere*, "to set forth" or "to expose," via the mathematical idea of an *exponent* (something "set above" the base in $a^n$). The *matrix exponential* $e^A = I + A + A^2/2! + \cdots$ extends the scalar exponential to matrices and was studied systematically in the late 19th century. *Product of exponentials* in robotics was introduced by **Roger Brockett** at Harvard in 1984 (*Robotic manipulators and the product of exponentials formula*) as a coordinate-free alternative to [[DH_Parameters|Denavit-Hartenberg]].

---

## What is a screw axis?

A **screw axis** $\mathcal{S} \in \mathbb{R}^6$ packages the geometry of a one-DoF rigid-body motion (rotation + translation along the same axis) into a single 6-vector:

$$
\mathcal{S} = \begin{bmatrix} \boldsymbol{\omega} \\ \mathbf{v} \end{bmatrix}
$$

where:

- $\boldsymbol{\omega} \in \mathbb{R}^3$ is the unit axis of rotation (or $\mathbf{0}$ for a pure translation).
- $\mathbf{v} \in \mathbb{R}^3$ is $-\boldsymbol{\omega} \times \mathbf{q} + h \boldsymbol{\omega}$, where $\mathbf{q}$ is any point on the axis and $h$ is the *pitch* (translation per radian of rotation).

For a pure **revolute** joint with axis through point $\mathbf{q}$: $\boldsymbol{\omega}$ is the unit rotation axis, $\mathbf{v} = -\boldsymbol{\omega} \times \mathbf{q}$, $h = 0$.

For a pure **prismatic** joint along direction $\hat{\mathbf{v}}$: $\boldsymbol{\omega} = \mathbf{0}$, $\mathbf{v} = \hat{\mathbf{v}}$.

A screw axis tells you both *where* the axis is (through point $\mathbf{q}$) and *what* it does (rotate, translate, or both). It is a single object encoding what DH spreads across four parameters.

---

## The bracket and matrix exponential

The bracket $[\mathcal{S}] \in \mathfrak{se}(3)$ embeds the 6-vector into a $4 \times 4$ matrix:

$$
[\mathcal{S}] = \begin{bmatrix} [\boldsymbol{\omega}]_\times & \mathbf{v} \\ \mathbf{0}^T & 0 \end{bmatrix}
$$

where $[\boldsymbol{\omega}]_\times$ is the skew-symmetric $3 \times 3$ matrix.

The matrix exponential $e^{[\mathcal{S}] \theta} \in SE(3)$ is the rigid-body transform produced by moving distance $\theta$ along the screw. It generalizes Euler's formula $e^{i\theta} = \cos\theta + i \sin\theta$ to rigid-body motions. Closed-form via the **Rodrigues-style formula** for SE(3):

$$
e^{[\mathcal{S}] \theta} = \begin{bmatrix} e^{[\boldsymbol{\omega}]_\times \theta} & G(\theta) \mathbf{v} \\ \mathbf{0}^T & 1 \end{bmatrix}
$$

with $G(\theta) = I \theta + (1 - \cos\theta)[\boldsymbol{\omega}]_\times + (\theta - \sin\theta)[\boldsymbol{\omega}]_\times^2$. See [[Lie_Groups]] for the full derivation.

---

## How to read the formula

For a chain with joints labeled 1 through $n$ from base to tool:

1. Place the robot in any **home configuration** — typically the geometric "zero" pose.
2. Record the home pose $M = T_{\text{ee}}(\mathbf{0})$.
3. For each joint $i$, identify its screw axis $\mathcal{S}_i$ in the **base frame**, with the robot at the home configuration.
4. Forward kinematics is then:

$$
T_{\text{ee}}(\boldsymbol{\theta}) = \underbrace{e^{[\mathcal{S}_1] \theta_1}}_{\text{joint 1 motion}} \, \underbrace{e^{[\mathcal{S}_2] \theta_2}}_{\text{joint 2 motion}} \cdots \underbrace{e^{[\mathcal{S}_n] \theta_n}}_{\text{joint n motion}} \, M
$$

Each exponential is a rigid-body motion *from the base frame's point of view*. The home pose $M$ collects the part of the kinematics that is independent of joint angles — the link lengths, twist offsets, and tool placement — into a single $SE(3)$ element.

There is also a **body form** that uses screw axes expressed in the end-effector frame and writes the product right-to-left, related to the space form by adjoint conjugation.

---

## What PoE buys you over DH

| | **DH** | **PoE** |
|---|---|---|
| Parameters per joint | 4 numbers | 6-vector screw + once-per-robot $M$ |
| Per-link frame placement | Required | Not required |
| Convention variants | 2 (Spong/Craig) | 1 |
| Geometric meaning | Indirect (frame-placement choice) | Direct (axes you can see in CAD) |
| Parallel axes | Common-normal ambiguous | No issue |
| Ties to Lie theory | None | Native |
| Computes Jacobian | Manually | One adjoint per column |
| Modern textbooks | Older curricula | Lynch & Park, Murray-Li-Sastry |

The PoE Jacobian is particularly clean: column $i$ of the **space Jacobian** is the screw axis $\mathcal{S}_i$ transported to the current configuration:

$$
J_{s, i}(\boldsymbol{\theta}) = \text{Ad}_{\left(e^{[\mathcal{S}_1] \theta_1} \cdots e^{[\mathcal{S}_{i-1}] \theta_{i-1}}\right)} \mathcal{S}_i
$$

See [[Jacobian_Matrix]] and [[Lie_Groups]].

---

## Worked example — 2R planar arm

Two revolute joints, both with axis along $\hat{\mathbf{z}}$, joint 1 at the origin and joint 2 at $(L_1, 0, 0)$. Home pose has the end-effector at $(L_1 + L_2, 0, 0)$.

Screw axes:

$$
\mathcal{S}_1 = \begin{bmatrix} 0 & 0 & 1 & 0 & 0 & 0 \end{bmatrix}^T \quad (\text{rotation about } \hat{\mathbf{z}} \text{ at origin})
$$

$$
\mathcal{S}_2 = \begin{bmatrix} 0 & 0 & 1 & 0 & -L_1 & 0 \end{bmatrix}^T \quad (\text{rotation about } \hat{\mathbf{z}} \text{ at } (L_1, 0, 0))
$$

Home pose:

$$
M = \begin{bmatrix} I_3 & (L_1 + L_2, 0, 0)^T \\ \mathbf{0}^T & 1 \end{bmatrix}
$$

Forward kinematics:

$$
T_{\text{ee}}(\theta_1, \theta_2) = e^{[\mathcal{S}_1] \theta_1} e^{[\mathcal{S}_2] \theta_2} M
$$

Multiply this out and you recover $x = L_1 \cos\theta_1 + L_2 \cos(\theta_1 + \theta_2)$, $y = L_1 \sin\theta_1 + L_2 \sin(\theta_1 + \theta_2)$ — the same answer as the trig-style derivation, but obtained by a uniform recipe.

---

## Why PoE plays nicely with modern robotics stacks

- **URDF parsing.** URDF specifies each joint by its axis vector and origin in the parent frame. That is essentially a screw axis. PoE-based libraries consume URDF directly without translating to DH.
- **Lie-group estimators.** EKFs and factor-graph optimizers (GTSAM, Ceres) increasingly do their math on SO(3)/SE(3) directly. PoE's exponential map is the same map those filters use for state perturbations — see [[Lie_Groups]].
- **Autodifferentiation.** Tools like CasADi, JAX, and PyTorch can autodiff through the matrix exponential, giving free Jacobians and Hessians for trajectory optimization.
- **Sim-to-real friendly.** No frame-convention drift between simulator and physical robot — both use the same screw axes from the URDF.

---

## Practical implementation

```python
# numpy-style PoE forward kinematics
import numpy as np
from scipy.linalg import expm

def screw_to_se3(S):
    """6-vector screw to 4x4 se(3) matrix."""
    w, v = S[:3], S[3:]
    W = np.array([[ 0,    -w[2],  w[1]],
                  [ w[2],  0,    -w[0]],
                  [-w[1],  w[0],  0   ]])
    out = np.zeros((4, 4))
    out[:3, :3] = W
    out[:3, 3]  = v
    return out

def fk_poe_space(M, screws, thetas):
    """Forward kinematics: T_ee = exp([S1]th1) ... exp([Sn]thn) M"""
    T = np.eye(4)
    for S, th in zip(screws, thetas):
        T = T @ expm(screw_to_se3(S) * th)
    return T @ M
```

Production implementations live in **Pinocchio**, **Drake** (`MultibodyPlant`), the **Modern Robotics** Python library, and **manif**.

---

## Recommended reading

- Lynch & Park, *Modern Robotics*, Ch. 4 — PoE forward kinematics worked from scratch (free PDF + companion library)
- Murray, Li, Sastry, *A Mathematical Introduction to Robotic Manipulation*, Ch. 2-3 — screw theory and exponential coordinates (free PDF)
- Brockett, R. (1984). *Robotic manipulators and the product of exponentials formula* — the original paper
- Solà, Deray, Atchuthan (2018), *A micro Lie theory for state estimation in robotics* (arXiv 1812.01537) — clean Lie-theoretic treatment

---

## Dataview

```dataview
LIST FROM #kinematics OR #product-of-exponentials WHERE contains(file.outlinks, [[Product_of_Exponentials]])
```
