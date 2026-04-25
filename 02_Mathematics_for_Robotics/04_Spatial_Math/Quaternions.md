---
title: Quaternions
description: A 4-dimensional number system that gives a singularity-free, compact, numerically stable representation of 3D rotations. The default choice for attitude representation in modern robotics and graphics.
tags:
  - mathematics
  - spatial-math
  - rotation
  - quaternions
  - SO3
  - robotics
  - orientation
layout: default
category: mathematics
author: Jordan_Smith
date: 2025-05-02
permalink: /quaternions/
related:
  - "[[Rotation_Matrix]]"
  - "[[Euler_Angles]]"
  - "[[Axis_Angle]]"
  - "[[Lie_Groups]]"
  - "[[Linear_Algebra_for_Robotics]]"
---

# Quaternions

A **quaternion** is a 4-dimensional number with one real part and three imaginary parts:

$$
q = w + x\,i + y\,j + z\,k
$$

where $i$, $j$, $k$ are imaginary units satisfying Hamilton's relations:

$$
i^2 = j^2 = k^2 = ijk = -1
$$

From those relations every other product follows — for example $ij = k$, $ji = -k$, $jk = i$, $kj = -i$. Multiplication is associative but **not commutative**, which is exactly what 3D rotations require.

> **Etymology.** Coined by William Rowan Hamilton on 16 October 1843 while walking across Brougham Bridge in Dublin; *quaternion* is from Latin *quaternio*, "group of four." He literally carved $i^2 = j^2 = k^2 = ijk = -1$ into the stone of the bridge.

---

## Unit quaternions and SO(3)

A **unit quaternion** satisfies $\lVert q \rVert = \sqrt{w^2 + x^2 + y^2 + z^2} = 1$. Unit quaternions parameterize rotations via:

$$
q = \left[\cos\tfrac{\theta}{2}, \; \hat{\mathbf{u}} \sin\tfrac{\theta}{2}\right]
$$

where $\hat{\mathbf{u}}$ is the unit rotation axis and $\theta$ is the rotation angle. The half-angle comes from the algebra — the unit quaternions form a **double cover** of the rotation group: $q$ and $-q$ represent the same rotation.

Geometrically: unit quaternions are the 3-sphere $S^3 \subset \mathbb{R}^4$, and the map $q \mapsto R(q)$ is 2-to-1 onto $SO(3)$. This double cover is why quaternion-based controllers must sometimes "shortest-path" by flipping sign when $q$ and the target are on opposite hemispheres.

---

## Rotating a vector with a quaternion

To rotate a 3-vector $\mathbf{v}$ by quaternion $q$, treat $\mathbf{v}$ as a pure quaternion $\mathbf{v} = [0, v_x, v_y, v_z]$ and compute:

$$
\mathbf{v}' = q \, \mathbf{v} \, q^{-1}
$$

For unit quaternions, $q^{-1} = q^* = [w, -x, -y, -z]$ (the conjugate).

In practice libraries give you a single `rotate(q, v)` function that does the sandwich product in-place with $\sim 15$ floating-point multiplies — cheaper than the $9$ mul + $6$ add of a rotation-matrix multiply only when the rotation is constructed fresh each call.

---

## Quaternion composition

Concatenating two rotations corresponds to quaternion multiplication:

$$
q_{\text{combined}} = q_2 \, q_1
$$

(apply $q_1$ first, then $q_2$). The product is defined by:

$$
q_1 q_2 = \begin{bmatrix} w_1 w_2 - \mathbf{v}_1 \cdot \mathbf{v}_2 \\ w_1 \mathbf{v}_2 + w_2 \mathbf{v}_1 + \mathbf{v}_1 \times \mathbf{v}_2 \end{bmatrix}
$$

where $q_i = [w_i, \mathbf{v}_i]$.

---

## Conversion formulas

### Quaternion to rotation matrix

$$
R(q) = \begin{bmatrix}
1 - 2(y^2 + z^2) & 2(xy - wz) & 2(xz + wy) \\
2(xy + wz) & 1 - 2(x^2 + z^2) & 2(yz - wx) \\
2(xz - wy) & 2(yz + wx) & 1 - 2(x^2 + y^2)
\end{bmatrix}
$$

### Axis-angle to quaternion

$$
q = \left[\cos\tfrac{\theta}{2},\; u_x \sin\tfrac{\theta}{2},\; u_y \sin\tfrac{\theta}{2},\; u_z \sin\tfrac{\theta}{2}\right]
$$

### Rotation matrix to quaternion

Several numerically stable formulations exist (Shepperd's method, the trace-based formula). In code, use a library — Eigen, scipy.spatial.transform, or ROS 2's tf2 all have battle-tested implementations.

---

## Why quaternions are the robotics default

**Advantages over Euler angles:**
- No [[Gimbal_Lock]] singularities
- Smooth interpolation (SLERP) between two orientations
- Convention-free — there are 24 Euler-angle conventions and they disagree

**Advantages over rotation matrices:**
- 4 numbers instead of 9
- One constraint ($\lVert q \rVert = 1$) instead of six
- Renormalization is one scalar division instead of an SVD
- Numerically stable under long integration (attitude propagation on IMUs)

**Advantages for interpolation — SLERP.** Given two unit quaternions $q_0$ and $q_1$ and a parameter $t \in [0, 1]$:

$$
\text{SLERP}(q_0, q_1, t) = \frac{\sin((1-t)\Omega)}{\sin\Omega} q_0 + \frac{\sin(t\Omega)}{\sin\Omega} q_1
$$

where $\cos\Omega = q_0 \cdot q_1$. SLERP traces a great-circle arc on $S^3$ — it corresponds to constant angular velocity interpolation in SO(3). For small $\Omega$, use linear interpolation + normalization (NLERP) to avoid numerical issues.

---

## Angular velocity and integration

Given angular velocity $\boldsymbol{\omega}$ in the body frame, the quaternion evolves as:

$$
\dot{q} = \tfrac{1}{2} q \, [0, \boldsymbol{\omega}]
$$

For numerical integration at time step $\Delta t$, the stable update is:

$$
q_{k+1} = q_k \cdot \exp\!\left(\tfrac{\Delta t}{2} [0, \boldsymbol{\omega}_k]\right), \quad \text{then normalize}
$$

This is the standard attitude-kinematics equation for IMU integration and quadrotor attitude controllers.

---

## Common pitfalls

- **Sign ambiguity.** $q$ and $-q$ represent the same rotation, but differ in SLERP/interpolation logic. Always pick the sign that makes $q \cdot q_\text{target} \geq 0$ (shorter arc).
- **Convention confusion.** Quaternions may be stored as $[w, x, y, z]$ (scalar-first, common in robotics) or $[x, y, z, w]$ (scalar-last, common in graphics/Unity). Check your library.
- **Normalization drift.** Every step of numerical integration slightly violates $\lVert q \rVert = 1$. Normalize after every update.
- **Multiplication order.** `q * p` in most libraries applies $p$ first, then $q$ — opposite of what some textbooks write. Test against a known case.

---

## Implementation notes

- **Eigen** (C++): `Eigen::Quaterniond q(w, x, y, z);` — note scalar-first.
- **scipy.spatial.transform.Rotation.from_quat([x, y, z, w])` — note scalar-last.
- **tf2** (ROS 2): `geometry_msgs::msg::Quaternion` has `.x`, `.y`, `.z`, `.w`.
- **GLM / Unity / Unreal**: scalar-last.

---

## Where quaternions appear in robotics

- Attitude estimation on IMUs (Madgwick, Mahony, MEKF)
- Drone flight controllers (PX4, ArduPilot use quaternion attitude)
- End-effector orientation targets in IK
- Trajectory interpolation and orientation planning
- Graphics pipelines and simulators (MuJoCo, Isaac Sim, Unreal)

---

## Dataview

```dataview
LIST FROM #rotation OR #quaternions WHERE contains(file.outlinks, [[Quaternions]])
```
