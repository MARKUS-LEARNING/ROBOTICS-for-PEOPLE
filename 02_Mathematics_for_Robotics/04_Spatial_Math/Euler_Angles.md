---
title: Euler Angles
description: A three-parameter representation of 3D orientation as a sequence of elementary rotations about principal axes. Compact and human-readable, but plagued by gimbal-lock singularities and convention ambiguity.
tags:
  - mathematics
  - spatial-math
  - rotation
  - euler-angles
  - orientation
  - robotics
layout: default
category: mathematics
author: Jordan_Smith
date: 2025-05-02
permalink: /euler_angles/
related:
  - "[[Rotation_Matrix]]"
  - "[[Quaternions]]"
  - "[[Axis_Angle]]"
  - "[[Gimbal_Lock]]"
  - "[[Linear_Algebra_for_Robotics]]"
---

# Euler Angles

**Euler angles** are a three-number parameterization of a 3D rotation obtained by composing three elementary rotations about the principal axes. Named after Leonhard Euler, who proved that every orientation of a rigid body can be reached by three such rotations.

> **Etymology.** Euler introduced the representation in his 1775 paper *Formulae generales pro translatione quacunque corporum rigidorum*. The theorem: any rotation $R \in SO(3)$ can be written as a composition of three rotations about (at least) two distinct axes.

---

## The three rotations

Choose a triple of axes — e.g., $Z$-$Y$-$X$. Then any rotation decomposes as:

$$
R(\alpha, \beta, \gamma) = R_Z(\alpha) \, R_Y(\beta) \, R_X(\gamma)
$$

where $\alpha, \beta, \gamma$ are the three Euler angles. The choice of axis sequence is the **convention** — there are twelve, and they disagree.

---

## Convention zoo (this is where bugs happen)

Two independent choices generate the conventions:

1. **Axis order.** 12 combinations: three proper Euler sequences (XYX, XZX, YXY, YZY, ZXZ, ZYZ) and three Tait-Bryan sequences (XYZ, XZY, YXZ, YZX, ZXY, ZYX) — each in two variants (intrinsic vs. extrinsic).
2. **Intrinsic vs. extrinsic.**
   - **Intrinsic:** each rotation is about the *new* (rotated) frame's axis.
   - **Extrinsic:** each rotation is about the *original* (fixed) frame's axis.

Intrinsic $(XYZ)$ is mathematically identical to extrinsic $(ZYX)$ — the sequences reverse. Libraries usually document which they implement.

### Common conventions in practice

| Convention | Where it's used | Angle names |
|---|---|---|
| **ZYX intrinsic** (= XYZ extrinsic) | Aerospace, ROS | yaw, pitch, roll |
| **ZXZ** | Classical mechanics | $\phi$, $\theta$, $\psi$ |
| **XYZ** Tait-Bryan | Robotics, computer graphics | roll, pitch, yaw |

**Yaw / pitch / roll** specifically:
- **Yaw** — rotation about body $Z$ (vertical). Turning left/right.
- **Pitch** — rotation about body $Y$. Nose up/down.
- **Roll** — rotation about body $X$ (forward). Tilting left/right.

---

## Example: ZYX intrinsic (aerospace yaw-pitch-roll)

$$
R = R_Z(\text{yaw}) \, R_Y(\text{pitch}) \, R_X(\text{roll})
$$

Explicitly, with $c_\alpha = \cos\alpha$, $s_\alpha = \sin\alpha$:

$$
R = \begin{bmatrix}
c_y c_p & c_y s_p s_r - s_y c_r & c_y s_p c_r + s_y s_r \\
s_y c_p & s_y s_p s_r + c_y c_r & s_y s_p c_r - c_y s_r \\
-s_p & c_p s_r & c_p c_r
\end{bmatrix}
$$

where $y$ = yaw, $p$ = pitch, $r$ = roll.

---

## Gimbal lock

The fatal defect of Euler angles. When the middle rotation reaches a critical value (e.g., pitch $= \pm 90°$ in ZYX), the first and third axes align — the two end rotations become redundant and one degree of freedom is lost. Any rotation rate about the affected direction cannot be represented as a unique combination of Euler-angle rates.

**Symptom in attitude controllers:** a drone pitched near $\pm 90°$ exhibits sudden yaw jumps or unresponsive yaw control because the Jacobian relating body rates to Euler-angle rates blows up.

**Fix:** do not use Euler angles as the primary state. Use [[Quaternions]] or [[Rotation_Matrix|rotation matrices]] internally; convert to Euler only for logging or display.

The mapping from body angular rates $\boldsymbol{\omega}$ to Euler rates in ZYX is:

$$
\begin{bmatrix} \dot\phi \\ \dot\theta \\ \dot\psi \end{bmatrix} = \begin{bmatrix}
1 & \sin\phi \tan\theta & \cos\phi \tan\theta \\
0 & \cos\phi & -\sin\phi \\
0 & \sin\phi/\cos\theta & \cos\phi/\cos\theta
\end{bmatrix} \boldsymbol{\omega}
$$

The $1/\cos\theta$ terms explode at $\theta = \pm 90°$ — this is gimbal lock manifesting in the math.

---

## Extraction from a rotation matrix (ZYX intrinsic)

Given $R = [r_{ij}]$:

$$
\text{pitch} = -\arcsin(r_{31})
$$

$$
\text{roll} = \arctan2(r_{32},\, r_{33})
$$

$$
\text{yaw} = \arctan2(r_{21},\, r_{11})
$$

Always use `atan2(y, x)`, never `atan(y / x)` — the latter loses sign information. At gimbal lock ($r_{31} = \pm 1$) the extraction is not unique and libraries adopt a convention (usually setting roll to 0 and putting all yaw into one angle).

---

## When Euler angles are fine

Despite the convention and gimbal-lock problems, Euler angles remain useful when:

1. The orientation is guaranteed not to approach the singularity. A car on flat ground only needs yaw.
2. Human intuition matters more than numerical robustness. A pilot thinks in roll/pitch/yaw; so does a log file viewer.
3. The rotation is constrained to a single plane (planar robots).

Don't store them as state; don't integrate them; don't compose them by adding the angles. Use as I/O, not internal representation.

---

## Quick reference: pick a representation

| Task | Representation |
|---|---|
| Attitude estimation on IMU | Quaternion |
| Transformation of a point cloud | Rotation matrix (or homogeneous transform) |
| Target orientation in a GUI slider | Euler angles |
| Reporting drone attitude in a flight log | Euler angles (yaw/pitch/roll) |
| Interpolating a camera between keyframes | Quaternion (SLERP) |
| Solving inverse kinematics | Quaternion or rotation matrix |

---

## Dataview

```dataview
LIST FROM #rotation OR #euler-angles WHERE contains(file.outlinks, [[Euler_Angles]])
```
