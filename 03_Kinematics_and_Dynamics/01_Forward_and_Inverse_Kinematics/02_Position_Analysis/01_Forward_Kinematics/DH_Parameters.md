---
title: DH Parameters (Denavit-Hartenberg)
description: A standardized 4-parameter convention for describing each joint of a serial-chain manipulator. Compact and historic, but tied to a specific frame-placement rule that has two competing variants.
tags:
  - kinematics
  - dh-parameters
  - forward-kinematics
  - manipulator
  - convention
layout: default
category: robotics
author: Jordan_Smith
date: 2025-05-02
permalink: /dh_parameters/
related:
  - "[[Kinematics]]"
  - "[[Forward_Kinematics]]"
  - "[[Product_of_Exponentials]]"
  - "[[Homogeneous_Transformation]]"
  - "[[Joints]]"
  - "[[Joint_Kinematics]]"
  - "[[Jacobian_Matrix]]"
---

# DH Parameters (Denavit-Hartenberg)

The **Denavit-Hartenberg (DH) parameters** are a four-number-per-joint convention for describing the kinematic structure of a serial-chain manipulator. Each joint is parameterized by $(a_i, \alpha_i, d_i, \theta_i)$ — link length, link twist, link offset, joint angle — and a single $4 \times 4$ homogeneous transform encapsulates the relative pose between consecutive link frames.

> **Etymology.** Named after **Jacques Denavit** and **Richard Scheunemann Hartenberg**, who introduced the convention in their 1955 ASME paper *A kinematic notation for lower-pair mechanisms based on matrices*. The motivation: every kinematic-pair connection has *two* axes (the joint axis and the link's geometric axis) — so any two consecutive link frames can be related by *exactly four* parameters, not six. The reduction is a consequence of the **lower pair** structure of revolute and prismatic joints.

---

## The four parameters

| Symbol | Name | Meaning |
|---|---|---|
| $a_i$ | **Link length** | Distance from $z_{i-1}$ to $z_i$ measured along $x_i$ (the common normal) |
| $\alpha_i$ | **Link twist** | Angle from $z_{i-1}$ to $z_i$ measured about $x_i$ |
| $d_i$ | **Link offset** | Distance from $x_{i-1}$ to $x_i$ measured along $z_{i-1}$ |
| $\theta_i$ | **Joint angle** | Angle from $x_{i-1}$ to $x_i$ measured about $z_{i-1}$ |

For a **revolute** joint, $\theta_i$ is the variable. For a **prismatic** joint, $d_i$ is the variable. The other three are fixed link constants.

The reduction from 6 pose parameters to 4 DH parameters works because:

- $z_i$ is *defined* to be the joint axis (no translation along $x_i$ needed for the axis itself).
- $x_i$ is *defined* to be the common normal between $z_{i-1}$ and $z_i$.

These two rules absorb two degrees of freedom in frame-placement, leaving only four numbers.

---

## The transformation

Each joint contributes a transform built from a fixed sequence of elementary rotations and translations:

$$
T_{i-1, i} = \text{Rot}_z(\theta_i) \cdot \text{Trans}_z(d_i) \cdot \text{Trans}_x(a_i) \cdot \text{Rot}_x(\alpha_i)
$$

Multiplied out:

$$
T_{i-1, i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i \cos\alpha_i & \sin\theta_i \sin\alpha_i & a_i \cos\theta_i \\
\sin\theta_i & \cos\theta_i \cos\alpha_i & -\cos\theta_i \sin\alpha_i & a_i \sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

[[Forward_Kinematics|Forward kinematics]] is the product:

$$
T_{0, n} = T_{0, 1}(\theta_1) \cdot T_{1, 2}(\theta_2) \cdots T_{n-1, n}(\theta_n)
$$

---

## Classic vs. modified DH — the convention split

This is where DH causes confusion. Two conventions coexist in the literature:

| | **Classic DH** (Spong, Sciavicco) | **Modified DH** (Craig) |
|---|---|---|
| Frame index | $z_i$ = axis of joint $i+1$ | $z_i$ = axis of joint $i$ |
| Order in transform | Rot$_z$, Trans$_z$, Trans$_x$, Rot$_x$ | Rot$_x$, Trans$_x$, Rot$_z$, Trans$_z$ |
| First link constants | $\alpha_0, a_0$ undefined | $\alpha_0, a_0$ defined |
| Software | KDL, Peter Corke RTB (default), older Drake | RDE, modern Drake (Craig variant), some Pinocchio |

They give the *same physical kinematics* but the parameter values differ. A robot's "DH table" must specify which convention it follows, or downstream code will produce wrong answers. **A vault entry that just says "use these DH parameters" without the convention is a bug waiting to happen.**

---

## Why DH is going out of fashion

Three friction points have pushed modern textbooks toward [[Product_of_Exponentials]] (PoE):

1. **Frame-placement is fiddly.** The rules for assigning $z_i$ and $x_i$ are well-defined but tedious, and slightly different between the two conventions.
2. **No frame for parallel axes.** When two consecutive joint axes are parallel, the common normal is non-unique — DH lets you pick, but two engineers will pick differently.
3. **Mostly used for FK.** Once you have $T_{0,n}$, the DH numbers themselves are no longer needed. PoE can do FK directly from screw axes without per-link frames.

PoE-based descriptions in modern textbooks (Lynch & Park, Murray-Li-Sastry) use 6-parameter screw axes but eliminate the convention-ambiguity entirely. URDF — the dominant robot description format in ROS — describes joints by axis-of-motion in a parent link's frame, much closer to PoE than to DH.

That said, DH still appears constantly:

- Manufacturer datasheets (older industrial arms ship with DH tables).
- Academic textbooks (Spong, Craig, Sciavicco are still standard curricula).
- Code-generation tools like `IKFast` accept DH input.
- Embedded controllers shipped with classical industrial arms.

So learn it, but learn PoE alongside.

---

## Worked example — Stanford arm (classic DH, fragment)

The Stanford arm (one of the first 6-DoF robots, 1969) has DH table:

| $i$ | $a_i$ | $\alpha_i$ | $d_i$ | $\theta_i$ |
|---|---|---|---|---|
| 1 | 0 | $-\pi/2$ | 0 | $\theta_1$ |
| 2 | 0 | $\pi/2$ | $d_2$ | $\theta_2$ |
| 3 | 0 | 0 | $d_3$ | 0 (prismatic) |
| 4 | 0 | $-\pi/2$ | 0 | $\theta_4$ |
| 5 | 0 | $\pi/2$ | 0 | $\theta_5$ |
| 6 | 0 | 0 | $d_6$ | $\theta_6$ |

Notice: joint 3 is prismatic (variable is $d_3$, $\theta_3 = 0$). Joints 4-5-6 form a spherical wrist (axes intersect), which is why this robot admits closed-form [[Inverse_Kinematics|inverse kinematics]].

---

## Common conventions for joint variable

The variable joint coordinate enters at *one slot* of the four parameters:

| Joint type | Variable | Constants |
|---|---|---|
| Revolute | $\theta_i$ | $a_i, \alpha_i, d_i$ |
| Prismatic | $d_i$ | $a_i, \alpha_i, \theta_i$ |
| Cylindrical (compound) | $\theta_i, d_i$ | $a_i, \alpha_i$ |

---

## Practical advice

1. **Always state the convention.** "DH(classic)" or "DH(modified)" — never just "DH."
2. **Check against URDF.** If you have a URDF for the same robot, derive DH from URDF (or vice versa) and compare end-effector poses at a few configurations. They must match.
3. **Don't hand-derive DH for new robots.** Use a CAD-to-DH tool, or describe the robot in URDF/SDF and let the kinematics solver consume that directly.
4. **Use PoE for new code.** It eliminates the convention ambiguity and integrates more naturally with modern Lie-group estimators and optimization.

---

## Recommended reading

- Denavit & Hartenberg (1955), *A kinematic notation for lower-pair mechanisms based on matrices* — the original
- Spong, Hutchinson, Vidyasagar, *Robot Modeling and Control*, Ch. 3 — classic DH
- Craig, *Introduction to Robotics: Mechanics and Control*, Ch. 3 — modified DH
- Lynch & Park, *Modern Robotics*, Appendix C — DH from a PoE perspective
- Hartenberg & Denavit, *Kinematic Synthesis of Linkages* (1964) — the textbook follow-up

---

## Dataview

```dataview
LIST FROM #kinematics OR #dh-parameters WHERE contains(file.outlinks, [[DH_Parameters]])
```
