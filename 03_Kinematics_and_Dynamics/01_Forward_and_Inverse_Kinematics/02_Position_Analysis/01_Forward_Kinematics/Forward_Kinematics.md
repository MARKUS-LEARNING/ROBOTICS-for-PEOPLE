---
title: Forward Kinematics (FK)
description: The map from joint angles to end-effector pose. A composition of rigid-body transforms, one per joint. The simplest, cheapest, and most-used computation in any manipulator stack.
tags:
  - kinematics
  - forward-kinematics
  - manipulator
  - mobile-robot
  - DH-parameters
  - product-of-exponentials
layout: default
category: robotics
author: Jordan_Smith
date: 2025-04-28
permalink: /forward_kinematics/
related:
  - "[[Kinematics]]"
  - "[[Inverse_Kinematics]]"
  - "[[Homogeneous_Transformation]]"
  - "[[DH_Parameters]]"
  - "[[Product_of_Exponentials]]"
  - "[[Jacobian_Matrix]]"
  - "[[Cartesian_Space]]"
  - "[[Configuration_Space]]"
  - "[[Lie_Groups]]"
---

# Forward Kinematics (FK)

**Forward Kinematics (FK)**, also called *direct kinematics*, computes the pose of a robot's end-effector from its joint variables. Given the joint angles $\boldsymbol{\theta} = [\theta_1, \theta_2, \ldots, \theta_n]^T$ of a serial manipulator, FK returns the $4 \times 4$ homogeneous transformation matrix $T_{\text{ee}} \in SE(3)$ that locates the gripper in the base frame.

> **Etymology.** *Forward* — Old English *foreweard*, "toward the front." Used here in the sense of *forward in the data flow*: input is the joint configuration (the cause), output is the resulting pose (the effect). Contrast with *[[Inverse_Kinematics|inverse kinematics]]*, which runs the data flow backwards.

The FK problem is the easiest of all kinematics problems: given enough geometric information, the answer is *always* unique and *always* computable in closed form by composing matrix multiplications.

---

## The fundamental equation

For a serial chain of $n$ joints:

$$
T_{\text{ee}}(\boldsymbol{\theta}) = T_{0,1}(\theta_1) \, T_{1,2}(\theta_2) \, \cdots \, T_{n-1, n}(\theta_n) \, T_{n, \text{tool}}
$$

where:

- $T_{i-1, i}(\theta_i)$ is the homogeneous transform from frame $i-1$ to frame $i$, parameterized by joint variable $\theta_i$.
- $T_{n, \text{tool}}$ is the fixed offset from the last link to the tool tip (often the identity).
- The product is **left-to-right multiplication of $4 \times 4$ matrices** — about 16 multiply-adds per joint.

Each $T_{i-1, i}(\theta_i)$ takes one of two canonical forms.

---

## Two parameterizations

### Denavit-Hartenberg (DH) convention

Introduced by Jacques Denavit and Richard Hartenberg in 1955, the **DH** convention assigns four parameters per joint $(a_i, \alpha_i, d_i, \theta_i)$ — link length, link twist, link offset, joint angle — and writes each transform as:

$$
T_{i-1, i} = \text{Rot}_z(\theta_i) \cdot \text{Trans}_z(d_i) \cdot \text{Trans}_x(a_i) \cdot \text{Rot}_x(\alpha_i)
$$

For a revolute joint, $\theta_i$ is the joint variable; for a prismatic joint, $d_i$ is. The rest are fixed link constants.

DH is compact (4 numbers/joint) and historic — every classical textbook uses it. But the frame-placement rules are fiddly, two conventions exist (classic Spong vs. modified Craig), and the choice of frame on each link is *not* unique. See [[DH_Parameters]].

### Product of Exponentials (PoE)

Brockett (1984) showed that any serial-chain FK can be written without per-link DH frames:

$$
T_{\text{ee}}(\boldsymbol{\theta}) = e^{[\mathcal{S}_1] \theta_1} \, e^{[\mathcal{S}_2] \theta_2} \, \cdots \, e^{[\mathcal{S}_n] \theta_n} \, M
$$

where:

- $\mathcal{S}_i \in \mathbb{R}^6$ is the **screw axis** of joint $i$ in the base frame at the home configuration.
- $[\mathcal{S}_i] \in \mathfrak{se}(3)$ is its $4 \times 4$ matrix form.
- $M \in SE(3)$ is the home-configuration end-effector pose.
- $e^{[\mathcal{S}_i] \theta_i}$ is the matrix exponential, a [[Lie_Groups|Lie-group]] operation.

PoE needs no per-link frames — only the screw axis of each joint and the home pose. Geometrically transparent, easier to teach, no convention ambiguities. The dominant choice in modern textbooks (Lynch & Park, Murray-Li-Sastry). See [[Product_of_Exponentials]].

| | **DH** | **PoE** |
|---|---|---|
| Parameters | 4 per joint | 6 per joint screw + 1 home pose |
| Per-link frames | Required | Not required |
| Closed-form | Yes | Yes |
| Convention ambiguity | Two (classic vs. modified) | None |
| Modern textbooks | Spong, Craig | Lynch & Park, Murray-Li-Sastry |
| Software | KDL, RBDyn, classical Pinocchio | Modern Pinocchio, Drake (`MultibodyPlant`), `manif` |

Both methods produce the *same* $T_{\text{ee}}$. They are different parameterizations, not different physics.

---

## Worked example — 2-DoF planar arm

Two revolute joints, link lengths $L_1$, $L_2$. Joint angles $\theta_1$, $\theta_2$. End-effector position:

$$
x = L_1 \cos\theta_1 + L_2 \cos(\theta_1 + \theta_2)
$$

$$
y = L_1 \sin\theta_1 + L_2 \sin(\theta_1 + \theta_2)
$$

This is FK in its simplest form — a sum of trig terms produced by composing two 2D rotations and translations. Try it with $L_1 = L_2 = 1$, $\theta_1 = 0$, $\theta_2 = 90°$: end-effector lands at $(1, 1)$.

---

## Properties

1. **Always unique.** Given $\boldsymbol{\theta}$, $T_{\text{ee}}$ is determined exactly — no branches, no multiple solutions, no failure modes.
2. **Always closed form.** A finite product of matrix exponentials or trig-and-translation matrices.
3. **Cheap.** $O(n)$ in the number of joints. A 7-DoF arm's FK fits in microseconds on any modern CPU.
4. **Differentiable.** The Jacobian falls out by differentiating the FK map — see [[Jacobian_Matrix]].
5. **Easily codegen'd.** Symbolic FK can be compiled to inlined C/CUDA. Tools like `IKFast` and Pinocchio's autocodegen exploit this.

---

## What FK is used for

- **State estimation** — joint encoders give $\boldsymbol{\theta}$; FK gives the current end-effector pose for visualization, control, and logging.
- **Visualization** — every URDF visualizer ([RViz](https://docs.ros.org/), Drake's `Meshcat`) uses FK to render the robot at each tick.
- **Control** — task-space PD controllers, impedance controllers, and operational-space control all need FK at every cycle.
- **IK initialization** — numerical IK solvers seed their search with FK at a starting configuration.
- **Simulation** — physics engines query FK to find collision-mesh poses each timestep.
- **Calibration** — FK is the model that calibration algorithms refine when correcting for manufacturing errors.

---

## Mobile robots

For wheeled and tracked platforms, "forward kinematics" usually means the **velocity** relation between wheel speeds and chassis motion, not a pose composition.

### Differential drive

For a two-wheel platform with wheel radius $r$ and wheelbase $b$:

$$
v = \frac{r(\dot\phi_R + \dot\phi_L)}{2}, \quad \omega = \frac{r(\dot\phi_R - \dot\phi_L)}{b}
$$

Linear velocity is the average of the two wheel speeds; angular velocity is their difference scaled by wheelbase. See [[Locomotion]] and [[Robot_Gaits]].

### Ackermann (cars)

For a bicycle model with wheelbase $L$ and steering angle $\delta$:

$$
v = r \dot\phi, \quad \omega = \frac{v \tan\delta}{L}
$$

Both are subject to **nonholonomic constraints** — see [[Nonholonomic_Constraint]] and [[Nonholonomic_Systems]].

---

## FK for parallel mechanisms

For a [[Parallel_Mechanisms_and_Robots|parallel manipulator]] (Stewart platform, Delta robot), forward kinematics is *harder* than inverse:

- IK is one independent leg-length computation per leg — closed form.
- FK requires solving a coupled nonlinear system relating leg lengths to platform pose. Often multiple solutions and numerical methods are required.

This is the geometric mirror image of serial chains, where FK is trivial and IK is hard. See [[Parallel_Mechanisms_and_Robots]] and [[Stewart_Platform]].

---

## Practical implementation

```python
# numpy-style FK for a serial chain using PoE
import numpy as np
from scipy.linalg import expm

def screw_to_se3(S):
    """Map a 6-vector screw [omega, v] to a 4x4 se(3) matrix."""
    w, v = S[:3], S[3:]
    W = np.array([[ 0,   -w[2],  w[1]],
                  [ w[2], 0,   -w[0]],
                  [-w[1], w[0], 0   ]])
    out = np.zeros((4, 4))
    out[:3, :3] = W
    out[:3, 3]  = v
    return out

def fk_poe(M, screws, thetas):
    """T_ee = exp([S1] th1) ... exp([Sn] thn) M"""
    T = np.eye(4)
    for S, th in zip(screws, thetas):
        T = T @ expm(screw_to_se3(S) * th)
    return T @ M
```

A production implementation lives in `Pinocchio` (C++/Python), which evaluates FK + Jacobian in microseconds for typical robot models, with autodifferentiable variants for trajectory optimization.

---

## Recommended reading

- Lynch & Park, *Modern Robotics*, Ch. 4 — PoE forward kinematics worked end-to-end
- Spong, Hutchinson, Vidyasagar, *Robot Modeling and Control*, Ch. 3 — DH-style derivation
- Murray, Li, Sastry, *A Mathematical Introduction to Robotic Manipulation*, Ch. 2-3 — screw theory
- Featherstone, *Rigid Body Dynamics Algorithms*, Ch. 3 — algorithmic spatial-vector kinematics
- Pinocchio documentation — production-grade reference implementation

---

## Dataview

```dataview
LIST FROM #kinematics OR #forward-kinematics WHERE contains(file.outlinks, [[Forward_Kinematics]])
```
