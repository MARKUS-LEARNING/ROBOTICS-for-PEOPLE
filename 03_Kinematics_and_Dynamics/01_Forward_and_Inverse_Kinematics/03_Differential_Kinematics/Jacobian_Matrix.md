---
title: Jacobian Matrix
description: The matrix of partial derivatives that maps joint velocities to end-effector velocities and end-effector wrenches to joint torques. The single most-used object in manipulator engineering.
tags:
  - kinematics
  - jacobian
  - manipulator
  - velocity
  - force
  - singularities
  - control
layout: default
category: robotics
author: Jordan_Smith
date: 2025-04-28
permalink: /jacobian_matrix/
related:
  - "[[Kinematics]]"
  - "[[Forward_Kinematics]]"
  - "[[Inverse_Kinematics]]"
  - "[[Singularities]]"
  - "[[Manipulability]]"
  - "[[Calculus_for_Robotics]]"
  - "[[Linear_Algebra_for_Robotics]]"
  - "[[Lie_Groups]]"
  - "[[Torque_and_Force_Calculations]]"
---

# Jacobian Matrix

The **Jacobian matrix** $J(\boldsymbol{\theta})$ is the matrix of first partial derivatives of the [[Forward_Kinematics|forward-kinematics]] map. It is the linear approximation of the FK map at a configuration — the *gradient* of where the gripper is, with respect to where the joints are.

> **Etymology.** Named after **Carl Gustav Jacob Jacobi** (German, 1804–1851), who introduced the matrix of partial derivatives in his 1841 paper *De determinantibus functionalibus* and used it to study transformations of multivariable functions. In robotics the symbol $J$ is universal; in pure math the symbol $D f$ or $\nabla f$ is more common.

For a manipulator with $n$ joints whose end-effector pose is $T_{\text{ee}}(\boldsymbol{\theta}) \in SE(3)$:

$$
J(\boldsymbol{\theta}) = \frac{\partial f}{\partial \boldsymbol{\theta}} = \begin{bmatrix} \dfrac{\partial f}{\partial \theta_1} & \dfrac{\partial f}{\partial \theta_2} & \cdots & \dfrac{\partial f}{\partial \theta_n} \end{bmatrix}
$$

When the output is a 6D twist (3 angular + 3 linear) the Jacobian is $6 \times n$. Each column is the contribution of joint $i$'s motion to the end-effector twist.

---

## The five hats the Jacobian wears

Almost every problem in differential kinematics, statics, control, and IK is a special use of $J$.

### 1. Velocity mapping (forward differential kinematics)

$$
\mathcal{V} = J(\boldsymbol{\theta}) \, \dot{\boldsymbol{\theta}}
$$

Joint velocities $\dot{\boldsymbol{\theta}}$ produce end-effector twist $\mathcal{V} = (\boldsymbol{\omega}, \mathbf{v}) \in \mathbb{R}^6$.

### 2. Inverse-rate control

Solve for joint velocities given a desired end-effector velocity:

$$
\dot{\boldsymbol{\theta}} = J^{-1} \mathcal{V} \quad \text{(square)}, \qquad \dot{\boldsymbol{\theta}} = J^\dagger \mathcal{V} \quad \text{(general)}
$$

This is **resolved-motion-rate control** — the simplest velocity-level Cartesian controller.

### 3. Force / torque duality (statics)

The transpose maps end-effector wrenches to joint torques:

$$
\boldsymbol{\tau} = J^T(\boldsymbol{\theta}) \, \mathcal{F}
$$

where $\mathcal{F} = (\boldsymbol{m}, \boldsymbol{f}) \in \mathbb{R}^6$ is the wrench (moment + force) at the end-effector. This duality is a consequence of the **principle of virtual work** — the same matrix that maps velocities forward must map forces backward.

This identity is the foundation of [[Torque_and_Force_Calculations|force control]], hybrid position/force control, impedance control, and admittance control.

### 4. Numerical inverse kinematics

Newton-style IK iterates:

$$
\boldsymbol{\theta} \leftarrow \boldsymbol{\theta} + J^\dagger(\boldsymbol{\theta}) \, \log(T_{\text{desired}} \cdot T_{\text{current}}^{-1})
$$

The Jacobian is the *only* derivative information available; every numerical IK solver depends on it. See [[Inverse_Kinematics]].

### 5. Manipulability and singularity analysis

The singular values of $J$ encode how isotropically the robot can move:

- **Manipulability:** $w(\boldsymbol{\theta}) = \sqrt{\det(J J^T)}$ — volume of the velocity ellipsoid. Big $w$ = dextrous; $w = 0$ = singular.
- **Singularity:** rank-deficient $J$. See [[Singularities]] and [[Manipulability]].
- **Condition number:** $\sigma_\text{max}(J) / \sigma_\text{min}(J)$ — how anisotropic the manipulator is at this configuration.

---

## Geometric vs. analytical Jacobian

Two flavors, both common, and unfortunately both called "the Jacobian":

### Geometric Jacobian $J_g$

Output is a **twist** $\mathcal{V} = (\boldsymbol{\omega}, \mathbf{v})$ — angular velocity expressed as a 3-vector in the world (or body) frame, and linear velocity of a designated point. Coordinate-free and Lie-theoretic. The dominant choice in modern textbooks (Lynch & Park, Murray-Li-Sastry).

Computed by stacking screw-axis columns: column $i$ is the screw axis of joint $i$ expressed in the chosen frame at the current configuration.

### Analytical Jacobian $J_a$

Output is the time derivative of a *parameterization* of the end-effector pose — e.g., $(x, y, z, \text{roll}, \text{pitch}, \text{yaw})$. Tied to a specific orientation representation.

Related to $J_g$ by:

$$
J_a = T_\phi(\boldsymbol{\phi}) J_g
$$

where $T_\phi$ is the Jacobian of the angular-velocity-to-Euler-rate map (which is singular at gimbal lock — see [[Euler_Angles]]).

**Use the geometric Jacobian** unless you have a specific reason to use the analytical one — it avoids the gimbal-lock singularities of Euler angles.

---

## The screw-axis construction

The cleanest formula: for a serial chain expressed in [[Product_of_Exponentials|Product-of-Exponentials]] form,

$$
T_{\text{ee}}(\boldsymbol{\theta}) = e^{[\mathcal{S}_1] \theta_1} \, e^{[\mathcal{S}_2] \theta_2} \cdots e^{[\mathcal{S}_n] \theta_n} M
$$

the **space Jacobian** $J_s$ has columns

$$
J_{s,i}(\boldsymbol{\theta}) = \text{Ad}_{(e^{[\mathcal{S}_1]\theta_1} \cdots e^{[\mathcal{S}_{i-1}]\theta_{i-1}})} \mathcal{S}_i
$$

where $\text{Ad}_T$ is the [[Lie_Groups|adjoint]] of $T$. Each column is the screw axis of joint $i$ transported into the world frame by the joints upstream of it. The body Jacobian $J_b$ has the same construction in the body frame.

This is the formula used by every modern dynamics library (Pinocchio, Drake, Featherstone-style RBDL).

---

## Worked example — 2-DoF planar arm

End-effector position from FK:
$x = L_1 c_1 + L_2 c_{12}$, $y = L_1 s_1 + L_2 s_{12}$ (with $c_1 = \cos\theta_1$, $c_{12} = \cos(\theta_1 + \theta_2)$, etc.)

Differentiating:

$$
J = \begin{bmatrix} \partial x / \partial \theta_1 & \partial x / \partial \theta_2 \\ \partial y / \partial \theta_1 & \partial y / \partial \theta_2 \end{bmatrix} = \begin{bmatrix} -L_1 s_1 - L_2 s_{12} & -L_2 s_{12} \\ L_1 c_1 + L_2 c_{12} & L_2 c_{12} \end{bmatrix}
$$

Determinant: $\det J = L_1 L_2 \sin\theta_2$. Singular when $\theta_2 = 0$ (arm fully extended) or $\theta_2 = \pm\pi$ (fully folded). Both are workspace-boundary singularities.

---

## Pseudoinverse — when $J$ is not square

Few Jacobians are square. Manipulators have $n \neq 6$, redundant arms have $n > 6$, and even 6-DoF arms become rank-deficient at singularities. The remedy is the **Moore-Penrose pseudoinverse**:

| Case | Pseudoinverse |
|---|---|
| $J$ square, full rank | $J^\dagger = J^{-1}$ |
| $J$ tall ($m > n$, full column rank) | $J^\dagger = (J^T J)^{-1} J^T$ — least-squares |
| $J$ wide ($m < n$, full row rank) | $J^\dagger = J^T (J J^T)^{-1}$ — minimum-norm |
| Rank-deficient | SVD-based pseudoinverse |

For redundant arms ($n > 6$), the minimum-norm solution finds the joint motion of smallest magnitude that achieves the desired end-effector velocity. The null-space term $(I - J^\dagger J)$ is orthogonal to it and can be used for secondary tasks. See [[Inverse_Kinematics]] for redundancy resolution.

---

## Damped pseudoinverse

Near a singularity, $J^\dagger$ blows up. Replace it with the **damped least-squares** version:

$$
J^\dagger_\lambda = J^T (J J^T + \lambda^2 I)^{-1}
$$

Equivalent to solving the regularized least-squares problem $\min \lVert J \dot{\boldsymbol{\theta}} - \mathcal{V} \rVert^2 + \lambda^2 \lVert \dot{\boldsymbol{\theta}} \rVert^2$. As $\lambda \to 0$ this converges to $J^\dagger$; as $\lambda \to \infty$ to zero.

Adaptive schemes set $\lambda^2$ proportional to the smallest singular value of $J$ — large damping near singularities, no damping far from them. This is the **Levenberg-Marquardt** trick applied to differential kinematics. See [[Optimization_for_Robotics]].

---

## Frame transformations

The Jacobian depends on the frame in which the twist is expressed. Two Jacobians for the same robot, expressed in frames $\{A\}$ and $\{B\}$, relate by the [[Lie_Groups|adjoint]]:

$$
J_A = \text{Ad}_{T_{AB}} \, J_B
$$

The most common choices:

- **Space Jacobian** $J_s$ — twist expressed in the world frame. Useful for global tasks.
- **Body Jacobian** $J_b$ — twist expressed in the end-effector body frame. Useful for tool-frame velocity control.

GTSAM, Sophus, manif, and Pinocchio all expose both forms.

---

## Tools

| Library | Jacobian computation |
|---|---|
| **Pinocchio** | `computeJointJacobian`, autodiff variants, $O(n)$ |
| **Drake** | `MultibodyPlant::CalcJacobianSpatialVelocity` |
| **KDL** | `ChainJntToJacSolver` |
| **Peter Corke's `roboticstoolbox`** | `robot.jacob0`, `robot.jacobe` |
| **`numpy`** for hand-derived | autodiff with `jax.jacfwd` or `torch.autograd.functional.jacobian` |

---

## Recommended reading

- Lynch & Park, *Modern Robotics*, Ch. 5 — body and space Jacobians from screws
- Spong, Hutchinson, Vidyasagar, *Robot Modeling and Control*, Ch. 4 — geometric vs. analytical Jacobian
- Murray, Li, Sastry, *A Mathematical Introduction to Robotic Manipulation*, Ch. 3 — adjoint and screw-theoretic treatment
- Sciavicco & Siciliano, *Modelling and Control of Robot Manipulators* — exhaustive worked examples
- Buss, *Introduction to Inverse Kinematics with Jacobian Transpose, Pseudoinverse and Damped Least Squares Methods* (2009 tutorial)

---

## Dataview

```dataview
LIST FROM #kinematics OR #jacobian WHERE contains(file.outlinks, [[Jacobian_Matrix]])
```
