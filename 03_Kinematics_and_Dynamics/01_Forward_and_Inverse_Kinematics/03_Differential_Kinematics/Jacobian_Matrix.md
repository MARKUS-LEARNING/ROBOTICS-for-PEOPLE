---
title: Jacobian Matrix
description: Explains the Jacobian matrix in robotics, its role in relating joint velocities to end-effector velocities and joint torques to end-effector forces, and its connection to singularities.
tags:
  - kinematics
  - dynamics
  - manipulator
  - velocity
  - force
  - singularities
  - control-theory
layout: default
category: robotics
author: Jordan_Smith_&_le_Chat
date: 2025-04-28
permalink: /jacobian_matrix/
related:
  - "[[Kinematics]]"
  - "[[Forward_Kinematics]]"
  - "[[Inverse_Kinematics]]"
  - "[[Velocity]]"
  - "[[Statics]]"
  - "[[Torque_and_Force_Calculations]]"
  - "[[Singularities]]"
  - "[[Manipulability]]"
  - "[[Resolved Motion Rate Control]]"
  - "[[Degrees_of_Freedom]]"
---

# Jacobian Matrix

The **Jacobian matrix** (often simply called the **Jacobian**) in robotics, denoted as $J(\mathbf{q})$, is a fundamental tool in [[Kinematics]] and [[Dynamics]]. It is a configuration-dependent matrix that relates the velocities in [[Joints|joint space]] to the velocities in [[Cartesian Space|Cartesian space]] (also called task space or operational space).

Specifically, it maps the vector of joint velocities $\dot{\mathbf{q}}$ to the end-effector's spatial velocity (twist), $\mathcal{V}$, expressed in a chosen reference frame {k}:

$$
^k \mathcal{V} = {}^k J(\mathbf{q}) \dot{\mathbf{q}}
$$

The twist $\mathcal{V}$ is a 6x1 vector combining the end-effector's angular velocity $\omega$ and linear velocity $\mathbf{v}$: $^k \mathcal{V} = [\omega^T_k \, \mathbf{v}^T_k]^T$. For a manipulator with $n$ degrees of freedom, the Jacobian $J(\mathbf{q})$ is typically a $6 \times n$ matrix. Each column $J_i$ of the Jacobian represents the contribution of the i-th joint's velocity ($\dot{q}_i$) to the end-effector's twist.

Mathematically, the Jacobian is the matrix of partial derivatives of the [[Forward_Kinematics]] mapping function (relating joint variables to end-effector pose) with respect to the joint variables.

---

## Duality: Velocity and Static Forces

The Jacobian matrix exhibits an important duality between velocity mapping and static force mapping, stemming from the principle of virtual work.

* **Velocity Mapping:** As defined above, $^k \mathcal{V} = {}^k J(\mathbf{q}) \dot{\mathbf{q}}$. It transforms joint velocities into end-effector Cartesian velocities.
* **Static Force Mapping:** The transpose of the Jacobian, $J^T(\mathbf{q})$, maps a static wrench $\mathcal{F}$ (a 6x1 vector combining forces $\mathbf{f}$ and moments $\mathbf{m}$ applied at the end-effector, expressed in frame {k}) to the vector of joint torques/forces $\tau$ required to statically balance that wrench:
    $$
    \tau = J^T(\mathbf{q}) \, ^k \mathcal{F}
    $$
    where $^k \mathcal{F} = [\mathbf{m}^T_k \, \mathbf{f}^T_k]^T$. This relationship is fundamental for [[Torque_and_Force_Calculations|Force Control]].

---

## Singularities

A critical concept associated with the Jacobian is that of [[Singularities]]. A singularity occurs at a specific joint configuration $\mathbf{q}$ where the Jacobian matrix $J(\mathbf{q})$ loses rank (its columns or rows become linearly dependent).

* **Consequences:** At a singularity:
    * The manipulator loses one or more [[Degrees_of_Freedom]] in Cartesian space; certain end-effector motions become impossible, regardless of joint velocities.
    * The [[Inverse_Kinematics]] problem becomes ill-posed. Solving $\dot{\mathbf{q}} = J^{-1}(\mathbf{q}) \mathcal{V}$ (for square, non-redundant Jacobians) is impossible as $J^{-1}$ does not exist, or it would require infinite joint velocities to produce finite end-effector velocities in certain directions.
    * The manipulator cannot resist arbitrary external wrenches; certain forces/moments can be applied to the end-effector without requiring any opposing joint torque.
* **Types:** Singularities typically occur at the boundary of the [[Workspace]] (e.g., arm fully stretched or folded) and may also occur inside the workspace (often due to alignment of multiple joint axes, e.g., wrist singularity).
* **Detection:** Singularities are identified by checking the rank of $J(\mathbf{q})$. For square Jacobians ($n=6$), this is equivalent to checking if $\det(J(\mathbf{q})) = 0$. For non-square Jacobians, rank determination via techniques like Singular Value Decomposition (SVD) is needed. Measures like [[Manipulability]] or the minimum singular value quantify the "distance" to a singularity.

---

## Calculation

The Jacobian matrix can be calculated using several methods:

* **Velocity Propagation:** This iterative method calculates the linear and angular velocities of each link frame, starting from the base and moving outwards. The velocity contribution of each joint to the end-effector twist forms the corresponding column of the Jacobian. Spatial vector algebra provides a concise formulation for this.
* **Direct Differentiation:** The forward kinematic equations $\mathbf{x}_{EE} = f(\mathbf{q})$ are analytically differentiated with respect to each joint variable $q_i$. The resulting partial derivative vectors $\partial f / \partial q_i$ form the columns of the Jacobian's linear velocity portion. Calculating the angular velocity portion requires careful differentiation of the orientation representation.

---

## Frame Transformation

The Jacobian is frame-dependent. If the Jacobian $^B J$ is known in frame {B}, it can be expressed in frame {A} using the rotation matrix $^A R_B$ relating the two frames:
$$
^A J = \begin{pmatrix} ^A R_B & 0 \\ 0 & ^A R_B \end{pmatrix} ^B J
$$
(This applies to the geometric Jacobian relating joint rates to end-effector twist). Alternatively, spatial transformation matrices ($^A X_B$) can be used in the spatial vector formalism.

---

## Applications

The Jacobian matrix is a central tool in many areas of robotics:

* **[[Inverse_Kinematics]]:** Numerical methods for solving IK heavily rely on the Jacobian (or its inverse/pseudoinverse) to iteratively compute joint corrections from Cartesian errors.
* **Velocity Control:** Maps desired Cartesian velocities to required joint velocities (e.g., [[Resolved Motion Rate Control]]).
* **Statics & Force Control:** Relates end-effector forces to joint torques, enabling force analysis and control.
* **[[Manipulability]] Analysis:** The properties of the Jacobian (e.g., singular values, condition number) quantify the robot's dexterity and closeness to singularities.
* **[[Trajectory Tracking]]:** Used in some advanced control schemes.

---

```dataview
LIST FROM #robotics OR #kinematics WHERE contains(file.outlinks, [[Jacobian_Matrix]])