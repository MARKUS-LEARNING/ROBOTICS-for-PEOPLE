---
title: Linear Algebra for Robotics
description: The language in which robotics is written. Vectors, matrices, and linear transformations model every spatial quantity a robot reasons about — positions, velocities, sensor readings, and the operators that move between coordinate frames.
tags:
  - mathematics
  - linear-algebra
  - vectors
  - matrices
  - transformations
  - robotics
  - foundations
layout: default
category: mathematics
author: Jordan_Smith
date: 2025-05-02
permalink: /linear_algebra_for_robotics/
related:
  - "[[Rotation_Matrix]]"
  - "[[Homogeneous_Transformation]]"
  - "[[Quaternions]]"
  - "[[Jacobian_Matrix]]"
  - "[[Lie_Groups]]"
---

# Linear Algebra for Robotics

**Linear algebra** is the branch of mathematics that studies *vectors*, *vector spaces*, *linear maps* between those spaces, and the **matrices** that represent those maps in coordinates. For a robot, nearly every measurable quantity — a joint angle, a position in the world, a pixel in an image, a velocity, an acceleration — lives in some vector space, and nearly every operation a robot performs on those quantities is a linear map (or a local linear approximation of something more complicated).

> **Etymology.** *Linear* comes from Latin *linea* (a line). A linear map is one that preserves the "straight-line" structure of space: it sends lines to lines and the origin to the origin. *Vector* comes from Latin *vehere*, "to carry" — a vector carries you from one point to another. *Matrix* is Latin for "womb" or "source", later meaning a rectangular array in which elements are laid out.

---

## Why linear algebra owns robotics

From first principles, a robot is a machine whose job is to transform information about the world into motion in the world. Both sides of that transformation are naturally expressed in coordinates:

- **Sensing.** An IMU (Inertial Measurement Unit) outputs a 3-vector of accelerations and a 3-vector of angular velocities. A camera outputs a 2D grid of pixel intensities. A LiDAR (Light Detection And Ranging) outputs a 3D point cloud.
- **Internal state.** The pose of a rigid body is a rotation plus a translation — an element of the group SE(3), which is built from 3×3 rotation matrices and 3-vectors.
- **Action.** A joint command is a vector in joint space; a motor torque is another vector; the mapping between joint velocities and end-effector velocities is a matrix (the Jacobian).

The same machinery — vectors, matrices, decompositions — spans all three layers.

---

## Vectors

A **vector** in $\mathbb{R}^n$ is an ordered list of $n$ real numbers:

$$
\mathbf{v} = \begin{bmatrix} v_1 \\ v_2 \\ \vdots \\ v_n \end{bmatrix} \in \mathbb{R}^n
$$

Two operations define a vector space: *addition* ($\mathbf{u} + \mathbf{v}$, component-wise) and *scalar multiplication* ($\alpha \mathbf{v}$). Every other linear-algebraic notion is built from these.

### Key vector operations in robotics

| Operation | Definition | Robotics use |
|---|---|---|
| **Dot product** | $\mathbf{a} \cdot \mathbf{b} = \sum_i a_i b_i$ | Work done by a force; projection of one vector onto another; similarity in feature spaces |
| **Cross product** (3D) | $\mathbf{a} \times \mathbf{b}$ (perpendicular, right-hand rule) | Angular-velocity-to-linear-velocity conversion; torque from force |
| **Norm** | $\lVert \mathbf{v} \rVert_2 = \sqrt{\sum_i v_i^2}$ | Distance in Euclidean space; cost functions |
| **Unit vector** | $\hat{\mathbf{v}} = \mathbf{v} / \lVert \mathbf{v} \rVert$ | Rotation axes; surface normals |
| **Outer product** | $\mathbf{a} \mathbf{b}^T$ (an $m \times n$ matrix) | Rank-one updates in Kalman filters and covariance estimation |

**Geometric intuition for the dot product:** $\mathbf{a} \cdot \mathbf{b} = \lVert \mathbf{a} \rVert \, \lVert \mathbf{b} \rVert \cos\theta$, where $\theta$ is the angle between the vectors. Two vectors are **orthogonal** when their dot product is zero.

---

## Matrices

A **matrix** $A \in \mathbb{R}^{m \times n}$ is a rectangular array that represents a **linear map** $A : \mathbb{R}^n \to \mathbb{R}^m$. The defining property is linearity:

$$
A(\alpha \mathbf{u} + \beta \mathbf{v}) = \alpha A\mathbf{u} + \beta A\mathbf{v}
$$

Matrix multiplication is function composition: if $A$ sends $\mathbb{R}^n \to \mathbb{R}^m$ and $B$ sends $\mathbb{R}^p \to \mathbb{R}^n$, then $AB$ sends $\mathbb{R}^p \to \mathbb{R}^m$.

$$
(AB)_{ij} = \sum_{k=1}^{n} A_{ik} B_{kj}
$$

### Matrices that matter in robotics

- **Rotation matrix** ($R \in SO(3)$): $3 \times 3$, orthogonal ($R^T R = I$), determinant $+1$. Rotates vectors while preserving lengths and angles. See [[Rotation_Matrix]].
- **Homogeneous transformation** ($T \in SE(3)$): $4 \times 4$, combines rotation and translation into a single matrix that composes under multiplication. See [[Homogeneous_Transformation]].
- **Jacobian** ($J(\mathbf{q})$): maps joint velocities $\dot{\mathbf{q}}$ to end-effector twists. Local linear approximation of forward kinematics. See [[Jacobian_Matrix]].
- **Covariance matrix** ($\Sigma$): symmetric positive semi-definite; captures the uncertainty in a Gaussian estimate of state. Central to [[Kalman_Filter]] and [[Extended_Kalman_Filter]].
- **Mass matrix** ($M(\mathbf{q})$): symmetric positive definite; appears in the manipulator equation $M(\mathbf{q}) \ddot{\mathbf{q}} + C(\mathbf{q}, \dot{\mathbf{q}}) \dot{\mathbf{q}} + \mathbf{g}(\mathbf{q}) = \boldsymbol{\tau}$.

---

## Four fundamental subspaces

Every matrix $A \in \mathbb{R}^{m \times n}$ induces four subspaces, which together tell you what $A$ can and cannot do:

| Subspace | Lives in | Meaning |
|---|---|---|
| **Column space** $\mathcal{R}(A)$ | $\mathbb{R}^m$ | Set of all outputs $A\mathbf{x}$ can produce |
| **Null space** $\mathcal{N}(A)$ | $\mathbb{R}^n$ | Inputs $\mathbf{x}$ that $A$ sends to zero |
| **Row space** $\mathcal{R}(A^T)$ | $\mathbb{R}^n$ | Orthogonal complement of the null space |
| **Left null space** $\mathcal{N}(A^T)$ | $\mathbb{R}^m$ | Orthogonal complement of the column space |

**Robotics interpretation of the Jacobian null space:** If $J(\mathbf{q})$ is a $6 \times n$ manipulator Jacobian, then $\dot{\mathbf{q}} \in \mathcal{N}(J)$ means "joint velocities that produce zero end-effector motion" — i.e. **redundant** motions the robot can execute without disturbing the task. A 7-DoF arm exploits a 1-dimensional null space to reconfigure around obstacles while holding the tool fixed.

---

## Matrix decompositions

Decompositions factor a matrix into simpler pieces that expose structure.

### Singular Value Decomposition (SVD)

Every real matrix $A \in \mathbb{R}^{m \times n}$ factors as:

$$
A = U \Sigma V^T
$$

where $U \in \mathbb{R}^{m \times m}$ and $V \in \mathbb{R}^{n \times n}$ are orthogonal and $\Sigma$ is diagonal with non-negative **singular values** $\sigma_1 \geq \sigma_2 \geq \ldots \geq 0$.

SVD is the single most useful decomposition in robotics:

- **Jacobian analysis:** The singular values of $J$ measure manipulability. A singular value near zero signals a **kinematic singularity** (the robot loses a DoF of end-effector motion).
- **Pseudo-inverse:** $J^+ = V \Sigma^+ U^T$ solves the least-squares problem $\min \lVert J\dot{\mathbf{q}} - \mathbf{v}_{\text{ee}} \rVert^2$.
- **Rank-deficient estimation:** SVD provides numerically stable least-squares solutions even when the design matrix is nearly singular.
- **Procrustes alignment:** Given two point clouds, SVD gives the closed-form optimal rotation aligning them.

### Eigen-decomposition

For a symmetric matrix $A \in \mathbb{R}^{n \times n}$:

$$
A = Q \Lambda Q^T
$$

where $Q$ is orthogonal (columns are eigenvectors) and $\Lambda$ is diagonal (eigenvalues). This works cleanly for **covariance matrices** (always symmetric PSD), **mass matrices** (symmetric PD), and stability-analysis matrices.

### QR decomposition

$A = QR$ with $Q$ orthogonal and $R$ upper triangular. The workhorse of least-squares regression: solving $A\mathbf{x} = \mathbf{b}$ becomes $R\mathbf{x} = Q^T\mathbf{b}$, which is fast by back-substitution.

### Cholesky decomposition

For a symmetric positive-definite $A$: $A = L L^T$ with $L$ lower triangular. Used everywhere in Kalman filtering and in solving the normal equations of Gauss-Newton optimization.

---

## Linear transformations in 3D robotics

### Rotations

A **rotation matrix** $R \in SO(3)$ satisfies:

$$
R^T R = I, \quad \det(R) = +1
$$

SO(3) is a 3-dimensional manifold embedded in the 9-dimensional space of $3 \times 3$ matrices — rotations have 3 degrees of freedom. Rotations compose by multiplication and invert by transposition: $R^{-1} = R^T$. Representations include matrices, Euler angles, axis-angle (Rodrigues), and quaternions — see [[Rotation_Matrix]], [[Euler_Angles]], [[Axis_Angle]], [[Quaternions]].

### Rigid-body transformations

A **rigid-body transformation** combines a rotation and a translation. In homogeneous coordinates it is a $4 \times 4$ matrix:

$$
T = \begin{bmatrix} R & \mathbf{t} \\ \mathbf{0}^T & 1 \end{bmatrix} \in SE(3)
$$

Composition is matrix multiplication: $T_{AC} = T_{AB} \, T_{BC}$. Inversion is closed-form:

$$
T^{-1} = \begin{bmatrix} R^T & -R^T \mathbf{t} \\ \mathbf{0}^T & 1 \end{bmatrix}
$$

SE(3) is the configuration space of any free-floating rigid body in 3D — the state space of a drone, a free-flying end-effector, or a camera. See [[Homogeneous_Transformation]] and [[Lie_Groups]].

---

## Solving linear systems

The canonical problem $A\mathbf{x} = \mathbf{b}$ underlies much of robotics numerics:

| Shape | Name | Method |
|---|---|---|
| $A$ square, non-singular | Exact solve | LU decomposition, Cholesky (if SPD) |
| $A$ tall (over-determined) | Least squares | Normal equations $A^T A \mathbf{x} = A^T \mathbf{b}$, QR, or SVD |
| $A$ wide (under-determined) | Minimum-norm solution | Pseudo-inverse $\mathbf{x} = A^T(AA^T)^{-1}\mathbf{b}$ |
| $A$ rank-deficient | Regularized solve | Truncated SVD, ridge regression $(A^T A + \lambda I)^{-1} A^T \mathbf{b}$ |

**Robotics examples:**
- **Forward kinematics to Cartesian velocity:** $\mathbf{v}_{\text{ee}} = J(\mathbf{q}) \dot{\mathbf{q}}$ — direct matrix-vector product.
- **Inverse differential kinematics:** $\dot{\mathbf{q}} = J^+ \mathbf{v}_{\text{ee}}$ — pseudo-inverse. For redundant robots use damped least squares $\dot{\mathbf{q}} = (J^T J + \lambda^2 I)^{-1} J^T \mathbf{v}_{\text{ee}}$ to regularize near singularities.
- **ICP (Iterative Closest Point) alignment:** at each iteration, solve for the rigid transform via SVD of the cross-covariance matrix.
- **Pose graph SLAM:** factor the sparse information matrix via Cholesky to solve the maximum a posteriori (MAP) estimate of all poses.

---

## Numerical hygiene

Floating-point arithmetic introduces error. For robotics code:

1. **Never compute $A^{-1}$ explicitly** when you only need $A^{-1}\mathbf{b}$. Use a solver (`solve(A, b)` in most libraries).
2. **Check orthogonality** of rotation matrices by computing $\lVert R^T R - I \rVert_F$ and re-orthogonalize via SVD or quaternion-renormalization when it drifts.
3. **Prefer Cholesky** over explicit inverse for SPD systems — twice as fast and more stable.
4. **Condition number** $\kappa(A) = \sigma_{\max}/\sigma_{\min}$ predicts error amplification. $\kappa > 10^{12}$ in double precision means you have lost most of your significant digits.
5. **Regularize** near singularities. Naive pseudo-inverses explode; damped least squares bounds the joint velocities.

---

## Applications across the stack

- **State representation:** robot poses ([[Pose_Representation]]), sensor readings, joint configurations
- **Kinematics:** [[Forward_Kinematics]], [[Inverse_Kinematics]], [[Jacobian_Matrix]]
- **Dynamics:** mass-matrix form of the [[Manipulator_Dynamics]] equation
- **Estimation:** [[Kalman_Filter]], [[Extended_Kalman_Filter]], pose-graph [[SLAM]]
- **Perception:** camera projection matrices, essential/fundamental matrices in multi-view geometry
- **Learning:** gradient descent on neural network weights is linear algebra at scale

---

## Recommended references

- Gilbert Strang, *Introduction to Linear Algebra* — the standard accessible textbook
- Sheldon Axler, *Linear Algebra Done Right* — abstract, proof-based; builds strong intuition
- Kevin Lynch & Frank Park, *Modern Robotics*, Chapter 3 — linear algebra specifically for SE(3)
- Gene Golub & Charles Van Loan, *Matrix Computations* — the numerical-analysis bible

---

## Dataview Plugin Features

### List of related concepts

```dataview
LIST FROM #mathematics OR #linear-algebra WHERE contains(file.outlinks, [[Linear_Algebra_for_Robotics]])
```
