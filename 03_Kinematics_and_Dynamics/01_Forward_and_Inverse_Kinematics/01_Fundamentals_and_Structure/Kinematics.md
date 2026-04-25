---
title: Kinematics
description: The geometry of motion. Kinematics describes where things are and how they move, ignoring the forces that cause the motion. The first language a roboticist learns to speak.
tags:
  - robotics
  - kinematics
  - mechanics
  - motion
  - geometry
  - manipulator-arm
type: Robotic Concept
application: Geometry and motion of robotic systems
layout: default
category: robotics
author: Jordan_Smith
date: 2025-04-29
permalink: /kinematics/
related:
  - "[[Kinematics_and_Dynamics]]"
  - "[[Forward_Kinematics]]"
  - "[[Inverse_Kinematics]]"
  - "[[Jacobian_Matrix]]"
  - "[[DH_Parameters]]"
  - "[[Product_of_Exponentials]]"
  - "[[Homogeneous_Transformation]]"
  - "[[Singularities]]"
  - "[[Configuration_Space]]"
  - "[[Lie_Groups]]"
---

# Kinematics

**Kinematics** is the branch of mechanics that describes the *motion* of bodies — their position, velocity, and acceleration — without regard to the forces and torques that produce that motion. Force-free geometry of motion. In robotics it is the foundation underneath every manipulator-arm pose, every mobile-robot trajectory, and every joint-to-end-effector transformation.

> **Etymology.** From the Greek *kinēma* (κίνημα), "movement" or "motion," from *kinein* (κινεῖν), "to move." The same root gives us *cinema* (motion pictures), *kinetic* (relating to motion), and *kinesis* (movement). The term *kinematics* was coined by André-Marie Ampère in 1834 in his *Essai sur la philosophie des sciences* to mark a clean separation from *dynamics* — what moves vs. why it moves.

---

## The split: kinematics vs. dynamics

| | **Kinematics** | **Dynamics** |
|---|---|---|
| Question | Where? How fast? | Why? How much force? |
| Inputs | Joint angles, velocities | Joint torques, masses, inertias |
| Math | Geometry, linear algebra, [[Lie_Groups]] | Newton-Euler, Lagrangian, ODEs |
| First chapter of every robotics text | Yes | Usually after kinematics |

Kinematics treats the robot as a rigid skeleton with hinges. Dynamics adds masses, motors, and gravity. The two together describe the full mechanical reality of the machine.

---

## Why kinematics gets its own chapter

A robot is fundamentally a mapping from *joint space* (angles, slide positions) to *task space* (where the end-effector is). Almost every interesting question reduces to manipulating that mapping:

- **Forward kinematics:** "Given the joint angles, where is the gripper?" — see [[Forward_Kinematics]].
- **Inverse kinematics:** "Given the desired gripper pose, what joint angles get me there?" — see [[Inverse_Kinematics]].
- **Differential kinematics:** "If I move joints at velocity $\dot{\boldsymbol{\theta}}$, how fast does the gripper move?" — answered by the [[Jacobian_Matrix]].
- **Trajectory generation:** A path through joint space or task space that respects the kinematic constraints — see [[Trajectory_Planning]].
- **Singularity analysis:** Where does the mapping break down? — see [[Singularities]].

These five questions are the bread-and-butter of manipulator-arm engineering, and every one of them is a kinematics question, no forces required.

---

## Core building blocks

### Rigid body and pose

A **rigid body** is an idealization: a collection of points whose pairwise distances never change. The configuration of a rigid body in 3D space is its **pose** — six numbers: three for position, three for orientation.

- Position lives in $\mathbb{R}^3$.
- Orientation lives in [[Lie_Groups|SO(3)]] — the group of 3D rotations.
- Pose lives in [[Lie_Groups|SE(3)]] — the group of rigid-body motions.

See [[Pose_Representation]], [[Rotation_Matrix]], [[Quaternions]], and [[Homogeneous_Transformation]] for representations.

### Joint and link

- **Link:** a rigid body. The "bones" of the robot. See [[Links]].
- **Joint:** the connection between two links that permits relative motion in some constrained way. See [[Joints]] and [[Joint_Kinematics]].

The two most common joint types:

| Joint | Symbol | Motion | DoF |
|---|---|---|---|
| **Revolute** (R) | hinge | rotation about an axis | 1 |
| **Prismatic** (P) | slider | translation along an axis | 1 |

Compound joints (cylindrical, spherical, planar) appear too, but most industrial arms are pure-R chains, and most Cartesian gantries are pure-P.

### Degrees of freedom (DoF)

The number of independent parameters needed to specify the robot's configuration. A 6-DoF arm has six independent joints; its configuration space is six-dimensional. Counting DoF is the first sanity check on any mechanism — see [[Degrees_of_Freedom]] and Grübler's formula in [[Independent_Constraints_(DoF)]].

### Kinematic chain

Links connected by joints form a **kinematic chain** — see [[Kinematic_Chains]]. Three flavors:

| Chain | Topology | Example |
|---|---|---|
| **Serial / open** | Linear sequence, base → end-effector | Most industrial arms (UR5, Franka, KUKA iiwa) |
| **Parallel** | Multiple chains share base and platform | Stewart platform, Delta robot |
| **Closed** | At least one closed loop | Four-bar linkage, parallelogram arm |

Serial chains have simple [[Forward_Kinematics]] (just compose joint transforms) but harder [[Inverse_Kinematics]]. Parallel chains flip this: easy IK, harder FK.

---

## The forward kinematics map

For a serial chain with $n$ joints, the **forward kinematics** is the map

$$
T_{\text{ee}} = f(\boldsymbol{\theta}) = T_{0,1}(\theta_1) \, T_{1,2}(\theta_2) \, \cdots \, T_{n-1, n}(\theta_n)
$$

where $\boldsymbol{\theta} = [\theta_1, \ldots, \theta_n]^T$ is the joint vector and $T_{i, i+1}$ is the homogeneous transform from frame $i$ to frame $i+1$. Each $T_{i, i+1}$ depends only on the corresponding joint variable.

Two standard parameterizations:

- **[[DH_Parameters|Denavit-Hartenberg (DH)]] convention** — four parameters per joint $(\theta, d, a, \alpha)$. Compact and historic; the dominant convention in older textbooks and software.
- **[[Product_of_Exponentials]] (PoE)** — uses screw axes and the matrix exponential on $\mathfrak{se}(3)$. Geometrically clean, no per-frame placement choices, increasingly the modern default.

Either way, computing $T_{\text{ee}}$ is just a sequence of $4 \times 4$ matrix multiplies — embarrassingly cheap and **always uniquely defined**.

---

## The inverse kinematics map

The **inverse kinematics** problem asks the inverse question:

$$
\boldsymbol{\theta} = f^{-1}(T_{\text{desired}})
$$

This is genuinely hard because:

1. **Multiple solutions.** A 6-DoF arm typically has 8 or 16 closed-form solutions ("elbow up/down," "wrist flip," etc.).
2. **No solution.** Targets outside the workspace have zero solutions.
3. **Infinite solutions.** Redundant arms ($n > 6$) have a continuum of solutions for a given pose.
4. **Closed form is rare.** Only specific geometries (spherical wrist, three intersecting axes, etc.) admit analytical IK. Most real arms use numerical IK.

See [[Inverse_Kinematics]], [[Analytical_IK_Solutions]], and [[Numerical_IK_Solutions]].

---

## Differential (velocity) kinematics

Differentiating forward kinematics gives the velocity relationship:

$$
\dot{\mathbf{x}} = J(\boldsymbol{\theta}) \, \dot{\boldsymbol{\theta}}
$$

where $J$ is the [[Jacobian_Matrix]] — an $m \times n$ matrix whose columns are the partial derivatives of the end-effector pose with respect to each joint.

The Jacobian is the single most-used object in manipulator engineering:

- **Velocity mapping:** joint velocities → end-effector twist (linear + angular velocity).
- **Force mapping:** end-effector wrench → joint torques (via $\boldsymbol{\tau} = J^T \mathbf{F}$).
- **Singularity detection:** $\det J = 0$ (square case) or rank-deficient $J$ (general case) signals a [[Singularities|singular]] configuration.
- **Manipulability:** $\sqrt{\det(J J^T)}$ measures how isotropically the robot can move at a configuration — see [[Manipulability]].
- **Numerical IK:** Newton, Gauss-Newton, and Levenberg-Marquardt iterations all use $J$ to update joint guesses.

---

## Configuration space

Each robot has a **configuration space** $\mathcal{C}$ — the manifold of all geometrically possible joint configurations.

| Robot | $\mathcal{C}$ |
|---|---|
| 6-R arm (no limits) | $S^1 \times \cdots \times S^1$ (a 6-torus) |
| 6-R arm with joint limits | A subset of $\mathbb{R}^6$ |
| Mobile robot in plane | $SE(2) = \mathbb{R}^2 \times S^1$ |
| Free-flying drone | $SE(3) = \mathbb{R}^3 \times SO(3)$ |
| Wheeled car (nonholonomic) | $SE(2)$ with constraints |

[[Configuration_Space]] is where motion planning lives — paths and obstacles are reasoned about in $\mathcal{C}$, not in workspace.

---

## Workspace

The set of all end-effector poses the robot can reach. See [[Workspace]] and [[Task_Space_and_Workspace]].

- **Reachable workspace:** the set of points the end-effector can touch.
- **Dexterous workspace:** the subset where it can touch a point in *every* orientation.

Workspace shape is a function of link lengths and joint limits. A serial 6-R arm typically has a roughly toroidal reachable workspace.

---

## Singularities

Configurations where the Jacobian loses rank — the robot loses one or more directions of instantaneous end-effector motion. Three classic types:

1. **Boundary singularity:** end-effector at the workspace boundary, arm fully extended.
2. **Interior singularity:** two joint axes become coincident or coplanar (e.g., wrist flip).
3. **Algorithmic singularity:** specific to redundancy resolution schemes.

Near a singularity, infinite joint velocities are needed to produce small Cartesian motions in the lost direction. Practical IK solvers detect this via the smallest singular value of $J$ and apply damping (Levenberg-Marquardt) or move along the null space. See [[Singularities]] and [[Manipulability]].

---

## Mobile-robot kinematics

Manipulators are not the only robots with kinematics. Mobile platforms have their own:

| Drive | DoF (configuration) | DoF (control) | Constraints |
|---|---|---|---|
| **Differential drive** (TurtleBot, Roomba) | 3 ($x, y, \theta$) | 2 (left/right wheel) | Nonholonomic — can't slide sideways |
| **Ackermann steering** (cars) | 3 | 2 (steering, throttle) | Nonholonomic |
| **Omnidirectional / Mecanum** | 3 | 3 | Holonomic — can move in any direction |
| **Quadrotor** | 6 ($SE(3)$) | 4 (rotor speeds) | Underactuated |

A **nonholonomic** constraint reduces the directions in which the robot can instantaneously move without reducing its configuration-space dimensionality. A car can park in any pose (3 DoF of configuration) but can't strafe sideways (only 2 DoF of instantaneous motion). See [[Nonholonomic_Constraint]] and [[Nonholonomic_Systems]].

---

## Why this is hard to learn the first time

Three sticky concepts:

1. **Frames everywhere.** Every link, every camera, every fixture has a coordinate frame. Kinematics is bookkeeping of "what is this number expressed in?" Switching frames means multiplying by transforms; getting it wrong is the most common source of bugs.
2. **SO(3) is not Euclidean.** You cannot average rotation matrices coordinate-wise, you cannot subtract them, you cannot interpolate them linearly. The right tools are [[Quaternions|quaternions]], [[Lie_Groups|Lie groups]], and [[Rotation_Matrix|rotation matrices]] — not Euler-angle subtraction. See [[Euler_Angles]] for why.
3. **Forward is easy, inverse is hard.** Don't trust this until you've watched a 6-DoF Newton solver bounce out of basin or a closed-form derivation produce eight branch sign-choices.

---

## Practical tooling

| Tool | What it provides |
|---|---|
| **ROS 2 `tf2`** | Distributed coordinate-frame bookkeeping across a robot's nodes |
| **Python: `numpy`, `scipy.spatial.transform`** | Rotations, quaternions, basic transforms |
| **`Pinocchio` (C++/Python)** | Fast spatial-algebra kinematics + dynamics; used in Crocoddyl, Stack of Tasks |
| **`KDL` / `orocos_kdl`** | Classic ROS kinematics library |
| **`Drake`** | Modern systems framework with full kinematic + dynamic models |
| **Peter Corke's `Robotics Toolbox` (Python/MATLAB)** | Pedagogical-strength SerialLink, ETS, mobile-robot models |
| **`MoveIt 2`** | Motion-planning + IK + collision-aware execution stack |
| **OpenRAVE / KDL-IK / IKFast** | Closed-form IK code-generators |

---

## Recommended reading

- Lynch & Park, *Modern Robotics: Mechanics, Planning, and Control* (free online) — the modern PoE-first textbook
- Spong, Hutchinson, Vidyasagar, *Robot Modeling and Control* — DH-first classic
- Craig, *Introduction to Robotics: Mechanics and Control* — staple undergraduate text
- Murray, Li, Sastry, *A Mathematical Introduction to Robotic Manipulation* (free online) — screw theory and Lie groups
- Siciliano et al., *Robotics: Modelling, Planning and Control* — comprehensive reference

---

## Dataview

```dataview
LIST FROM #kinematics OR #robotics WHERE contains(file.outlinks, [[Kinematics]])
```
