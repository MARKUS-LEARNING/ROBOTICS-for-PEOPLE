---
title: Degrees of Freedom (DoF)
description: Defines Degrees of Freedom (DoF) as the number of independent parameters required to specify the configuration of a mechanism or body.
tags:
  - kinematics
  - robotics
  - configuration-space
  - mobility
  - robot-design
  - manipulator-arm
  - mobile-robot
  - legged-robot
  - parallel-robot
layout: default
category: robotics
author: Jordan_Smith
date: 2025-04-27
permalink: /degrees_of_freedom/
related:
  - "[[Joints_(DoF)]]"
  - "[[Links_(DoF)]]"
  - "[[Grübler's_Formula]]"
  - "[[Configuration_Space]]"
  - "[[Workspace_Analysis]]"
---

# Degrees of Freedom (DoF)

## Intuition: What Are Degrees of Freedom?

Hold a book in the air. You can slide it left/right, forward/back, and up/down — that is 3 translational freedoms. You can also tilt it (pitch), turn it (yaw), and roll it — that is 3 rotational freedoms. Together, the book has **6 degrees of freedom** in 3D space.

Now set the book flat on a table. It can still slide in two directions and spin, but it cannot lift off or tilt — the table constrains it to **3 DoF**. Every joint in a robot works this way: it *allows* some motions and *constrains* others. Counting the net freedoms tells you how many independent motors (or inputs) you need to fully control the robot, and what kinds of tasks it can perform.

---

**Degrees of Freedom (DoF)** refers to the minimum number of independent parameters required to completely specify the configuration (position and orientation) of a mechanism or a body in space. It quantifies the independent ways a system can move, providing insight into its mobility and control complexity. Understanding DoF is crucial for designing robots that can effectively interact with their environment.

---

## Degrees of Freedom (DoF) Calculation

The Degrees of Freedom (DoF) of a mechanical or robotic system can be calculated using the following formulas:

$$
\text{DoF} = \sum_{i=1}^{n} f_i - c
$$

where $f_i$ is the number of freedoms of the $i$-th body (or point) and $c$ is the number of independent constraints. Equivalently, for a mechanism with $n$ rigid bodies and $j$ joints:

$$
\text{DoF} = \sum_{i=1}^{n} (\text{freedoms of body } i) - \sum_{k=1}^{c} (\text{constraint } k)
$$

> **Practitioner's note:** In practice, you count 6 freedoms per body in 3D (or 3 per body in 2D), then subtract the constraints imposed by each joint. See [[Grübler's_Formula]] for the full worked-out form.

### Grübler-Kutzbach Criterion (Worked Example)

The general mobility formula for a spatial mechanism with $N$ links (including the fixed ground link), $J$ joints, and $f_i$ freedoms per joint is:

$$
M = 6(N - 1 - J) + \sum_{i=1}^{J} f_i
$$

**Example: KUKA KR6 R900 (6R serial manipulator)**

- $N = 7$ links (6 moving links + 1 fixed base)
- $J = 6$ revolute joints, each with $f_i = 1$

$$
M = 6(7 - 1 - 6) + 6(1) = 6(0) + 6 = 6 \text{ DoF}
$$

The robot can position and orient its end-effector arbitrarily in 3D — exactly what is needed for general-purpose manipulation.

**Example: Franka Emika Panda (7R redundant manipulator)**

$$
M = 6(8 - 1 - 7) + 7(1) = 0 + 7 = 7 \text{ DoF}
$$

The 7th DoF provides a 1-dimensional *self-motion manifold*: the elbow can swing while the end-effector stays fixed, enabling [[Singularities|singularity avoidance]] and obstacle clearance.

**Example: Stewart platform (6-UPS parallel robot)**

- $N = 14$ (1 base + 6 legs × 2 links each + 1 platform)
- $J = 18$ (6 universal + 6 prismatic + 6 spherical), with $f_i = 2 + 1 + 3 = 6$ per leg

$$
M = 6(14 - 1 - 18) + 6(2) + 6(1) + 6(3) = 6(-5) + 12 + 6 + 18 = -30 + 36 = 6 \text{ DoF}
$$

The platform can translate in $x, y, z$ and rotate in roll, pitch, yaw — used in flight simulators and precision machining.

---
## DoF of Rigid Bodies

- **In 3D Space**: A free [[Rigid_Body|rigid body]] has 6 DoF: 3 translational (e.g., $x$, $y$, $z$) and 3 rotational (e.g., roll $\phi$, pitch $\theta$, yaw $\psi$). These degrees allow the body to move freely in space and orient itself in any direction.
  <br>

- **In 2D Plane**: A free rigid body moving on a plane has 3 DoF: 2 translational ($x$, $y$ position) and 1 rotational (orientation $\theta$). This configuration is common in planar robots and mechanisms constrained to two-dimensional motion.
  <br>

---

## DoF of Mechanisms

The DoF of a mechanism is the number of independent inputs needed to determine the configuration of all its parts. This concept is essential for analyzing and designing robotic systems:

- **Joint Contribution**: Joints connect links and constrain their relative motion. Each joint allows a certain number of DoF:
  - **Revolute/Prismatic Joints**: 1 DoF
  - **Cylindrical/Universal Joints**: 2 DoF
  - **Spherical/Planar Joints**: 3 DoF
  - A connection with no constraints can be modeled as a 6-DoF joint.
  <br>

- **Serial Manipulators**: For typical open [[Kinematic_Chains|kinematic chain]] manipulators, the total DoF is usually the sum of the DoF of its joints:
  - A general-purpose spatial manipulator typically requires 6 DoF to arbitrarily position and orient its end-effector in 3D space.
  - Manipulators with more than 6 DoF are called **kinematically redundant**. Redundancy increases dexterity, potentially allowing for [[Singularities]] and obstacle avoidance.
  - Manipulators with fewer than 6 DoF have restricted motion capabilities and operate within a lower-dimensional subspace of the full 6D space.
  - Tasks requiring less than 6 DoF (e.g., using symmetric tools) make a 6-DoF robot task-redundant.
  - Kinematic singularities are configurations where the manipulator locally loses one or more DoF in terms of end-effector motion.
  <br>

- **Mobile Robots (Planar)**:
  - The chassis of a mobile robot moving on a plane typically has 3 DoF ($x$, $y$, $\theta$). This refers to the chassis pose.
  - The wheels impose kinematic constraints (holonomic or nonholonomic) that affect the **differential degrees of freedom (DDOF)**, which is the number of independent velocity inputs the robot can control instantaneously.
  - An **omnidirectional** robot has 3 DDOF (its velocity in $x$, $y$, and $\theta$ can be controlled independently). A differential drive robot has 2 DDOF.
  - The **maneuverability** ($\delta M$) considers both mobility ($\delta m = \text{DDOF}$) and steerability ($\delta s$).
  <br>

- **Legged Robots**: DoF is often described by the number of actuated joints per leg (typically 2-3), plus the DoF of the main body (often 6 if considered a floating base).
  <br>

- **Parallel Robots**: These have closed kinematic loops. Their DoF is typically calculated using formulas like the Grübler-Kutzbach criterion, which considers the number of links, joints, and joint types. The DoF can potentially change depending on the configuration (posture).
  <br>

---

## Relevance in Robotics

- **Task Requirements**: The necessary DoF for a robot depends on the task. Simple tasks might require fewer than 6 DoF (e.g., vertical assembly). Using symmetric tools can also reduce the required DoF, making a 6-DoF robot task-redundant.
  <br>

- **Workspace**: The DoF defines the dimensionality of the robot's configuration space. The [[Workspace]] (reachable and dexterous) is the set of poses the robot can achieve within this space, limited by link geometry and joint limits.
  <br>

- **Singularities**: At kinematic [[Singularities]], a manipulator loses one or more DoF in its ability to move the end-effector in Cartesian space, regardless of joint velocities. Redundancy can help mitigate this.
  <br>

- **Control**: The DoF determines the number of independent variables that need to be controlled. [[Motion_Control|Control strategies]] often differ based on whether the system is fully actuated (number of [[Actuator|actuators]] = DoF) or [[Open_Loop_vs_Closed_Loop|underactuated]].
  <br>

The concept of DoF is crucial for understanding a robot's mobility, dexterity, and control complexity. It plays a fundamental role in the design and analysis of robotic systems, influencing their ability to perform tasks and interact with their environment.

---

## Over-Constrained and Under-Constrained Mechanisms

Not all mechanisms follow the "expected" DoF from Grubler's formula. Understanding these exceptions is critical for practitioners:

### Over-Constrained Mechanisms

An **over-constrained** mechanism has more constraints than predicted by the generic mobility formula, yet it still moves due to special geometric conditions (e.g., parallel axes, coplanar joints). The formula predicts $M \leq 0$, but the mechanism has $M > 0$ in practice.

**Examples:**
- **Bennett linkage**: A spatial 4-bar linkage with 4 revolute joints. Grubler predicts $M = 6(4-1-4) + 4(1) = -2$, yet it has $M = 1$ DoF due to specific link length and twist angle relationships.
- **Stewart-Gough platform**: Six legs, each with a universal-prismatic-spherical (UPS) chain. The naive formula gives $M = 6(14-1-18) + 6(2+1+3) = 6$. The actual platform has 6 DoF, but only because of specific geometric alignment of the joint axes.
- **Planar four-bar linkage analyzed in 3D**: Grubler's spatial formula gives $M = 6(4-1-4) + 4(1) = -2$, but the mechanism works because all joints are parallel (a planar sub-group). Always apply the formula in the correct subspace.

### Under-Constrained Mechanisms

An **under-constrained** (or **under-actuated**) mechanism has more DoF than actuators. The unactuated DoF must be stabilized through dynamic control or passive elements.

**Examples:**
- **Acrobot**: A 2-link pendulum with only 1 actuator at the elbow. It has 2 DoF but only 1 actuator, requiring swing-up control algorithms.
- **Quadrotor**: 6 DoF (position + orientation) but only 4 [[Actuator|actuators]] (rotors). Roll and pitch are controlled indirectly through differential thrust.
- **Cart-pole (inverted pendulum)**: 2 DoF, 1 actuator. The classic underactuated control benchmark.

> **Practitioner's tip:** If Grubler's formula gives a negative or zero DoF but the mechanism clearly moves, check for special geometry (parallel axes, intersecting axes, symmetric link lengths). These create "passive constraints" that are not truly independent.

---

## Workspace Constraints and Practical Considerations

The DoF of a robot determines the dimensionality of its [[Workspace|workspace]], but practical factors further restrict the achievable workspace:

### Joint Limits

Every real joint has physical limits. A revolute joint rarely achieves a full $360°$ range due to cable routing, mechanical stops, and interference:
- **Typical industrial revolute joint range**: $\pm 170°$ to $\pm 350°$ depending on axis
- **Typical prismatic joint stroke**: 0.1 m to 2.0 m depending on actuator type

### Workspace Volume

For a serial manipulator with $n$ revolute joints, each with range $[\theta_{i,\min}, \theta_{i,\max}]$, the configuration space is an $n$-dimensional hyper-rectangle. The reachable workspace in Cartesian space is the image of this set under the forward kinematics map:

$$
\mathcal{W} = \{ \mathbf{p} \in \mathbb{R}^3 : \mathbf{p} = f(\mathbf{q}), \; \mathbf{q} \in \mathcal{Q} \}
$$

where $f(\mathbf{q})$ is the forward kinematics function and $\mathcal{Q}$ is the set of all feasible joint configurations.

### Dexterous Workspace

The **dexterous workspace** is the subset of the reachable workspace where the end-effector can achieve any arbitrary orientation:

$$
\mathcal{W}_d = \{ \mathbf{p} \in \mathcal{W} : \forall R \in SO(3), \; \exists \mathbf{q} \in \mathcal{Q} \text{ s.t. } f(\mathbf{q}) = (\mathbf{p}, R) \}
$$

In practice, the dexterous workspace is always a subset (often much smaller) of the reachable workspace.

---

## Comparison of DoF for Common Robot Types

| Robot Type | Typical DoF | Actuated DoF | Workspace Shape | Example Robots |
|---|---|---|---|---|
| **SCARA** | 4 (3R + 1P) | 4 | Cylindrical annulus | Epson T6, Fanuc SR-3iA |
| **6-Axis Serial** | 6 (6R) | 6 | Irregular sphere | KUKA KR6, ABB IRB 6700, UR5e |
| **7-Axis Redundant** | 7 (7R) | 7 | Irregular sphere (with self-motion) | KUKA LBR iiwa, Franka Emika Panda |
| **Delta / Parallel** | 3 or 6 | 3 or 6 | Cylindrical / dome | ABB IRB 360 FlexPicker |
| **Cartesian / Gantry** | 3 (3P) | 3 | Rectangular prism | Güdel gantry systems |
| **Differential Drive** | 3 (chassis) | 2 | Plane (with constraints) | iRobot Roomba, TurtleBot |
| **Omnidirectional Mobile** | 3 (chassis) | 3 | Plane (unconstrained) | KUKA youBot base, Mecanum platforms |
| **Mobile Manipulator** | 9+ (3 base + 6+ arm) | 8+ | Full room-scale 6D | Fetch Mobile Manipulator, KUKA KMR |
| **Humanoid** | 30-50+ | 30-50+ | Complex, multi-limb | Boston Dynamics Atlas (28 DoF), Honda ASIMO (57 DoF) |
| **Quadruped** | 12-16 (3-4 per leg) | 12-16 | Terrain-dependent | Boston Dynamics Spot (12), Unitree Go2 (12) |

> **Practitioner's note:** A common mistake is confusing *configuration-space DoF* (number of joint variables) with *task-space DoF* (degrees of freedom of the end-effector pose). A 6R robot has 6 configuration-space DoF but can only control up to 6 task-space DoF ($x, y, z, \phi, \theta, \psi$). A 7R redundant robot still has only 6 task-space DoF -- the extra joint provides a 1D self-motion manifold used for obstacle avoidance or singularity avoidance.

---

## Configuration Space and DoF

The DoF of a robot defines the dimension of its **configuration space** $\mathcal{C}$. Each point in $\mathcal{C}$ is a vector of all joint variables $\mathbf{q} = (q_1, q_2, \ldots, q_n)$ that fully specifies the robot's pose.

- A 6R manipulator lives in $\mathcal{C} = \mathbb{T}^6$ (a 6-dimensional torus, since each revolute joint wraps around).
- A mobile manipulator (3 DoF base + 6 DoF arm) lives in $\mathcal{C} = SE(2) \times \mathbb{T}^6$, a 9-dimensional space.

**Why this matters:** [[Graph_Theory|Path planning]] algorithms ([[A*_Algorithm|A*]], [[Dijkstra's_Algorithm|Dijkstra's]], RRT, PRM) search through $\mathcal{C}$ to find collision-free trajectories. The curse of dimensionality means that grid-based planners become impractical above ~4 DoF, which is why sampling-based planners (RRT, PRM) are standard for manipulator arms. [[Optimization|Trajectory optimization]] methods (direct collocation, shooting) also operate in $\mathcal{C}$ and scale better to high-DoF systems like humanoids (30+ DoF).

Understanding a robot's DoF is therefore the first step in choosing the right planning and [[AI_and_Robot_Control|control]] strategy.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

- **List all related kinematics and robotics concepts**:
  ```dataview
  LIST FROM #kinematics OR #robot-design WHERE contains(file.outlinks, [[Degrees_of_Freedom]])
  ```
