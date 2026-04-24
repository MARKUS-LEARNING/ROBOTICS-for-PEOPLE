---
title: Robotics
description: Robotics is an interdisciplinary field that integrates computer science and engineering to design, construct, operate, and use robots for various applications, enhancing automation, efficiency, and human capabilities.
tags:
  - technology
  - automation
  - engineering
  - AI
  - control-systems
  - mechatronics
  - sensors
  - actuators
  - autonomous-systems
layout: default
category: robotics
author: Jordan_Smith
date: 2026-02-14
permalink: /robotics/
related:
  - "[[Robots]]"
  - "[[History_of_Robotics]]"
  - "[[Control_Systems]]"
  - "[[Mechatronics]]"
  - "[[Human-Robot_Interaction]]"
---

# Robotics

**Robotics** is an interdisciplinary field that integrates computer science and engineering to design, construct, operate, and use robots for various applications. It encompasses the development of intelligent machines capable of performing tasks autonomously or semi-autonomously, enhancing automation, efficiency, and human capabilities. Robotics draws from various disciplines, including mechanics, electronics, computer science, and artificial intelligence, to create systems that can interact with the physical world.

---
<img src="https://resources.news.e.abb.com/images/2023/3/15/0/ABB_Robotics_Mega_Factory_Opening_AI-powered_robotic_systems_take_on_tasks_such_as_screwdriving.jpg"></img>
<font size=1>*source: https://new.abb.com/news/detail/100845/abb-to-expand-robotics-factory-in-us*</font>
---

## Key Components of Robotics

1. **Mechanical Design**: The physical structure of a robot — its [[Links_(DoF)|links]], [[Joints_(DoF)|joints]], and [[Actuator|actuators]] — determines what it can physically do. A 6-axis industrial arm like the FANUC M-20iA has a 1,811 mm reach and 35 kg payload capacity; those numbers come directly from link lengths, joint ranges, and structural stiffness.

2. **[[Sensors]]**: Robots perceive the world through sensors — [[Camera_Systems|cameras]], [[LIDAR]], [[IMU_Sensors|IMUs]], force/torque sensors, encoders. A joint encoder on a servo motor typically provides 4,096–262,144 counts per revolution, giving position resolution on the order of $0.001°$. Sensor fusion combines these modalities into a coherent state estimate.

3. **[[Actuator|Actuators]]**: Convert energy into motion. Electric servo motors dominate industrial robotics due to their controllability — a typical servo provides continuous torque of 1–50 Nm with peak torques 3–5x higher. Hydraulic actuators deliver higher force density (up to $20 \text{ MPa}$ working pressure) for heavy-duty applications like excavators and legged robots.

4. **[[Control_Systems|Control Systems]]**: The algorithms that close the loop between sensing and actuation. In practice, most industrial robots run cascaded control: an outer position loop at 100–500 Hz commands an inner current/torque loop running at 5–20 kHz.

5. **[[Artificial_Intelligence|Artificial Intelligence]]**: Enables robots to handle uncertainty and adapt. Classical AI provides path planning (e.g., [[Path_Planning|A*, RRT]]); modern deep learning enables vision-based grasping, [[Reinforcement_Learning_for_Robots|reinforcement learning]] for locomotion, and [[Foundation_Models|foundation models]] for task planning from natural language.

6. **[[Human-Robot_Interaction|Human-Robot Interaction]]**: [[Collaborative_Robots|Cobots]] like the Universal Robots UR5e limit contact forces to < 150 N (per ISO/TS 15066) via force-limited joints and power monitoring, enabling cage-free operation alongside humans.

---

## Mathematical Foundations

The three pillars of robotics mathematics are kinematics, dynamics, and control. Each builds on the previous:

### [[Kinematics]] — Where Is the Robot?

For a serial manipulator with $n$ joints, the end-effector pose is computed by chaining homogeneous transformation matrices:

$$
T_0^n = T_0^1 \cdot T_1^2 \cdot \ldots \cdot T_{n-1}^n
$$

Each $T_{i-1}^i \in SE(3)$ is a $4 \times 4$ matrix encoding both rotation and translation:

$$
T_{i-1}^i = \begin{bmatrix} R_{i-1}^i & d_{i-1}^i \\ 0 & 1 \end{bmatrix}
$$

where $R \in SO(3)$ is the $3 \times 3$ rotation matrix and $d \in \mathbb{R}^3$ is the translation vector. Using the [[Links_and_Joints_Definitions|Denavit-Hartenberg convention]], each $T_{i-1}^i$ is parameterized by four quantities: $\theta_i$ (joint angle), $d_i$ (link offset), $a_i$ (link length), and $\alpha_i$ (link twist).

**Practitioner note**: For a 6-DOF arm, forward kinematics is computationally cheap (~microseconds). Inverse kinematics — finding joint angles $q$ for a desired pose $T_{\text{desired}}$ — is the harder problem, often requiring numerical methods like the Jacobian pseudoinverse:

$$
\Delta q = J^+(q) \cdot \Delta x, \quad \text{where } J^+ = J^T(JJ^T)^{-1}
$$

### [[Dynamics]] — What Forces Are Needed?

The manipulator equation of motion in joint space:

$$
M(q)\ddot{q} + C(q, \dot{q})\dot{q} + g(q) = \tau
$$

| Term | Physical Meaning | Practical Impact |
|------|-----------------|------------------|
| $M(q)$ | Joint-space inertia matrix | Determines required motor torques for acceleration |
| $C(q, \dot{q})\dot{q}$ | Coriolis and centrifugal forces | Significant at high speeds; causes coupling between joints |
| $g(q)$ | Gravity torque vector | Must be compensated even when stationary; dominates at low speeds |
| $\tau$ | Applied joint torques | What your actuators must deliver |

This is equivalent to the Euler-Lagrange formulation where $L = T - V$:

$$
\frac{d}{dt} \left( \frac{\partial L}{\partial \dot{q}_i} \right) - \frac{\partial L}{\partial q_i} = \tau_i
$$

**Practitioner note**: For real-time control, the recursive Newton-Euler algorithm computes inverse dynamics in $O(n)$ time, compared to $O(n^3)$ for the Lagrangian approach. This matters when your control loop runs at 1 kHz.

### [[Control_Systems|Control]] — How Do We Get There?

The [[PID_Control|PID controller]] remains the workhorse of industrial robotics:

$$
\tau(t) = K_p e(t) + K_i \int_0^t e(\sigma) \, d\sigma + K_d \frac{de(t)}{dt}
$$

where $e(t) = q_{\text{desired}}(t) - q_{\text{actual}}(t)$ is the tracking error.

**Computed torque control** (model-based) improves performance by compensating for the nonlinear dynamics:

$$
\tau = M(q)\left(\ddot{q}_d + K_d \dot{e} + K_p e\right) + C(q, \dot{q})\dot{q} + g(q)
$$

This linearizes the closed-loop system to $\ddot{e} + K_d \dot{e} + K_p e = 0$, which is a simple second-order system with predictable settling behavior. Choosing $K_p = \omega_n^2$ and $K_d = 2\zeta\omega_n$ gives critically damped response at natural frequency $\omega_n$.

### The Jacobian — Connecting Joint and Task Space

The Jacobian $J(q)$ maps joint velocities to end-effector velocities:

$$
\dot{x} = J(q) \dot{q}, \quad \text{where } J_{ij} = \frac{\partial x_i}{\partial q_j}
$$

It also maps task-space forces to joint torques via the transpose:

$$
\tau = J^T(q) F_{\text{ext}}
$$

**Singularities** occur when $\det(J) = 0$ — the robot loses a degree of freedom and cannot move in certain directions. Every practitioner must map their robot's singular configurations during workcell design.

---

## Applications of Robotics

| Domain | Example Systems | Key Metrics |
|--------|----------------|-------------|
| **[[Manufacturing]]** | FANUC, ABB, KUKA arms | Cycle time < 1s, repeatability $\pm 0.02$ mm |
| **[[Healthcare]]** | Intuitive da Vinci, Stryker Mako | Sub-mm precision, 7-DOF tremor filtering |
| **[[Exploration]]** | NASA Mars rovers, underwater ROVs | Years of autonomous operation, extreme environments |
| **[[Agriculture]]** | Agrobot harvesters, DJI crop drones | Coverage rate (hectares/hr), crop damage < 1% |
| **[[Transportation]]** | Waymo, self-driving trucks | Miles between disengagements, sensor redundancy |
| **Logistics** | Amazon Kiva/Proteus, Locus AMRs | Picks/hr, fleet coordination of 1000+ units |

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #technology OR #automation WHERE contains(file.outlinks, [[Robotics]])
```
