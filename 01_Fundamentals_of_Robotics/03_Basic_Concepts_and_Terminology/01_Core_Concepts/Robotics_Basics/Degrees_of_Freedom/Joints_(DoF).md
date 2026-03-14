---
title: Joints
description: Joints are critical components in robotic systems that connect links, allowing controlled motion and enabling robots to perform tasks by facilitating movement and force transmission.
tags:
  - glossary-term
  - component
  - motion
  - kinematics
  - dynamics
  - design
  - mechanism
  - manipulator-arm
  - mobile-robot
  - mechatronics
  - robotics
layout: default
category: robotics
author: Jordan_Smith_and_le_Chat
date: 2025-05-02
permalink: /joints/
related:
  - "[[Actuator]]"
  - "[[Links]]"
  - "[[Kinematic_Chains]]"
  - "[[Degrees_of_Freedom]]"
  - "[[Robot_Arm_Design]]"
  - "[[Manipulator_Dynamics]]"
  - "[[Statics]]"
  - "[[Material_Science]]"
  - "[[Structural_Analysis]]"
  - "[[Mechatronics]]"
  - "[[Robot_Design]]"
  - "[[Mobile_Robots]]"
  - "[[Locomotion]]"
  - "[[Wheeled_Mobile_Robots]]"
  - "[[Legged_Robots]]"
  - "[[Humanoid_Robots]]"
  - "[[Exoskeletons]]"
  - "[[Modular_Robotics]]"
---

# Joints

**Joints** are critical components in robotic systems that connect [[Links]], allowing controlled motion and enabling robots to perform tasks by facilitating movement and force transmission. They are essential for defining the [[Kinematic_Chains]] and [[Degrees_of_Freedom]] of a robot, influencing both its design and functionality. Joints are used in various robotic systems, including manipulator arms, mobile robots, and humanoid robots, to achieve precise and coordinated movements.

---

## Function

Joints serve several critical functions in a robotic system:

* **Motion Control**: Enable controlled movement by allowing rotation or translation between connected links.
* **Force Transmission**: Transmit forces and torques between [[Links]], enabling the robot to perform tasks such as lifting, pushing, or moving objects.
* **Kinematic Definition**: Define the spatial relationships between links, determining the robot's [[Kinematic_Chains]] and [[Degrees_of_Freedom]].
* **Dynamic Interaction**: Influence the robot's dynamic behavior, affecting properties like inertia, [[Manipulator_Dynamics|momentum]], and [[Statics|stability]].

---

## Types of Joints

Joints can be categorized based on their motion and design:

* **Revolute Joints**: Allow rotation about a single axis, similar to a hinge. Common in robotic arms and manipulators.
* **Prismatic Joints**: Allow linear motion along a single axis, similar to a sliding mechanism. Used in applications requiring linear actuation.
* **Spherical Joints**: Allow rotation in multiple axes, providing a wide range of motion. Used in applications like robotic shoulders or wrists.
* **Universal Joints**: Allow rotation in two axes, often used in mechanical linkages and transmissions.
* **Fixed Joints**: Do not allow any motion; used to rigidly connect links where no movement is required.

---

## Design Considerations

Designing joints involves balancing several factors:

* **Range of Motion**: The extent to which a joint can move, affecting the robot's workspace and flexibility.
* **Strength and Rigidity**: Ensuring the joint can withstand the expected loads and forces without deforming.
* **Precision**: The accuracy with which the joint can control motion, crucial for tasks requiring fine manipulation.
* **Weight**: Minimizing weight to improve energy efficiency and reduce inertia.
* **Cost**: Balancing performance with material and manufacturing costs.
* **Manufacturability**: Considering the ease of production, assembly, and maintenance.
* **Environmental Factors**: Ensuring the joint can operate in the intended environment, considering factors like temperature, humidity, and corrosion.

---

## Mathematical Representation

The behavior of joints in a robotic system can be mathematically represented using various equations:

### Kinematic Equations

The position and orientation of a robot's end-effector can be described using homogeneous transformation matrices. For a serial manipulator with \( n \) joints, the forward kinematics is given by:

$$
T_n = T_1 \cdot T_2 \cdot \ldots \cdot T_n
$$

where $T_i$ represents the transformation matrix for the \( i \)-th joint.

### Dynamic Equations

The dynamics of a robotic joint can be described using the Euler-Lagrange equation:

$$
\frac{d}{dt} \left( \frac{\partial L}{\partial \dot{q}} \right) - \frac{\partial L}{\partial q} = \tau
$$

where $L = T - V$ is the Lagrangian, $T$ is the kinetic energy, $V$ is the potential energy, $q$ is the generalized coordinate, and $\tau$ is the applied torque.

### Inertia and Momentum

The inertia matrix $I$ of a joint affects its dynamic response. The angular momentum $L$ of a joint rotating with angular velocity $\omega$ is given by:

$$
L = I \cdot \omega
$$

where $L$ is the angular momentum, $I$ is the inertia matrix, and $\omega$ is the angular velocity.

### Statics and Stability

The static equilibrium of a joint under forces $F$ and torques $\tau$ is given by:

$$
\sum F = 0 \quad \text{and} \quad \sum \tau = 0
$$

where $\sum F$ is the sum of forces and $\sum \tau$ is the sum of torques.

---

---

## Joint Friction and Backlash Modeling

Real joints exhibit friction and backlash that significantly affect control performance. Ignoring these effects leads to limit cycles, steady-state errors, and poor trajectory tracking.

### Coulomb + Viscous Friction Model

The most widely used friction model for revolute joints combines Coulomb (dry) friction and viscous (velocity-dependent) friction:

$$
\tau_{\text{friction}} = b \dot{q} + f_c \, \text{sgn}(\dot{q})
$$

where:
- $b$ is the viscous friction coefficient (N-m-s/rad)
- $f_c$ is the Coulomb friction torque (N-m)
- $\dot{q}$ is the joint velocity
- $\text{sgn}(\cdot)$ is the sign function

A more complete model adds **static friction** (stiction) and the **Stribeck effect**:

$$
\tau_{\text{friction}} = \left[ f_c + (f_s - f_c) e^{-(\dot{q}/v_s)^2} \right] \text{sgn}(\dot{q}) + b \dot{q}
$$

where $f_s$ is the static friction torque ($f_s > f_c$) and $v_s$ is the Stribeck velocity. Typical values for a small industrial robot joint (e.g., UR5e wrist joint): $f_c \approx 0.5$ N-m, $f_s \approx 0.8$ N-m, $b \approx 0.1$ N-m-s/rad, $v_s \approx 0.01$ rad/s.

### Backlash (Dead Zone) Model

Gear backlash creates a dead zone where the motor moves but the output does not respond:

$$
\tau_{\text{output}} = \begin{cases} k(\theta_m / N - \theta_l - \alpha) & \text{if } \theta_m / N - \theta_l > \alpha \\ 0 & \text{if } |\theta_m / N - \theta_l| \leq \alpha \\ k(\theta_m / N - \theta_l + \alpha) & \text{if } \theta_m / N - \theta_l < -\alpha \end{cases}
$$

where $\theta_m$ is the motor angle, $\theta_l$ is the load angle, $N$ is the gear ratio, $k$ is the gear stiffness, and $2\alpha$ is the backlash angle.

**Typical backlash values:**
- Harmonic drives: < 1 arcmin (0.017°) -- essentially zero backlash
- Cycloidal drives: 1-3 arcmin
- Planetary gearboxes: 3-10 arcmin
- Spur gears: 10-30+ arcmin

> **Practitioner's tip:** For collaborative robots and precision applications, harmonic drives (strain wave gears) are preferred despite their higher cost (~$500-$2000 per unit) because they provide near-zero backlash and high reduction ratios (50:1 to 160:1) in a compact package.

---

## Joint Compliance and Stiffness

### Joint Stiffness Model

A joint is never perfectly rigid. Joint stiffness $K_j$ relates the deflection $\Delta q$ to the applied torque:

$$
\tau = K_j \Delta q
$$

For a series elastic actuator (SEA), the stiffness is intentionally designed to be low for safety and force sensing:

$$
\tau_{\text{spring}} = K_s (q_m - q_l)
$$

where $q_m$ is the motor-side angle and $q_l$ is the link-side angle. This is the principle behind collaborative robots like the Franka Emika Panda.

### Cartesian Stiffness

The relationship between joint stiffness and Cartesian (end-effector) stiffness is:

$$
K_x = J^{-T} K_q J^{-1}
$$

where $K_q = \text{diag}(K_{q1}, K_{q2}, \ldots, K_{qn})$ is the diagonal joint stiffness matrix and $J$ is the manipulator Jacobian. This equation shows that Cartesian stiffness depends on the robot configuration (through $J$), which is why robots feel "stiffer" in some poses than others.

**Typical joint stiffness values:**
- Industrial robot (rigid): $K_j > 10{,}000$ N-m/rad
- Collaborative robot (SEA): $K_j \approx 100\text{--}1{,}000$ N-m/rad
- Soft robotic joint: $K_j \approx 1\text{--}50$ N-m/rad

---

## Joint Types: Practical Reference Table

| Joint Type | Symbol | DoF | Constraints (3D) | Motion Type | Common Applications | Key Limitations |
|---|---|---|---|---|---|---|
| **Revolute (R)** | $R$ | 1 | 5 | Rotation about 1 axis | Robot arms, hinges, pan-tilt units | Limited range ($\pm 170°$ typical) |
| **Prismatic (P)** | $P$ | 1 | 5 | Translation along 1 axis | Linear stages, SCARA Z-axis, CNC | Stroke limited by actuator |
| **Helical (H)** | $H$ | 1 | 5 | Coupled rotation + translation | Lead screws, ball screws | Fixed pitch ratio |
| **Cylindrical (C)** | $C$ | 2 | 4 | Rotation + translation (same axis) | Hydraulic cylinders with twist | Requires 2 actuators |
| **Universal (U)** | $U$ | 2 | 4 | Rotation about 2 intersecting axes | Cardan shafts, Stewart platform legs | Cannot transmit rotation about own axis |
| **Spherical (S)** | $S$ | 3 | 3 | Rotation about 3 axes | Shoulder joints, ball joints | Difficult to actuate directly |
| **Planar (E)** | $E$ | 3 | 3 | 2 translations + 1 rotation in plane | Planar mechanisms | Confined to a plane |

> **Practitioner's tip:** In serial manipulators, revolute joints dominate because they are easy to actuate with rotary motors, have well-understood dynamics, and can be made very precise. Prismatic joints appear in SCARA robots (Z-axis) and gantry systems. Spherical joints are almost never directly actuated -- they are typically replaced by a 3R wrist (three intersecting revolute joints) which is mechanically simpler to build and control.

---

## Applications

Joints are integral to various robotic systems:

* **Manipulator Arms**: Enable precise and controlled movements for tasks like assembly, welding, and material handling.
* **Mobile Robots**: Provide the necessary articulation for locomotion and interaction with the environment.
* **Humanoid Robots**: Require joints that mimic human joints, balancing strength, flexibility, and weight.
* **Exoskeletons**: Need joints that are lightweight, strong, and adaptable to human movements.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #component OR #kinematics WHERE contains(file.outlinks, [[Joints]])
