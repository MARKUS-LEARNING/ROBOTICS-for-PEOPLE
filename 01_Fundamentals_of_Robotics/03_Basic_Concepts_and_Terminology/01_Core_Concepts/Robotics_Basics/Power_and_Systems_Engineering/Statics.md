---
title: Statics
description: "Statics is the branch of mechanics that deals with the analysis of forces and moments on bodies at rest."
tags:
  - mechanics
  - engineering
  - physics
  - equilibrium
  - forces
type: Mechanical Concept
application: Analysis of forces and moments on bodies at rest
layout: default
category: mechanics
author: Jordan_Smith_and_le_Chat
date: 2025-05-02
permalink: /statics/
related:
  - "[[Mechanics]]"
  - "[[Engineering_Design]]"
  - "[[Structural_Analysis]]"
  - "[[Force_Analysis]]"
  - "[[Moment_Analysis]]"
  - "[[Equilibrium]]"
  - "[[Free_Body_Diagram]]"
---

# Statics

**Statics** is the branch of mechanics that deals with the analysis of forces and moments on bodies at rest. It focuses on understanding how forces and moments interact to maintain equilibrium in physical systems. Statics is fundamental in engineering and physics, providing the basis for analyzing the stability and balance of structures, machines, and other mechanical systems.

---
<img width="735" alt="image" src="https://github.com/user-attachments/assets/43301fd3-69bf-4ea6-836e-c3c6a64684d2" />

<font size=1>*source: https://www.youtube.com/watch?v=eBti6XIoR5U*</font>
---

## Key Concepts in Statics

1. **Equilibrium**: The state of a body where the net force and net moment acting on it are zero, resulting in no acceleration. Equilibrium is a central concept in statics, as it describes the conditions under which a body remains at rest.
   <br>

2. **Free Body Diagram**: A diagram that represents a body isolated from its environment, showing all the external forces and moments acting on it. Free body diagrams are essential tools in statics for visualizing and analyzing the forces and moments on a body.
   <br>

3. **Force Analysis**: The process of determining the magnitudes and directions of forces acting on a body. Force analysis involves resolving forces into components and using equilibrium equations to solve for unknown forces.
   <br>

4. **Moment Analysis**: The process of determining the moments (or torques) acting on a body and their effects on the body's rotational equilibrium. Moment analysis involves calculating the moments about a point or axis and using equilibrium conditions to solve for unknown moments.
   <br>

5. **Resultant Force**: The single force that has the same effect as a system of forces acting on a body. The resultant force is the vector sum of all the forces acting on the body.
   <br>

6. **Center of Gravity**: The point at which the weight of a body is considered to be concentrated. The center of gravity is the point where the resultant force of gravity acts on the body.
   <br>

---

## Key Equations

### Equilibrium Conditions

The conditions for equilibrium require that the sum of forces and moments in each direction must be zero:

$$
\sum F_x = 0, \quad \sum F_y = 0, \quad \sum F_z = 0
$$

$$
\sum M_x = 0, \quad \sum M_y = 0, \quad \sum M_z = 0
$$

### Resultant Force

The resultant force $F_R$ is calculated as the vector sum of the components of the forces acting on the body:

$$
F_R = \sqrt{F_x^2 + F_y^2 + F_z^2}
$$

### Moment About a Point

The moment $M$ of a force about a point is given by:

$$
M = r \times F
$$

where $r$ is the position vector from the point to the line of action of the force, and $F$ is the force vector.

### Center of Gravity

The coordinates of the center of gravity $(x_{\text{cg}}, y_{\text{cg}}, z_{\text{cg}})$ are calculated as:

$$
x_{\text{cg}} = \frac{\sum m_i x_i}{\sum m_i}, \quad y_{\text{cg}} = \frac{\sum m_i y_i}{\sum m_i}, \quad z_{\text{cg}} = \frac{\sum m_i z_i}{\sum m_i}
$$

where $m_i$ are the masses of the individual parts of the body, and $(x_i, y_i, z_i)$ are the coordinates of the centers of mass of the individual parts.

---

## Friction Models for Robotics

### Coulomb Friction Model

The **Coulomb friction model** is the most fundamental friction model, relating the maximum tangential friction force to the normal contact force:

$$
f_{\text{friction}} \leq \mu N
$$

where $\mu$ is the coefficient of static friction and $N$ is the normal force at the contact. This defines a **friction cone**: the contact force must lie within a cone of half-angle $\alpha = \arctan(\mu)$ about the surface normal.

In 3D, the friction cone at a contact point is:

$$
\mathcal{FC} = \left\{ \mathbf{f} \mid \sqrt{f_t^2 + f_s^2} \leq \mu f_n, \quad f_n \geq 0 \right\}
$$

where $f_n$ is the normal force component and $f_t$, $f_s$ are tangential components.

**Typical friction coefficients in robotics:**

| Contact Pair | $\mu_s$ (static) | $\mu_k$ (kinetic) |
|---|---|---|
| Rubber on metal | 0.5 -- 0.8 | 0.4 -- 0.6 |
| Rubber on wood | 0.6 -- 0.9 | 0.4 -- 0.7 |
| Steel on steel (dry) | 0.5 -- 0.8 | 0.4 -- 0.6 |
| Steel on steel (lubricated) | 0.1 -- 0.2 | 0.05 -- 0.15 |
| Rubber wheel on concrete | 0.6 -- 1.0 | 0.5 -- 0.8 |
| Silicone gripper on glass | 0.5 -- 1.0 | 0.4 -- 0.8 |

### Extended Friction Models

For precise robot control, the Coulomb model is extended to include:

**Viscous friction:**

$$
\tau_{\text{friction}} = \mu_c \cdot \text{sign}(\dot{q}) + b \cdot \dot{q}
$$

where $\mu_c$ is the Coulomb friction torque and $b$ is the viscous friction coefficient.

**Stribeck effect** (friction reduction at low velocities, common in geared joints):

$$
\tau_{\text{friction}} = \left[ \mu_c + (\mu_s - \mu_c) e^{-|\dot{q}/v_s|^2} \right] \text{sign}(\dot{q}) + b \cdot \dot{q}
$$

where $v_s$ is the Stribeck velocity.

---

## Static Wrench Analysis for Robot Grippers

### Wrench at a Grasp Contact

When analyzing a gripper holding an object in static equilibrium, the total wrench (force and torque) exerted on the object must balance gravity and any external loads:

$$
\sum_{i=1}^{k} \mathbf{w}_i + \mathbf{w}_{\text{ext}} = \mathbf{0}
$$

where $\mathbf{w}_i = [\mathbf{f}_i^T, \, \boldsymbol{\tau}_i^T]^T$ is the wrench at contact $i$ and $\mathbf{w}_{\text{ext}}$ includes gravity.

### Static Force at the Joint Level

For a robot holding a static payload, the required joint torques are determined by the transpose of the Jacobian:

$$
\boldsymbol{\tau} = J^T(\mathbf{q}) \, \mathbf{F}_{\text{ee}} + \mathbf{g}(\mathbf{q})
$$

where $\mathbf{F}_{\text{ee}}$ is the wrench applied at the end-effector and $\mathbf{g}(\mathbf{q})$ is the gravity torque vector.

**Practical implication:** This equation determines the continuous motor torque requirement. Motors must sustain this torque indefinitely without overheating, which is often the limiting factor in motor sizing (not peak torque).

---

## Center of Mass Calculation for Serial Chains

For a serial manipulator with $n$ links, the overall center of mass position in the base frame is:

$$
\mathbf{r}_{\text{cm}} = \frac{\sum_{i=1}^{n} m_i \, \mathbf{r}_i(\mathbf{q})}{\sum_{i=1}^{n} m_i}
$$

where $m_i$ is the mass of link $i$ and $\mathbf{r}_i(\mathbf{q})$ is the position of link $i$'s center of mass, computed from forward kinematics.

The **center of mass Jacobian** relates joint velocities to the velocity of the center of mass:

$$
\dot{\mathbf{r}}_{\text{cm}} = J_{\text{cm}}(\mathbf{q}) \, \dot{\mathbf{q}}
$$

where:

$$
J_{\text{cm}} = \frac{1}{M} \sum_{i=1}^{n} m_i J_{v,i}
$$

and $J_{v,i}$ is the linear velocity Jacobian for link $i$'s center of mass, and $M = \sum m_i$ is the total mass.

**Application in balance control:** For legged robots, the center of mass must remain within the **support polygon** (convex hull of the foot contacts) for static stability. The ZMP (Zero Moment Point) criterion extends this to dynamic walking.

---

## Impact on Engineering and Physics

- **Structural Design**: Statics is essential for designing structures that can withstand external forces and moments without collapsing. It provides the tools and principles necessary to analyze the stability and strength of buildings, bridges, and other structures.
  <br>

- **Mechanical Systems**: In mechanical engineering, statics is used to analyze the forces and moments acting on machines and components, ensuring that they operate safely and efficiently under load.
  <br>

- **Equilibrium Analysis**: Statics provides the foundation for understanding and analyzing equilibrium in physical systems, which is crucial for designing stable and balanced structures and mechanisms.
  <br>

- **Force and Moment Calculations**: The principles of statics are used to calculate forces and moments in various engineering applications, from simple machines to complex systems, ensuring that they can withstand the loads they are designed to bear.
  <br>

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #mechanics OR #engineering WHERE contains(file.outlinks, [[Statics]])
