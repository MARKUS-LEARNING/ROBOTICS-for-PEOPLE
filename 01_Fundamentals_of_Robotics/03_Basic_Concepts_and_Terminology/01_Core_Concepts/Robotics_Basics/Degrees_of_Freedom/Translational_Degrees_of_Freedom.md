---
title: Translational Degrees of Freedom
description: Translational Degrees of Freedom refer to the number of independent linear motions a mechanical system can undergo, crucial for understanding and designing robotic systems with specific motion capabilities.
tags:
  - robotics
  - kinematics
  - degrees-of-freedom
  - translational-motion
  - mechanism-design
  - engineering
  - glossary-term
  - manipulator-arm
  - mobile-robot
layout: default
category: robotics
author: Jordan_Smith_and_le_Chat
date: 2025-05-02
permalink: /translational_degrees_of_freedom/
related:
  - "[[Kinematics]]"
  - "[[Degrees_of_Freedom]]"
  - "[[Mechanism_Design]]"
  - "[[Cartesian_Coordinates]]"
  - "[[Robot_Design]]"
  - "[[Workspace_Analysis]]"
---

# Translational Degrees of Freedom

**Translational Degrees of Freedom** refer to the number of independent linear motions a mechanical system can undergo. Understanding translational degrees of freedom is crucial for designing robotic systems with specific motion capabilities, such as manipulators and mobile robots. These degrees of freedom determine how a system can move linearly along different axes, affecting its ability to navigate and interact with its environment.

---

## Key Concepts

### Translational Motion

Translational motion involves movement along a straight line without rotation. In robotics, translational motion is essential for tasks such as reaching, positioning, and navigating through space.

### Cartesian Coordinates

Cartesian coordinates are commonly used to describe translational motion in three-dimensional space. They define the position of a point using three coordinates: $x$, $y$, and $z$.

### Workspace Analysis

Workspace analysis involves determining the range of motion a robotic system can achieve within its environment. Translational degrees of freedom are crucial for defining the workspace and ensuring the system can reach all necessary points.

---

## Mathematical Formulation

### Position Representation

The position of a point in three-dimensional space can be represented using Cartesian coordinates:

$$
\mathbf{p} = \begin{bmatrix} x \\ y \\ z \end{bmatrix}
$$

where $x$, $y$, and $z$ are the coordinates along the respective axes.

### Translational Motion Equation

The translational motion of a point from an initial position $\mathbf{p}_0$ to a final position $\mathbf{p}_f$ can be described by:

$$
\mathbf{p}_f = \mathbf{p}_0 + \mathbf{d}
$$

where $\mathbf{d}$ is the displacement vector:

$$
\mathbf{d} = \begin{bmatrix} \Delta x \\ \Delta y \\ \Delta z \end{bmatrix}
$$

### Example: Robotic Arm

Consider a robotic arm with three translational joints (prismatic joints) that allow linear motion along the $x$, $y$, and $z$ axes. The position of the end-effector can be described by:

$$
\mathbf{p} = \begin{bmatrix} x \\ y \\ z \end{bmatrix}
$$

The workspace of the robotic arm is defined by the range of motion along each axis. For example, if each joint can move between 0 and 1 meter, the workspace is a cubic volume with side length 1 meter.

---

---

## Prismatic Joint Dynamics and Force Equations

### Equation of Motion

A prismatic joint translating along a single axis with position $d$, mass $m$ (including payload), and actuator force $F$:

$$
F = m \ddot{d} + b \dot{d} + f_c \, \text{sgn}(\dot{d}) + m g \sin\alpha + F_{\text{ext}}
$$

where:
- $m$ is the total moving mass (slider + payload) (kg)
- $b$ is the viscous friction coefficient (N-s/m)
- $f_c$ is the Coulomb friction force (N)
- $g$ is gravitational acceleration ($9.81$ m/s$^2$)
- $\alpha$ is the angle of the joint axis from horizontal
- $F_{\text{ext}}$ is the external force (from other joints, contact, etc.)

For a vertical prismatic joint ($\alpha = 90°$), the gravity term $mg$ becomes a constant load that the actuator must support continuously -- this is why counterbalances or brakes are essential on vertical axes.

### PD Control for a Prismatic Joint

A simple PD position controller:

$$
F = K_p (d_{\text{ref}} - d) + K_d (\dot{d}_{\text{ref}} - \dot{d}) + m g \sin\alpha
$$

The last term is **gravity compensation** -- without it, the joint will sag under its own weight. Typical gains for a 10 kg payload linear axis with 1 m stroke: $K_p \approx 5{,}000\text{--}20{,}000$ N/m, $K_d \approx 100\text{--}500$ N-s/m.

---

## Linear Actuator Technologies: Comparison

| Parameter | Ball Screw | Lead Screw | Belt Drive | Linear Motor | Rack & Pinion | Pneumatic Cylinder |
|---|---|---|---|---|---|---|
| **Max Speed** | 1-2 m/s | 0.5 m/s | 5-10 m/s | 5-10+ m/s | 2-5 m/s | 1-2 m/s |
| **Max Force** | 5-100 kN | 1-20 kN | 0.5-5 kN | 0.1-10 kN | 5-50 kN | 0.1-50 kN |
| **Repeatability** | 1-5 um | 10-50 um | 50-200 um | 0.1-1 um | 20-100 um | 0.1-1 mm |
| **Stroke** | 0.1-6 m | 0.1-3 m | 0.5-20+ m | 0.1-6 m | Unlimited | 0.05-3 m |
| **Efficiency** | 85-95% | 25-50% | 95%+ | 95%+ | 85-95% | 50-70% |
| **Backdrivable?** | Yes | Usually no | Yes | Yes | Yes | Yes |
| **Self-locking?** | No | Often yes | No | No | No | No |
| **Cost** | Medium-High | Low | Low-Medium | High | Medium | Low |
| **Typical Use** | CNC, robots | Low-speed positioning | Gantries, 3D printers | Semiconductor, pick-and-place | Large gantries | Clamping, simple pick-and-place |

### Ball Screw Mechanics

The most common actuator for precision prismatic joints. The relationship between motor torque $\tau_m$ and linear force $F$:

$$
F = \frac{2 \pi \eta \tau_m}{p}
$$

where $p$ is the lead (linear distance per revolution, typically 5-20 mm) and $\eta$ is the efficiency (0.85-0.95 for ball screws). The linear velocity:

$$
v = \frac{p \cdot n}{60}
$$

where $n$ is the motor speed in RPM. For a ball screw with $p = 10$ mm driven at 3000 RPM: $v = 0.5$ m/s, and with motor torque $\tau_m = 2$ N-m: $F = \frac{2\pi \times 0.9 \times 2}{0.01} = 1{,}131$ N.

### Linear Motor

Direct-drive linear motors eliminate the screw mechanism entirely, providing the highest speed and accuracy but at significant cost. Force is generated directly:

$$
F = K_f \cdot I
$$

where $K_f$ is the force constant (N/A) and $I$ is the current. Typical force constants range from 10-200 N/A.

---

## Workspace Volume Calculation for Prismatic Joints

### Cartesian Robot (PPP)

For a Cartesian robot with three independent prismatic joints along orthogonal axes, each with stroke $[d_{i,\min}, d_{i,\max}]$:

$$
V = (d_{1,\max} - d_{1,\min}) \times (d_{2,\max} - d_{2,\min}) \times (d_{3,\max} - d_{3,\min})
$$

This is a simple rectangular prism. For a gantry robot with strokes of 2 m x 1.5 m x 0.5 m:

$$
V = 2.0 \times 1.5 \times 0.5 = 1.5 \text{ m}^3
$$

### SCARA Robot (RRP Configuration)

A SCARA has two revolute joints ($\theta_1, \theta_2$) and one prismatic joint ($d_3$). The planar workspace is an annular region:

$$
A = \pi (L_1 + L_2)^2 - \pi (L_1 - L_2)^2 = 4\pi L_1 L_2
$$

where $L_1$ and $L_2$ are the link lengths. The 3D workspace volume is:

$$
V = A \times (d_{3,\max} - d_{3,\min}) = 4\pi L_1 L_2 \times \Delta d_3
$$

For a typical SCARA (e.g., Epson T6: $L_1 = L_2 = 300$ mm, $\Delta d_3 = 200$ mm):

$$
V = 4\pi \times 0.3 \times 0.3 \times 0.2 = 0.226 \text{ m}^3
$$

### Cylindrical Robot (RPP)

A cylindrical robot with one revolute ($\theta$, full rotation) and two prismatic joints ($d_r, d_z$):

$$
V = \pi (d_{r,\max}^2 - d_{r,\min}^2) \times (d_{z,\max} - d_{z,\min})
$$

> **Practitioner's tip:** Workspace volume alone does not determine robot suitability. A Cartesian gantry may have a larger workspace than a 6-axis arm, but it cannot orient the tool. Always consider the *task workspace* -- the subset where the robot can achieve the required pose (position + orientation) for the specific application.

---

## Applications in Robotics

- **Robotic Manipulators**: Translational degrees of freedom enable manipulators to position their end-effectors precisely for tasks such as picking and placing objects.
- **Mobile Robots**: Allow mobile robots to navigate through their environment by moving linearly along different paths.
- **Industrial Automation**: Implementing translational motion in automated machinery to perform tasks such as assembly and material handling.
- **3D Printing**: Translational degrees of freedom are used to control the movement of the print head in three-dimensional space, enabling precise fabrication of objects.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #robotics OR #kinematics WHERE contains(file.outlinks, [[Translational_Degrees_of_Freedom]])
```
