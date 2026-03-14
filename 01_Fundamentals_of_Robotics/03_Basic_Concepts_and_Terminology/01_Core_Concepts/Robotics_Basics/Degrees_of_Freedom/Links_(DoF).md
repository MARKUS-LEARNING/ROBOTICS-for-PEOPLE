---
title: Links
description: "Defines Link: A rigid or semi-rigid component that connects joints in a robotic system, enabling structured movement and force transmission. Links are essential for defining the kinematic and dynamic properties of robots, both in manipulators and mobile robots."
tags:
  - glossary-term
  - component
  - structure
  - kinematics
  - dynamics
  - design
  - mechanism
  - manipulator-arm
  - mobile-robot
  - mechatronics
type: Mechanical Concept
application: Structural and dynamic definition in robotic systems
layout: default
category: robotics
author: Jordan_Smith_&_le_Chat
date: 2025-05-02
permalink: /link/
related:
  - "[[Actuator]]"
  - "[[Joints]]"
  - "[[Kinematic_Chains]]"
  - "[[Degrees_of_Freedom]]"
  - "[[Robot_Arm_Design]]"
  - "[[End_Effectors]]"
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

# Link

A **link** in robotics refers to a rigid or semi-rigid component that connects [[Joints]] in a robotic system. Links are crucial for transmitting forces and enabling structured movement, defining the robot's kinematic and dynamic properties. They form the backbone of both manipulator arms and mobile robots, influencing their overall design and functionality.

---

## Function

Links serve several critical functions in a robotic system:

* **Structural Support**: Provide the physical structure that holds the robot together, supporting [[Actuator|actuators]], [[Sensors]], and other components.
* **Force Transmission**: Transmit forces and torques between [[Joints]], enabling the robot to perform tasks such as lifting, pushing, or moving objects.
* **Kinematic Definition**: Define the spatial relationships between joints, determining the robot's [[Kinematic_Chains]] and [[Degrees_of_Freedom]].
* **Dynamic Interaction**: Influence the robot's dynamic behavior, affecting properties like inertia, [[Manipulator_Dynamics|momentum]], and [[Statics|stability]].

---

## Types of Links

Links can be categorized based on their design and function:

* **Rigid Links**: Made from stiff materials (e.g., metals, rigid plastics) to minimize deformation under load. Common in industrial robots and manipulators where precision and strength are crucial.
* **Semi-Rigid Links**: Used in applications requiring some flexibility, such as [[Humanoid_Robots]] or [[Exoskeletons]], where compliance and safety are important.
* **Modular Links**: Designed for [[Modular_Robotics|Modular Robotics]], allowing easy reconfiguration and customization of the robot's structure.
* **Lightweight Links**: Optimized for weight reduction, often used in mobile robots and aerospace applications where energy efficiency is a priority.

---

## Material Considerations

The choice of material for links is critical and depends on the application's requirements:

* **Metals**: Offer high strength and rigidity, suitable for industrial robots and heavy-duty applications. Common materials include aluminum, steel, and titanium.
* **Composites**: Provide a good strength-to-weight ratio, used in aerospace and mobile robots where weight is a concern. Examples include carbon fiber and fiberglass.
* **Polymers**: Lightweight and cost-effective, used in consumer robots and prototypes. Materials like ABS and PLA are common in 3D-printed links.
* **Advanced Materials**: Includes smart materials and [[Material_Science|nanomaterials]] that offer unique properties like shape memory, self-healing, or enhanced strength.

---

## Design Considerations

Designing links involves balancing several factors:

* **Strength and Rigidity**: Ensuring the link can withstand the expected loads and forces without deforming.
* **Weight**: Minimizing weight to improve energy efficiency and reduce inertia.
* **Cost**: Balancing performance with material and manufacturing costs.
* **Manufacturability**: Considering the ease of production, assembly, and maintenance.
* **Environmental Factors**: Ensuring the link can operate in the intended environment, considering factors like temperature, humidity, and corrosion.

---

## Mathematical Representation

The behavior of links in a robotic system can be mathematically represented using various equations:

### Kinematic Equations

The position and orientation of a robot's end-effector can be described using homogeneous transformation matrices. For a serial manipulator with $n$ links, the forward kinematics is given by:

$$
T_n = T_1 \cdot T_2 \cdot \ldots \cdot T_n
$$

where $T_i$ represents the transformation matrix for the $i$-th link.

### Link Deflection Under Load (Beam Bending)

In practice, robot links are not perfectly rigid. Understanding link deflection is critical for precision applications. A link can be modeled as a cantilever beam with an end load $F$:

$$
\delta_{\max} = \frac{F L^3}{3 E I}
$$

where:
- $\delta_{\max}$ is the maximum tip deflection (m)
- $F$ is the applied force at the tip (N)
- $L$ is the link length (m)
- $E$ is the Young's modulus of the material (Pa)
- $I$ is the area moment of inertia of the cross-section (m$^4$)

For a distributed load $w$ (N/m) along the link:

$$
\delta_{\max} = \frac{w L^4}{8 E I}
$$

**Common cross-sections and their area moments of inertia:**

| Cross-Section | Area Moment of Inertia $I$ |
|---|---|
| Solid circular ($d$ = diameter) | $\frac{\pi d^4}{64}$ |
| Hollow circular ($d_o$, $d_i$) | $\frac{\pi (d_o^4 - d_i^4)}{64}$ |
| Solid rectangular ($b \times h$) | $\frac{b h^3}{12}$ |
| Hollow rectangular ($b, h, t$) | $\frac{bh^3 - (b-2t)(h-2t)^3}{12}$ |

**Worked example:** An aluminum link ($E = 69$ GPa) with a hollow circular cross-section ($d_o = 80$ mm, $d_i = 70$ mm), length $L = 0.5$ m, carrying a 50 N end load:

$$
I = \frac{\pi (0.08^4 - 0.07^4)}{64} = 7.46 \times 10^{-7} \text{ m}^4
$$

$$
\delta_{\max} = \frac{50 \times 0.5^3}{3 \times 69 \times 10^9 \times 7.46 \times 10^{-7}} = 0.040 \text{ mm}
$$

This 40-micron deflection is negligible for most pick-and-place tasks but matters for precision machining or inspection applications (where tolerances may be 10 microns or less).

### Natural Frequency of a Link

A link's first natural frequency determines the maximum control bandwidth. For a cantilever beam with tip mass $m_{\text{tip}}$:

$$
f_n = \frac{1}{2\pi} \sqrt{\frac{3EI}{(0.24 m_{\text{link}} + m_{\text{tip}}) L^3}}
$$

where $m_{\text{link}}$ is the distributed mass of the link itself. The factor 0.24 converts the distributed beam mass to an equivalent tip mass for the first mode.

> **Practitioner's rule of thumb:** The servo bandwidth should be at most 1/3 of the first structural resonance frequency to avoid exciting vibrations. If your link resonates at 30 Hz, keep your servo bandwidth below 10 Hz.

---

## Material Selection for Robot Links

| Material | Density (kg/m$^3$) | Young's Modulus $E$ (GPa) | Yield Strength (MPa) | $E/\rho$ (Specific Stiffness) | Typical Use |
|---|---|---|---|---|---|
| Steel (4340) | 7,850 | 200 | 470 | 25.5 | Heavy industrial robots (KUKA KR1000) |
| Aluminum 6061-T6 | 2,700 | 69 | 276 | 25.6 | Most robot arms (UR5e, Franka Panda) |
| Aluminum 7075-T6 | 2,810 | 72 | 503 | 25.6 | High-strength robot links |
| Carbon fiber composite | 1,600 | 70-230 | 600-3,500 | 44-144 | Lightweight arms, aerospace robots |
| Titanium Ti-6Al-4V | 4,430 | 114 | 880 | 25.7 | Space robotics, medical devices |
| ABS (3D printed) | 1,040 | 2.3 | 40 | 2.2 | Prototypes, hobby robots |

> **Practitioner's note:** Specific stiffness ($E/\rho$) is roughly the same for steel, aluminum, and titanium. Choosing aluminum over steel does not make a link stiffer per unit mass -- it makes it lighter at the same stiffness. Carbon fiber composites genuinely offer higher specific stiffness but at 5-10x the cost and with anisotropic properties that complicate design.

---

## Link Mass Estimation

### Hollow Tube Approximation

Most robot links are hollow tubes or box sections. For a hollow circular tube:

$$
m = \rho \pi (r_o^2 - r_i^2) L
$$

where $r_o$ and $r_i$ are the outer and inner radii.

### Practical Weight Budget

For a serial manipulator, the link closest to the base carries all subsequent links, actuators, and payload. A useful rule of thumb for link mass ratios in a well-designed robot:

$$
m_i \approx \frac{m_{\text{payload}}}{(0.3)^{n-i}}
$$

where $m_i$ is the mass of link $i$ (counting from the tip), $n$ is the total number of links, and $m_{\text{payload}}$ is the payload mass. This reflects the geometric scaling: each link closer to the base must be heavier to support everything distal to it.

**Example:** For a 6-DoF robot with 5 kg payload:
- Link 6 (wrist): ~5 kg
- Link 5: ~17 kg
- Link 4: ~56 kg
- ... and so on toward the base

This is why base joints on industrial robots are massively built while wrist joints are compact.

---

## Applications

Links are integral to various robotic systems:

* **Manipulator Arms**: Form the structural framework that connects joints and [[End_Effectors]], enabling precise and controlled movements.
* **Mobile Robots**: Provide the chassis and structural components that support locomotion and interaction with the environment.
* **Humanoid Robots**: Require links that mimic human limbs, balancing strength, flexibility, and weight.
* **Exoskeletons**: Need links that are lightweight, strong, and adaptable to human movements.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #component OR #kinematics WHERE contains(file.outlinks, [[Links_(DoF)]])
```
