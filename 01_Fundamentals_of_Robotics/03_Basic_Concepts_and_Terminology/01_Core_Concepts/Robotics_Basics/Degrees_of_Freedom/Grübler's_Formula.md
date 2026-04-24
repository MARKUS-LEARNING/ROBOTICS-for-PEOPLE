---
title: Grübler's Formula
description: Grübler's Formula is used to calculate the degrees of freedom (mobility) of a kinematic chain or mechanism, essential for understanding and designing robotic systems.
tags:
  - robotics
  - kinematics
  - degrees-of-freedom
  - mechanism-design
  - engineering
  - mechanism
  - manipulator-arm
  - mobile-robot
layout: default
category: robotics
author: Jordan_Smith
date: 2025-05-02
permalink: /grublers_formula/
related:
  - "[[Kinematics]]"
  - "[[Degrees_of_Freedom]]"
  - "[[Mechanism_Design]]"
  - "[[Kutzbach's_Criterion]]"
  - "[[Chebychev-Grübler-Kutzbach_Criterion]]"
---

# Grübler's Formula

**Grübler's Formula** is used to calculate the degrees of freedom (mobility) of a kinematic chain or mechanism. It is essential for understanding and designing robotic systems, as it provides insight into how many independent parameters are needed to define the configuration of the mechanism. This formula is particularly useful in robotics for analyzing the mobility of robotic manipulators and other mechanical systems.

---

## Key Concepts

### Degrees of Freedom

Degrees of freedom refer to the number of independent parameters that define the configuration of a mechanical system. In robotics, degrees of freedom determine the complexity and capability of a robotic mechanism to perform various tasks.

### Kinematic Chain

A kinematic chain is a series of rigid bodies connected by joints that allow relative motion between them. Grübler's formula is applied to these chains to determine their mobility.

### Mobility

Mobility is a measure of the number of independent parameters required to specify the position of all the links in a mechanism. It is calculated using Grübler's formula and is crucial for designing mechanisms with desired motion capabilities.

---

## Mathematical Formulation

### Grübler's Formula

Grübler's formula for calculating the degrees of freedom $M$ of a kinematic chain is given by:

$$
M = 3(n - 1 - j) + \sum_{i=1}^{j} f_i
$$

where:
- $n$ is the number of links in the mechanism.
- $j$ is the number of joints.
- $f_i$ is the number of degrees of freedom of the $i$-th joint.

### Simplified Version

For planar mechanisms, where motion is constrained to a plane, the formula simplifies to:

$$
M = 3(n - 1 - j) + \sum_{i=1}^{j} f_i
$$

For spatial mechanisms, the formula is:

$$
M = 6(n - 1 - j) + \sum_{i=1}^{j} f_i
$$

### Example

Consider a planar mechanism with 4 links and 4 revolute joints (each with 1 degree of freedom). Using Grübler's formula:

$$
M = 3(4 - 1 - 4) + 4 \times 1 = 1
$$

This indicates that the mechanism has 1 degree of freedom.

---

## Worked Example: 6R Serial Manipulator (Spatial)

A standard 6-axis industrial robot (e.g., KUKA KR6, ABB IRB 6700) is a serial chain of 6 revolute joints. Let us verify its DoF using the spatial Grubler formula.

**Parameters:**
- $n = 7$ links (6 moving links + 1 fixed base/ground)
- $j = 6$ joints (all revolute)
- Each revolute joint has $f_i = 1$ DoF

**Calculation:**

$$
M = 6(n - 1 - j) + \sum_{i=1}^{j} f_i = 6(7 - 1 - 6) + 6 \times 1 = 6(0) + 6 = 6
$$

This confirms 6 DoF, which is the minimum required for arbitrary positioning and orienting of the end-effector in 3D space.

### Worked Example: Planar Five-Bar Linkage

A five-bar linkage is a single closed-loop planar mechanism commonly used in parallel robots (e.g., pantograph mechanisms).

**Parameters:**
- $n = 5$ links (4 moving + 1 ground)
- $j = 5$ revolute joints (all planar, $f_i = 1$)

**Calculation (planar):**

$$
M = 3(5 - 1 - 5) + 5 \times 1 = 3(-1) + 5 = 2
$$

The mechanism has 2 DoF, meaning 2 independent inputs (actuators) are needed. This is exactly the configuration used in many 2-DoF parallel planar robots.

---

## Exceptions and Special Cases

### Over-Constrained Mechanisms

Grubler's formula assumes that all constraints are independent. When special geometric conditions exist (parallel axes, coplanar joints, symmetric dimensions), some constraints become redundant, and the formula **under-predicts** the true mobility.

| Mechanism | Links ($n$) | Joints ($j$) | Joint DoF | Grubler Prediction | Actual DoF | Reason |
|---|---|---|---|---|---|---|
| Bennett linkage | 4 | 4R | 4 | $6(4-1-4)+4 = -2$ | 1 | Special twist-angle/link-length ratios |
| Sarrus linkage | 6 | 6R | 6 | $6(6-1-6)+6 = 0$ | 1 | Alternating perpendicular axes |
| Planar 4-bar (3D formula) | 4 | 4R | 4 | $6(4-1-4)+4 = -2$ | 1 | All axes parallel (planar sub-group) |
| Bricard linkage | 6 | 6R | 6 | $6(6-1-6)+6 = 0$ | 1 | Specific axis geometry |

**Correction approach:** For over-constrained mechanisms, a corrected formula is sometimes used:

$$
M = d(n - 1 - j) + \sum_{i=1}^{j} f_i + f_{\text{passive}}
$$

where $d$ is the order of the motion subgroup (e.g., $d = 3$ for planar, $d = 6$ for general spatial) and $f_{\text{passive}}$ accounts for passive (redundant) constraints removed due to special geometry.

### Instantaneous vs. Full-Cycle Mobility

Some mechanisms have mobility that changes with configuration. The **instantaneous** DoF (based on the Jacobian rank) can differ from the **full-cycle** DoF:

$$
M_{\text{inst}} = n_q - \text{rank}(\mathbf{J}(\mathbf{q}))
$$

where $n_q$ is the number of joint variables and $\mathbf{J}(\mathbf{q})$ is the constraint Jacobian evaluated at configuration $\mathbf{q}$. At singular configurations, $\text{rank}(\mathbf{J})$ drops and the instantaneous mobility increases.

---

## Common Pitfalls When Applying Grubler's Formula

1. **Forgetting to count the ground link.** The ground (base, frame) is always counted as link $n = 1$. A serial robot with 6 moving links has $n = 7$ total links.

2. **Using the wrong formula dimension.** Use $d = 3$ (planar) only when all motion is truly confined to a plane. If any joint axis is out-of-plane, use $d = 6$ (spatial). Mixing these is the most common student error.

3. **Double-counting constraints at multi-body joints.** A pin joint connecting 3 links at one point is not one joint -- it is 2 independent revolute joints. In general, $k$ links meeting at a single pin create $k - 1$ revolute joints.

4. **Ignoring redundant constraints.** If two constraints enforce the same geometric condition (e.g., two parallel revolute joints both prevent the same out-of-plane rotation), one constraint is redundant. The formula will under-predict DoF.

5. **Confusing joint DoF with constraint count.** A revolute joint in 3D has $f_i = 1$ DoF but imposes $6 - 1 = 5$ constraints. The formula already accounts for this through the $6(n - 1 - j)$ term, so you only add the $f_i$ values, not subtract constraints directly.

> **Practitioner's tip:** When Grubler gives you a negative number, do not panic. It usually means you are analyzing a structure (a truss), not a mechanism, or you have an over-constrained mechanism with special geometry. Check your joint count, verify the formula dimension, and look for geometric special cases.

---

## Applications in Robotics

- **Robotic Manipulators**: Grubler's formula is used to determine the degrees of freedom of robotic arms, which is essential for designing manipulators capable of performing complex tasks.
- **Mechanism Design**: Helps in designing mechanisms with the desired mobility, ensuring they can achieve the required motions for specific applications.
- **Analysis of Mechanical Systems**: Provides a systematic approach to analyzing the mobility of various mechanical systems, aiding in the understanding of their kinematic properties.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #robotics OR #kinematics WHERE contains(file.outlinks, [[Grübler's_Formula]])
```
