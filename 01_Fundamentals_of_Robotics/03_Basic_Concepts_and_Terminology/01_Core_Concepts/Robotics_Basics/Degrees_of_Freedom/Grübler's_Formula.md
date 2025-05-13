---
title: Grübler's Formula
description: Grübler's Formula is used to calculate the degrees of freedom (mobility) of a kinematic chain or mechanism, essential for understanding and designing robotic systems.
tags:
  - robotics
  - kinematics
  - degrees-of-freedom
  - mechanism-design
  - engineering
  - glossary-term
  - mechanism
  - manipulator-arm
  - mobile-robot
layout: default
category: robotics
author: Jordan_Smith_and_le_Chat
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
![image](https://github.com/user-attachments/assets/3e6ba0f5-8938-4b8f-bc1c-68031a173861)


<font size=1>*source: https://medium.com/@khalil_idrissi/degrees-of-freedom-of-a-robot-c21624060d25*</font>
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

## Applications in Robotics

- **Robotic Manipulators**: Grübler's formula is used to determine the degrees of freedom of robotic arms, which is essential for designing manipulators capable of performing complex tasks.
- **Mechanism Design**: Helps in designing mechanisms with the desired mobility, ensuring they can achieve the required motions for specific applications.
- **Analysis of Mechanical Systems**: Provides a systematic approach to analyzing the mobility of various mechanical systems, aiding in the understanding of their kinematic properties.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #robotics OR #kinematics WHERE contains(file.outlinks, [[Grübler's_Formula]])
