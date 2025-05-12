---
title: Shape Memory Alloys
description: Shape Memory Alloys (SMAs) are materials that can return to a predefined shape upon heating, exhibiting unique properties like superelasticity and shape memory effect.
tags:
  - materials
  - robotics
  - engineering
  - actuators
  - sensors
type: Material
application: Actuators and sensors in robotic systems
layout: default
category: robotics
author: Jordan_Smith_and_le_Chat
date: 2025-05-02
permalink: /shape-memory-alloys/
related:
  - "[[Material_Science]]"
  - "[[Robot_Design]]"
  - "[[Mechatronics]]"
  - "[[Actuator]]"
  - "[[Sensors]]"
  - "[[Control_Systems]]"
  - "[[Manipulator Arm]]"
  - "[[Legged_Robots]]"
  - "[[Wheeled_Mobile_Robots]]"
---

# Shape Memory Alloys

**Shape Memory Alloys (SMAs)** are materials that can return to a predefined shape upon heating, exhibiting unique properties like superelasticity and the shape memory effect. These alloys are particularly useful in robotics for creating actuators and sensors that can adapt to changes in temperature or mechanical stress. SMAs are typically made from nickel-titanium (NiTi) alloys, commonly known as Nitinol, but other compositions are also used.

---

## Key Properties of Shape Memory Alloys

1. **Shape Memory Effect**: The ability of the material to return to its original shape upon heating after being deformed. This effect is due to a phase transformation between martensite and austenite phases.
   <br>

2. **Superelasticity**: The ability to undergo large deformations and return to the original shape without heating, due to the reversible phase transformation under stress.
   <br>

3. **High Strength and Durability**: SMAs can withstand significant mechanical stress and are resistant to fatigue, making them suitable for robust robotic applications.
   <br>

---

## Key Equations

### Phase Transformation Temperature

The transformation temperature $T_{\text{trans}}$ of an SMA can be described by:

$$
T_{\text{trans}} = A_s + \frac{\sigma}{C_A}
$$

where:
- $T_{\text{trans}}$ is the transformation temperature.
- $A_s$ is the austenite start temperature.
- $\sigma$ is the applied stress.
- $C_A$ is the Clausius-Clapeyron constant.

### Strain Recovery

The recoverable strain $\epsilon_{\text{rec}}$ in an SMA is given by:

$$
\epsilon_{\text{rec}} = \epsilon_{\text{total}} - \epsilon_{\text{plastic}}
$$

where:
- $\epsilon_{\text{rec}}$ is the recoverable strain.
- $\epsilon_{\text{total}}$ is the total strain.
- $\epsilon_{\text{plastic}}$ is the plastic strain.

### Superelastic Stress-Strain Relationship

The stress-strain relationship within the superelastic range is described by:

$$
\sigma = E \cdot \epsilon
$$

where:
- $\sigma$ is the stress.
- $E$ is the modulus of elasticity.
- $\epsilon$ is the strain within the superelastic range.

---

## Impact on Robotics

- **Actuators**: SMAs are used to create actuators that can change shape or apply force in response to temperature changes. These actuators are lightweight, quiet, and can produce significant force, making them ideal for applications like robotic grippers and artificial muscles.
  <br>

- **Sensors**: The unique properties of SMAs make them suitable for creating sensors that can detect changes in temperature or mechanical stress. These sensors can be integrated into robotic systems for feedback control and monitoring.
  <br>

- **Design and Integration**: The selection and integration of SMAs are important aspects of [[Robot_Design|Robot Design]] and [[Mechatronics]], influencing the performance and capabilities of robotic systems. SMAs enable the creation of adaptive and responsive components that enhance robotic functionality.
  <br>

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #materials OR #robotics WHERE contains(file.outlinks, [[Shape_Memory_Alloys]])
