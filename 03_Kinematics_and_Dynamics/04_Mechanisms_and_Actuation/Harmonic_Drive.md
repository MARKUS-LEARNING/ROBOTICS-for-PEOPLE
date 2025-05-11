---
title: Harmonic Drive
description: A Harmonic Drive is a mechanical gear system that uses a flexible spline, a wave generator, and a circular spline to transmit motion with high precision and efficiency.
tags:
  - mechanics
  - robotics
  - engineering
  - actuators
  - gears
type: Mechanical System
application: High precision motion transmission in robotic systems
layout: default
category: robotics
author: Jordan_Smith_and_le_Chat
date: 2025-04-29
permalink: /harmonic-drive/
related:
  - "[[Robot_Design]]"
  - "[[Mechatronics]]"
  - "[[Gears]]"
  - "[[Transmission_Mechanisms]]"
  - "[[Kinematics_and_Dynamics]]"
  - "[[Manipulator_Arm]]"
---

# Harmonic Drive

A **Harmonic Drive** is a mechanical gear system that uses a flexible spline, a wave generator, and a circular spline to transmit motion with high precision and efficiency. This unique design allows for high gear ratios, compact size, and minimal backlash, making it ideal for applications requiring precise control and positioning, such as robotics, aerospace, and medical devices.

---

## Components of a Harmonic Drive

1. **Wave Generator**: An elliptical cam that deforms the flexible spline, causing it to engage with the circular spline. The wave generator is typically connected to the input shaft.

2. **Flexible Spline**: A thin, flexible gear with external teeth that mesh with the circular spline. The flexible spline deforms to match the shape of the wave generator as it rotates.

3. **Circular Spline**: A rigid, circular gear with internal teeth that mesh with the flexible spline. The circular spline is usually fixed to the housing and does not rotate.

---

## Key Equations

- **Gear Ratio**:
  $$
  \text{Gear Ratio} = \frac{N_f}{N_c - N_f}
  $$
  where $N_f$ is the number of teeth on the flexible spline, and $N_c$ is the number of teeth on the circular spline.
  <br></br>

- **Efficiency**:
  $$
  \eta = \frac{P_{\text{out}}}{P_{\text{in}}}
  $$
  where $\eta$ is the efficiency, $P_{\text{out}}$ is the output power, and $P_{\text{in}}$ is the input power. Harmonic drives typically have high efficiency, often exceeding 90%.
  <br></br>

- **Torque Transmission**:
  $$
  T_{\text{out}} = T_{\text{in}} \cdot \text{Gear Ratio} \cdot \eta
  $$
  where $T_{\text{out}}$ is the output torque, $T_{\text{in}}$ is the input torque, and $\eta$ is the efficiency.

---

## Impact on Robotics

- **Precision and Accuracy**: Harmonic drives provide high precision and accuracy in motion transmission, making them ideal for robotic applications that require precise positioning and control.

- **Compact Design**: The compact size of harmonic drives allows for space-efficient designs, which is particularly beneficial in applications with limited space, such as robotic joints and manipulators.

- **High Gear Ratios**: Harmonic drives can achieve high gear ratios in a single stage, reducing the need for multiple gear stages and simplifying the design of robotic systems.

- **Low Backlash**: The minimal backlash in harmonic drives ensures smooth and precise motion, which is crucial for applications requiring high positional accuracy.

- **Design and Integration**: The selection and integration of harmonic drives are important aspects of [[Robot_Design]] and [[Mechatronics]], influencing the performance and capabilities of robotic systems. Understanding harmonic drives is essential for designing effective motion transmission systems in robotics.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts
```dataview
LIST FROM #kinematics OR #mobile-robot WHERE contains(file.outlinks, [[Harmonic_Drive]])
