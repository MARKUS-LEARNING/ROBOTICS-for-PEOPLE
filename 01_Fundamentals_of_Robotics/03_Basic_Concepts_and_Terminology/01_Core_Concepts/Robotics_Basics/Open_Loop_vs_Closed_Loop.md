---
title: Open Loop vs Closed Loop
description: Open Loop vs Closed Loop are fundamental control concepts in robotics, representing two distinct approaches to controlling systems, with and without feedback.
tags:
  - robotics
  - control-theory
  - open-loop
  - closed-loop
  - feedback
  - system-design
  - engineering
  - glossary-term
  - manipulator-arm
  - mobile-robot
layout: default
category: robotics
author: Jordan_Smith_and_le_Chat
date: 2025-05-02
permalink: /open_loop_vs_closed_loop/
related:
  - "[[Control_Theory]]"
  - "[[Feedback_Control]]"
  - "[[PID_Control]]"
  - "[[System_Dynamics]]"
  - "[[Stability_Analysis]]"
  - "[[Robot_Design]]"
---

# Open Loop vs Closed Loop

**Open Loop vs Closed Loop** are fundamental control concepts in robotics, representing two distinct approaches to controlling systems. Open-loop control systems operate without feedback, relying solely on predefined inputs to achieve the desired output. In contrast, closed-loop control systems incorporate feedback, using the system's output to adjust the control inputs dynamically, ensuring more accurate and stable performance.

---
<img src="https://www.newport.com/mam/celum/celum_assets/np/Picomotor_Figure_1_600w.gif"></img>
<font size=1>*source: https://www.newport.com/n/open-vs-closed-loop-picomotor*</font>
---

## Key Concepts

### Open-Loop Control

Open-loop control systems do not use feedback to adjust their behavior. Instead, they rely on a predetermined set of inputs to produce the desired output. These systems are simpler to design and implement but are susceptible to errors and disturbances, as they cannot compensate for variations in the system or environment.

### Closed-Loop Control

Closed-loop control systems use feedback to continuously monitor the system's output and adjust the control inputs accordingly. This feedback mechanism allows the system to correct for errors and disturbances, ensuring more precise and stable operation. Closed-loop systems are essential for applications requiring high accuracy and reliability.

### Feedback Control

Feedback control is the process of using the system's output to adjust the control inputs. It is a fundamental aspect of closed-loop control systems, enabling them to respond to changes and maintain desired performance.

### System Dynamics

System dynamics describe how a system responds to inputs over time. Understanding system dynamics is crucial for designing effective control strategies, whether open-loop or closed-loop.

---

## Mathematical Formulation

### Open-Loop Control

In an open-loop control system, the output $y(t)$ is determined solely by the input $u(t)$ and the system's dynamics, represented by the transfer function $G(s)$:

$$
Y(s) = G(s) \cdot U(s)
$$

where $Y(s)$ and $U(s)$ are the Laplace transforms of the output and input, respectively.

### Closed-Loop Control

In a closed-loop control system, the output $y(t)$ is fed back to the controller, which adjusts the input $u(t)$based on the error \( e(t) \) between the desired output $r(t)$ and the actual output $y(t)$:

$$
E(s) = R(s) - Y(s)
$$

$$
U(s) = C(s) \cdot E(s)
$$

$$
Y(s) = G(s) \cdot U(s)
$$

where $C(s)$ is the controller transfer function, and $E(s)$ is the error signal.

### Example: Temperature Control

Consider a temperature control system for a heating element. In an open-loop configuration, the system sets the heating element to a predetermined power level based on the desired temperature. However, this approach cannot account for external factors like ambient temperature changes.

In a closed-loop configuration, a temperature sensor provides feedback on the actual temperature, allowing the controller to adjust the power level dynamically to maintain the desired temperature. This feedback mechanism ensures more accurate and stable temperature control.

---

## Applications in Robotics

- **Motion Control**: Closed-loop control is essential for precise motion control in robotic manipulators and mobile robots, enabling them to follow trajectories accurately despite disturbances.
- **Temperature Regulation**: Closed-loop systems are used to maintain precise temperature levels in industrial processes and laboratory equipment.
- **Autonomous Vehicles**: Closed-loop control ensures stable and accurate navigation and maneuvering in autonomous vehicles, adapting to changing road conditions and obstacles.
- **Process Control**: Open-loop control is often used for simple, predictable processes, while closed-loop control is necessary for complex processes requiring precise regulation.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #robotics OR #control-theory WHERE contains(file.outlinks, [[Open_Loop_vs_Closed_Loop]])
