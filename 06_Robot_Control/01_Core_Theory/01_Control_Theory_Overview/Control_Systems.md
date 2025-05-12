---
title: Control Systems
description: Defines Control Systems and Control Theory, explaining core concepts like feedback, stability, and common strategies used in robotics.
tags:
  - glossary-term
  - control-theory
  - control-systems
  - feedback
  - stability
  - robot-control
  - PID
  - core-concept
layout: default
category: robotics
author: Jordan_Smith_&_Gemini
date: 2025-04-29
permalink: /control_systems/
related:
  - "[[Control Theory]]"
  - "[[Feedback_Control]]"
  - "[[Stability]]"
  - "[[PID_Control]]"
  - "[[Computed Torque Control]]"
  - "[[Adaptive Control]]"
  - "[[Force Control]]"
  - "[[Visual Servoing]]"
  - "[[AI_and_Robot_Control]]"
  - "[[Kinematics]]"
  - "[[Dynamics]]"
  - "[[Actuator]]"
  - "[[Sensor]]"
  - "[[Glossary]]"
---

# Control Systems / Control Theory

**Control Systems** (or, more formally, the field of **[[Control Theory]]**) is a fundamental engineering discipline concerned with analyzing and influencing the behavior of dynamical systems to achieve desired outcomes. In [[Robotics]], control systems are essential for making [[Robots|robots]] perform tasks accurately, efficiently, and safely by managing their [[Actuator|actuators]] based on [[Sensor|sensor]] information and desired goals.

---

## Core Concepts

* **Dynamical System:** A mathematical model (often differential equations) describing how a system's state (e.g., robot joint positions and velocities - [[Kinematics]], [[Dynamics]]) changes over time in response to inputs.
* **Controller:** An algorithm or device that calculates the necessary control inputs (e.g., motor torques/voltages) to drive the system towards a desired state or trajectory.
* **Plant:** The physical system being controlled (e.g., the robot arm, mobile robot base).
* **Control Input (u):** Variables manipulated by the controller (e.g., [[Actuator]] commands).
* **System Output (y) / State (x):** Variables measured or estimated from the system ([[Sensor]] readings like joint angles, end-effector pose, contact forces).
* **Setpoint / Reference / Desired Trajectory ($y_d$, $x_d$):** The target value or time-varying path the system output should follow.
* **Error (e):** The difference between the desired setpoint/trajectory and the actual measured output (e = $y_d - y$).
* **[[Feedback_Control]] (Closed-Loop):** The controller uses measurements of the *actual* system output (feedback) to calculate the error and continuously adjust the control input to reduce this error. This makes the system robust to disturbances and modeling inaccuracies.
    * *Typical Structure:* Setpoint -> Comparator (+) -> Controller -> Plant -> Output -> Sensor -> Comparator (-).
* **Open-Loop Control:** The controller calculates the input based *only* on the desired setpoint and a model of the plant, without using feedback from the actual output. Simpler but very sensitive to errors and disturbances.
* **[[Stability]]:** A critical property ensuring the system's output remains bounded and typically converges towards the desired state without oscillations or divergence. Formal methods (Lyapunov analysis, root locus, Bode plots, Nyquist criterion) are used to analyze stability.
* **Performance:** Metrics used to evaluate how well a controller performs, such as:
    * **Transient Response:** Rise time, settling time, overshoot.
    * **Steady-State Error:** The remaining error after the system has settled.
    * **Robustness:** Ability to maintain stability and performance despite modeling errors or external disturbances.
    * **Tracking Accuracy:** How closely the system output follows a time-varying reference trajectory.

---

## Common Control Strategies in Robotics

While basic linear control theory forms the foundation, robotics often involves complex, nonlinear systems requiring specialized strategies:

* **[[PID_Control]]:** Proportional-Integral-Derivative control. A workhorse linear controller calculating input based on the present error (P), the accumulation of past errors (I), and the rate of change of error (D). Widely used for independent joint control in industrial robots. Simple and often effective, but may struggle with highly coupled or nonlinear dynamics.
* **[[Computed Torque Control]]:** (Nonlinear) Uses the robot's [[Inverse_Dynamics|inverse dynamics]] model to calculate the torques needed to cancel out nonlinear effects (Coriolis, centrifugal, gravity) and joint coupling. Transforms the complex robot dynamics into simpler, decoupled linear systems, which can then be easily controlled (often with [[PID_Control|PD]] action). Requires an accurate dynamic model.
* **[[Adaptive Control]]:** Adjusts controller parameters online based on observed system behavior or estimated parameter changes. Useful when the robot's dynamics are uncertain or change over time (e.g., picking up unknown payloads).
* **Robust Control:** Designs controllers guaranteed to maintain stability and performance within specified bounds, even with known limits on model uncertainty or disturbances.
* **Optimal Control:** Formulates control design as an optimization problem to minimize a cost function (e.g., energy, time, error magnitude) subject to system dynamics and constraints.
* **[[Force Control]]:** Explicitly controls the interaction forces between the robot and its environment. Essential for tasks involving contact, such as assembly, grinding, or physical human-robot interaction. Includes methods like impedance control and hybrid position/force control.
* **[[Visual Servoing]]:** Uses visual information directly from [[Camera_Systems]] in the feedback loop to control the robot's motion relative to visual targets.
* **Learning-Based Control:** Employs [[Machine Learning]] techniques like [[Reinforcement Learning (RL)]] or [[Imitation Learning]] to learn control policies from data or interaction, potentially bypassing the need for explicit models. See [[Neural_Networks_in_Control]].

Control systems are fundamental to enabling robots to perform useful tasks by bridging the gap between desired behavior and the complex physics of the real world.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #robotics OR #control-systems WHERE contains(file.outlinks, [[Robot_Control]])
