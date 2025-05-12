---
title: PID Control
description: Defines Proportional-Integral-Derivative (PID) Control, a common feedback control loop mechanism used in robotics and automation.
tags:
  - glossary-term
  - control-theory
  - feedback-control
  - algorithm
  - controller
layout: default
category: robotics
author: Jordan_Smith_and_le_Chat
date: 2025-04-28
permalink: /pid_control/
related:
  - "[[Control_Theory_and_Principles]]"
  - "[[Actuator]]"
  - "[[Feedback_Control]]"
  - "[[Motion_Control]]"
  - "[[Trajectory_Tracking]]"
  - "[[Independent-Joint_Control]]"
---

# PID Control (Proportional-Integral-Derivative Control)

**PID Control** is a widely used feedback control loop mechanism employed in industrial control systems, robotics, and other applications requiring continuously modulated control. It calculates an error value $e(t)$ as the difference between a measured process variable (e.g., joint angle, wheel speed) and a desired setpoint. The controller attempts to minimize the error over time by adjusting a control input, $u(t)$, based on three terms: Proportional (P), Integral (I), and Derivative (D).

---

## Control Terms

The PID controller algorithm involves three constant parameters, called the tuning parameters: the proportional, integral, and derivative gains, denoted $K_p$, $K_i$, and $K_d$.

* **Proportional (P) term**: This term produces an output proportional to the current error $e(t)$. It provides the primary driving force to reduce the error. Increasing $K_p$ typically increases the speed of the response but can lead to overshoot and oscillations. A high proportional gain alone often results in a steady-state error.
    * Contribution: $K_p e(t)$
  <br>

* **Integral (I) term**: This term is proportional to the magnitude and duration of the error, essentially summing the instantaneous error over time (integral). It works to eliminate the steady-state error that can occur with a pure P controller (e.g., counteracting gravity or constant friction). However, the integral term can worsen transient response, potentially increasing overshoot or causing oscillations (integral windup) if not properly tuned or limited.
    * Contribution: $K_i \int_{0}^{t} e(\tau) \, d\tau$
  <br>

* **Derivative (D) term**: This term is proportional to the rate of change (derivative) of the error. It acts to dampen oscillations and improve stability by anticipating future error based on its current rate of change. It effectively adds damping to the system response. However, the derivative term is sensitive to high-frequency noise in the measurement, which can cause large, undesirable fluctuations in the control output if the error signal is noisy. Filtering is often applied to the derivative term.
    * Contribution: $K_d \frac{de(t)}{dt}$
  <br>

---

## Control Law

The standard form of the PID control algorithm combines these three terms:

$$
u(t) = K_p e(t) + K_i \int_{0}^{t} e(\tau) \, d\tau + K_d \frac{de(t)}{dt}
$$

where:
* $u(t)$ is the control output (e.g., motor torque or voltage).
* $e(t) = y_{sp}(t) - y_{m}(t)$ is the error signal (setpoint minus measured variable).
* $K_p$, $K_i$, $K_d$ are the non-negative proportional, integral, and derivative gains, respectively.

In digital implementations, discrete-time versions of this equation are used, often employing Z-transforms or finite difference approximations for the integral and derivative terms.

---

## Applications in Robotics

PID control is ubiquitous in robotics due to its simplicity, effectiveness, and clear physical interpretation of its terms.

* **Industrial Robots**: Used extensively for independent joint control in manipulators (e.g., in the classic PUMA 560 controller). It regulates joint position and velocity. PD (Proportional-Derivative) control, often with gravity compensation, is a common variant used for regulation tasks.
  <br>

* **Mobile Robots**: Used for controlling wheel speeds in differential drive or omnidirectional robots, controlling steering angles, and maintaining desired headings using feedback from sensors like gyroscopes or compasses.
  <br>

* **Underwater & Aerial Robots**: Used in autopilots and for controlling thrusters or control surfaces.
  <br>

* **Other Applications**: Used in process control for regulating temperature, pressure, flow rate, etc.
  <br>

---

## Tuning

The process of finding the optimal values for the gains $K_p$, $K_i$, $K_d$ is called **PID tuning**. The goal is typically to achieve a desired closed-loop performance, balancing factors like:

* Fast response time (rise time).
* Minimal overshoot.
* Short settling time.
* Zero steady-state error.
* Stability and robustness to noise and disturbances.

Common tuning methods include:
* **Manual Tuning**: Adjusting gains iteratively based on observing the system's response.
* **Ziegler-Nichols Method**: A heuristic method based on finding the ultimate gain and period of oscillation.
* **Software Tools**: Various software packages offer auto-tuning capabilities.
* **Rule-Based Methods**: Like the "Compound Tuning Rule" which provides guidelines based on desired error reduction for repetitive tasks.

Proper tuning is crucial as poorly tuned PID controllers can lead to instability, oscillations, or sluggish performance. The simplicity of the controller structure often belies the complexity of achieving optimal performance across all operating conditions, especially for nonlinear systems like robot manipulators.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #control-theory OR #feedback-control WHERE contains(file.outlinks, [[PID_Control]])
