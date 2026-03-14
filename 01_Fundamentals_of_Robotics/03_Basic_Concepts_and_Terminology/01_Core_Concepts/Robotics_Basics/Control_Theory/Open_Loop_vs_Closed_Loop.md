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

In a closed-loop control system, the output $y(t)$ is fed back to the controller, which adjusts the input $u(t)$ based on the error $e(t)$ between the desired output $r(t)$ and the actual output $y(t)$:

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

### Closed-Loop Transfer Function

Combining the equations above, the closed-loop transfer function from reference $R(s)$ to output $Y(s)$ is:

$$
T(s) = \frac{Y(s)}{R(s)} = \frac{C(s)\,G(s)}{1 + C(s)\,G(s)}
$$

This is one of the most important equations in control engineering. The denominator $1 + C(s)G(s)$ is the **characteristic polynomial** --- its roots (the closed-loop poles) determine stability, bandwidth, and transient response.

### Example: Temperature Control

Consider a temperature control system for a heating element. In an open-loop configuration, the system sets the heating element to a predetermined power level based on the desired temperature. However, this approach cannot account for external factors like ambient temperature changes.

In a closed-loop configuration, a temperature sensor provides feedback on the actual temperature, allowing the controller to adjust the power level dynamically to maintain the desired temperature. This feedback mechanism ensures more accurate and stable temperature control.

---

## Discrete-Time Control (Z-Transform Formulations)

Modern robot controllers are implemented digitally, so the continuous-time formulations must be converted to discrete time. The Z-transform plays the same role in discrete systems that the Laplace transform plays in continuous systems.

### Discrete-Time State-Space

$$
\mathbf{x}[k+1] = A_d\,\mathbf{x}[k] + B_d\,\mathbf{u}[k]
$$

$$
\mathbf{y}[k] = C_d\,\mathbf{x}[k] + D_d\,\mathbf{u}[k]
$$

where $k$ is the sample index. The discrete matrices are obtained from the continuous matrices via the matrix exponential:

$$
A_d = e^{A T_s}, \quad B_d = \left(\int_0^{T_s} e^{A\tau}\,d\tau\right) B
$$

where $T_s$ is the sampling period.

### Discrete Transfer Function

The open-loop discrete transfer function is obtained via the Z-transform:

$$
G(z) = C_d(zI - A_d)^{-1}B_d + D_d
$$

The closed-loop discrete transfer function with a digital controller $C(z)$ is:

$$
T(z) = \frac{C(z)\,G(z)}{1 + C(z)\,G(z)}
$$

### Discretization Methods

Common methods for converting a continuous controller $C(s)$ to a discrete controller $C(z)$:

| Method | Substitution | Accuracy | Stability Preservation |
|---|---|---|---|
| Forward Euler (Forward Rectangular) | $s = \frac{z-1}{T_s}$ | First-order | Can create instability |
| Backward Euler | $s = \frac{z-1}{T_s z}$ | First-order | Always stable-preserving |
| Tustin (Bilinear / Trapezoidal) | $s = \frac{2}{T_s}\frac{z-1}{z+1}$ | Second-order | Stable, frequency warping |
| Zero-Order Hold (ZOH) | Exact for step inputs | Exact at samples | Preserves stability |

**Practical note:** The Tustin (bilinear) transform is the most common choice for discretizing PID controllers. For a 1 kHz servo loop ($T_s = 1$ ms), frequency warping is negligible below ~100 Hz, which covers the bandwidth of most robot joints.

**Example:** A continuous PID $C(s) = K_p + \frac{K_i}{s} + K_d s$ discretized via Tustin becomes:

$$
C(z) = K_p + K_i \frac{T_s}{2}\frac{z+1}{z-1} + K_d \frac{2}{T_s}\frac{z-1}{z+1}
$$

This is the form implemented in most industrial servo drives (Beckhoff TwinCAT, Yaskawa Sigma-7, Siemens S7-1500T).

---

## Stability Analysis

Stability is the most critical property of a control system. An unstable robot controller can cause violent, destructive motion. There are several complementary methods for analyzing stability.

### Continuous-Time Stability (s-Domain)

A continuous-time system is stable if and only if all poles of the closed-loop transfer function $T(s)$ have **negative real parts** (lie in the left half of the s-plane).

### Discrete-Time Stability (z-Domain)

A discrete-time system is stable if and only if all poles of $T(z)$ lie **inside the unit circle** $|z| < 1$.

### Bode Plot Analysis

A Bode plot shows the magnitude and phase of the open-loop transfer function $C(j\omega)G(j\omega)$ as a function of frequency $\omega$. Two key stability margins are read directly from the Bode plot:

- **Gain margin (GM):** The amount of gain increase (in dB) that would make the system unstable. Measured at the **phase crossover frequency** (where the phase is $-180^\circ$):

$$
\text{GM} = -20\log_{10}|C(j\omega_{pc})G(j\omega_{pc})| \quad \text{dB}
$$

- **Phase margin (PM):** The amount of additional phase lag that would make the system unstable. Measured at the **gain crossover frequency** (where $|C(j\omega)G(j\omega)| = 1$):

$$
\text{PM} = 180^\circ + \angle C(j\omega_{gc})G(j\omega_{gc})
$$

**Typical robotics specifications:**
- Industrial servo loops: GM $\geq$ 6 dB, PM $\geq$ 45 degrees
- Aggressive tuning (high-speed pick-and-place): PM = 30--40 degrees (faster response, less damping)
- Conservative tuning (collaborative robots, force control): PM = 60--70 degrees (more damping, slower response)

### Nyquist Criterion

The Nyquist plot traces $C(j\omega)G(j\omega)$ in the complex plane as $\omega$ varies from $-\infty$ to $+\infty$. The **Nyquist stability criterion** states:

The closed-loop system is stable if and only if the Nyquist contour of $C(j\omega)G(j\omega)$ encircles the critical point $(-1, 0)$ exactly $P$ times **counter-clockwise**, where $P$ is the number of open-loop right-half-plane poles.

For a stable open-loop system ($P = 0$), this simplifies to: the Nyquist plot must **not** encircle $(-1, 0)$.

**Robotics context:** The Nyquist criterion is particularly useful for analyzing robot systems with time delay (e.g., vision-based control with 30--100 ms camera latency, or remote teleoperation with network delay). Time delay adds phase lag $e^{-j\omega T_d}$ that spirals the Nyquist plot inward, reducing stability margins.

---

## When Open-Loop Control Is Actually Preferred

Despite the superiority of closed-loop control in most scenarios, there are important robotics cases where open-loop control is preferred or even necessary:

### Stepper Motors

Stepper motors are inherently open-loop actuators. They move a precise number of steps per pulse without requiring position feedback:

- **How it works:** A stepper motor divides a full rotation into discrete steps (typically 200 steps/rev = 1.8 deg/step, or 400 steps/rev with half-stepping). A step driver sends pulse signals, and the motor moves exactly one step per pulse.
- **Why open-loop works:** The motor's mechanical detent holds it at each step position. As long as the load torque stays below the motor's holding torque (and the acceleration stays below the pull-out torque curve), no feedback is needed.
- **Typical specs:** NEMA 17 stepper (common in 3D printers): holding torque 0.4--0.5 Nm, max speed ~1000 RPM, step accuracy $\pm$5% of one step.
- **When it fails:** If the load exceeds the motor's torque capacity, the motor "loses steps" (skips). The controller has no way to detect this without adding an encoder (which converts it to closed-loop).

**Used in:** 3D printers (Prusa, Ender), CNC routers, low-cost robot arms (e.g., AR3/AR4 by Annin Robotics), pick-and-place machines.

### Other Open-Loop Robotics Applications

- **Pneumatic grippers:** Many industrial grippers (Schunk PGN-plus, SMC MHZ2) operate open-loop: fully open or fully closed, actuated by a solenoid valve. No force feedback.
- **Bang-bang trajectory profiles:** Simple trapezoidal velocity profiles for point-to-point motion in non-critical applications.
- **Feedforward compensation:** Gravity compensation ($\tau = g(q)$) and friction compensation are open-loop terms added to improve closed-loop performance. The feedforward term is computed from the model, not from feedback.
- **Ballistic motions:** Throwing, kicking, or launching --- once the object is released, no feedback is possible. The entire trajectory is computed open-loop before release.

### Decision Framework

| Factor | Prefer Open-Loop | Prefer Closed-Loop |
|---|---|---|
| Disturbances | Small, predictable | Large, unpredictable |
| Accuracy required | Low--moderate ($\pm$0.1 mm) | High ($\pm$0.01 mm or better) |
| Cost sensitivity | High (no sensor needed) | Lower (sensor cost acceptable) |
| Safety criticality | Low | High (must detect faults) |
| System dynamics | Well-known, repeatable | Uncertain, time-varying |
| Example | Stepper on 3D printer Z-axis | Servo on industrial robot joint |

---

### Example: Temperature Control

Consider a temperature control system for a heating element. In an open-loop configuration, the system sets the heating element to a predetermined power level based on the desired temperature. However, this approach cannot account for external factors like ambient temperature changes.

In a closed-loop configuration, a temperature sensor provides feedback on the actual temperature, allowing the controller to adjust the power level dynamically to maintain the desired temperature. This feedback mechanism ensures more accurate and stable temperature control.

---

## Applications in Robotics

- **Motion Control**: Closed-loop control is essential for precise motion control in robotic manipulators and mobile robots, enabling them to follow trajectories accurately despite disturbances.
- **Temperature Regulation**: Closed-loop systems are used to maintain precise temperature levels in industrial processes and laboratory equipment.
- **Autonomous Vehicles**: Closed-loop control ensures stable and accurate navigation and maneuvering in autonomous vehicles, adapting to changing road conditions and obstacles.
- **Process Control**: Open-loop control is often used for simple, predictable processes, while closed-loop control is necessary for complex processes requiring precise regulation.
- **Stepper-Driven Systems**: Open-loop stepper motors are standard in 3D printers, CNC machines, and low-cost robotic arms where cost and simplicity outweigh the need for position feedback.
- **Hybrid Feedforward + Feedback**: Most high-performance robot controllers combine open-loop feedforward (gravity/friction compensation) with closed-loop feedback (PID/LQR) for the best of both worlds.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #robotics OR #control-theory WHERE contains(file.outlinks, [[Open_Loop_vs_Closed_Loop]])
```
