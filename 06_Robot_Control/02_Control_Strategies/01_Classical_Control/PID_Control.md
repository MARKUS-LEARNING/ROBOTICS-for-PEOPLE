---
title: PID Control
description: "PID (proportional-integral-derivative) control is the most widely used feedback controller in robotics and industry. Covers what each term does, a simulated robot-joint example comparing P, PD and PID under a load (overshoot, steady-state error), discrete implementation with anti-windup and derivative filtering, tuning methods, and where PID sits under learned policies in 2026."
aliases:
  - PID
  - PID Controller
  - Proportional-Integral-Derivative
tags:
  - pid
  - feedback-control
  - control
type: method
level: beginner
subfield:
  - control
status: keystone
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Feedback_Control]]"
related:
  - "[[Control_Theory_and_Principles]]"
  - "[[Control_Systems]]"
  - "[[ros2_control]]"
  - "[[Computed_Torque_Control]]"
  - "[[Drones]]"
key_takeaways:
  - "P reacts to the present error, I to the accumulated past error (removing steady offsets such as gravity loads), D to the predicted future error (adding damping)."
  - "In a simulated joint carrying a load, P alone overshot 30% and sat 0.1 rad off target; adding D removed the overshoot; adding I drove the load-induced error toward zero."
  - "Real PID needs anti-windup, a filtered derivative and a fixed fast sample rate; it runs underneath nearly every robot, including those driven by learned policies."
sources:
  - "Åström, K. J. and Hägglund, T. (2006). Advanced PID Control. https://www.isa.org/products/advanced-pid-control"
  - "Åström, K. J. and Murray, R. M. (2021). Feedback Systems, 2nd edition, chapter 11: PID Control. https://fbswiki.org/wiki/index.php/Feedback_Systems:_An_Introduction_for_Scientists_and_Engineers"
  - "ros-controls (2026). control_toolbox: PID implementation for ros2_control. https://github.com/ros-controls/control_toolbox"
  - "PX4 (2026). Multicopter PID tuning guide. https://docs.px4.io/main/en/config_mc/pid_tuning_guide_multicopter.html"
---

# PID Control

A **PID controller** computes a command from the error $e(t)$ between where something should be and where it is:

$$u(t) = K_p\, e(t) + K_i \int_0^t e(\tau)\, d\tau + K_d\, \frac{de(t)}{dt}$$

It needs no model of the system, just three gains, which is why it runs in the joints of robot arms, the motors of drones, the wheels of mobile robots and countless industrial processes.

> **Etymology.** Elmer Sperry's ship gyro-pilots (1910s) and Nicolas Minorsky's 1922 analysis of ship steering, which described all three terms, are the usual origins. The names come from what each term is *proportional* to: the error, its *integral* and its *derivative*.

## What each term does

| Term | Responds to | Effect | Too much causes |
|---|---|---|---|
| **P** | Present error | Pushes toward the target; stiffness | Oscillation, overshoot |
| **I** | Accumulated past error | Removes steady offsets (gravity, friction, constant loads) | Slow oscillation, overshoot, windup |
| **D** | Rate of change of error (predicts the future) | Damping; slows the approach | Noise amplification, jitter |

## Worked example: a robot joint under load

A joint is modelled as $J\ddot\theta + b\dot\theta = u + d$ with inertia $J = 0.01$ kg·m², friction $b = 0.1$ N·m·s/rad, and command torque $u$. It moves to 1 rad, and at $t = 1.5$ s a constant load torque $d = -0.2$ N·m appears (picking up a tool). Simulated at 1 kHz:

| Controller ($K_p, K_i, K_d$) | Overshoot to the step | Error under load at 3 s | Error at 6 s |
|---|---|---|---|
| P (2, 0, 0) | 30.4% | 0.100 rad | 0.100 rad |
| PD (2, 0, 0.2) | 0% | 0.100 rad | 0.100 rad |
| PID (4, 2, 0.3) | 4.4% | 0.013 rad | 0.003 rad |

Reading the table:
- **P alone** reaches the target (the joint integrates its own velocity) but overshoots, and under load settles where $K_p e = 0.2$ N·m, so $e = 0.2/2 = 0.1$ rad, a 5.7° sag.
- **D** adds damping and removes the overshoot, but cannot fight a constant load.
- **I** keeps accumulating the error until its output cancels the load, so the error decays toward zero. The price is a little overshoot and a slower tail.

## Implementing it properly

```python
class PID:
    def __init__(self, kp, ki, kd, dt, u_min, u_max, tau_d=0.005):
        self.kp, self.ki, self.kd, self.dt = kp, ki, kd, dt
        self.u_min, self.u_max, self.tau_d = u_min, u_max, tau_d
        self.i = 0.0; self.d = 0.0; self.prev_y = None

    def step(self, ref, y):
        e = ref - y
        # derivative on the measurement (no kick when ref jumps), low-pass filtered
        dy = 0.0 if self.prev_y is None else (y - self.prev_y) / self.dt
        a = self.dt / (self.tau_d + self.dt)
        self.d = (1 - a) * self.d + a * (-dy)
        self.prev_y = y
        u = self.kp * e + self.i + self.kd * self.d
        u_sat = min(max(u, self.u_min), self.u_max)
        if u == u_sat:                      # anti-windup: integrate only when not saturated
            self.i += self.ki * e * self.dt
        return u_sat
```

Three details matter in practice:
1. **Anti-windup**: stop integrating when the actuator saturates, or the integral grows huge and causes large overshoot afterwards.
2. **Derivative on measurement, filtered**: avoids a spike when the target jumps and limits noise.
3. **Fixed, fast sample rate**: 1 kHz for robot joints; delay erodes stability (see [[Control_Systems]]).

## Tuning

- **By hand**: raise $K_p$ until the response is quick with slight overshoot; add $K_d$ to damp it; add a small $K_i$ to remove offsets.
- **Ziegler–Nichols and relay auto-tuning**: find the gain and period at which the loop oscillates, then apply formula gains. Aggressive; usually softened.
- **Model-based**: choose gains for a target bandwidth and damping from a model of the joint.
- **Automatic**: drone autopilots (PX4, Betaflight) and many servo drives offer auto-tune routines; system identification plus optimisation is common in research.

## PID in 2026 robots

Learned locomotion policies usually output **joint position targets**, which a PD (often stiffness-and-damping) controller in each joint tracks at hundreds of hertz to 1 kHz. In ros2_control, the same role is played by `joint_trajectory_controller` and the PID utilities in `control_toolbox`. Drones run cascaded PID for attitude and rate. PID is the reliable last layer between intelligence and motors.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[PID_Control]])
```
