---
title: Robot Control
description: "How robots in particular are controlled: what makes robot dynamics hard (coupled, nonlinear, configuration-dependent), joint-space versus task-space control, the control modes motors expose (position, velocity, torque, impedance), a computed gravity-compensation example, and how model-based and learned controllers share the work on a 2026 robot."
aliases:
  - Robotic Control
  - Robot Motion Control
tags:
  - control
  - manipulation
  - dynamics
type: overview
level: intermediate
subfield:
  - control
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Control_Theory_Overview]]"
  - "[[Manipulator_Arm]]"
related:
  - "[[PID_Control]]"
  - "[[Computed_Torque_Control]]"
  - "[[Compliance]]"
  - "[[Model_Predictive_Control_(MPC)]]"
  - "[[Reinforcement_Learning_for_Robots]]"
  - "[[ros2_control]]"
key_takeaways:
  - "Robot dynamics are nonlinear and coupled: gravity, inertia and Coriolis effects change with posture, so fixed-gain loops alone leave errors that model-based terms remove."
  - "Without gravity compensation, a PD joint with stiffness 50 N·m/rad sags about 5° holding a 2 kg link; adding the model's gravity torque removes the sag without raising the gains."
  - "Choose the control mode by the task: position control for free motion, torque or impedance control for contact; learned policies typically command position targets or impedance set points."
sources:
  - "Lynch, K. M. and Park, F. C. (2017). Modern Robotics, chapter 11: Robot Control. https://hades.mech.northwestern.edu/index.php/Modern_Robotics"
  - "Khatib, O. (1987). A unified approach for motion and force control of robot manipulators: The operational space formulation. https://ieeexplore.ieee.org/document/1087068"
  - "Siciliano, B. and Khatib, O., eds. (2016). Springer Handbook of Robotics, 2nd edition, chapters on motion and force control. https://link.springer.com/referencework/10.1007/978-3-319-32552-1"
  - "Tedrake, R. (2026). Underactuated Robotics. https://underactuated.csail.mit.edu/"
---

# Robot Control

**Robot control** applies [[Control_Theory|control theory]] to the particular difficulties of robots: many joints that affect each other, dynamics that change with posture, contact with the world, and goals expressed in terms of where the hand or body should be rather than individual motor angles.

> **Etymology.** See [[Control_Theory_Overview]].

## Why robots are hard to control

A manipulator's dynamics are

$$M(\mathbf q)\ddot{\mathbf q} + C(\mathbf q, \dot{\mathbf q})\dot{\mathbf q} + \mathbf g(\mathbf q) = \boldsymbol\tau + J^\top(\mathbf q)\mathbf F_{ext}$$

- the inertia $M(\mathbf q)$ changes as the arm moves (an outstretched arm is harder to swing);
- Coriolis and centrifugal terms $C$ couple the joints at speed;
- gravity $\mathbf g(\mathbf q)$ depends on posture;
- contact forces $\mathbf F_{ext}$ enter through the Jacobian $J$.

Independent PID loops per joint ignore all this, which works for slow, stiff, highly geared industrial arms and less well for fast, light or torque-controlled robots.

## Joint space and task space

| | Joint space | Task (operational) space |
|---|---|---|
| Goal expressed as | Joint angles $\mathbf q$ | Hand pose, force, body centre of mass |
| Typical controller | Per-joint PID, computed torque | Operational-space control, inverse-kinematics-based control, whole-body QP |
| Strengths | Simple, robust | Natural for tasks, contact and redundancy |

Khatib's **operational-space formulation** (1987) controls the end effector's motion and force directly through the robot's dynamics. It is the ancestor of today's whole-body controllers for humanoids.

## Control modes motors expose

| Mode | You command | Good for |
|---|---|---|
| **Position** | Joint angles; the drive runs its own loop | Free-space motion, most industrial arms |
| **Velocity** | Joint speeds | Mobile bases, teleoperation |
| **Torque / current** | Joint torques | Model-based and force control, legged robots |
| **Impedance** | Stiffness, damping and a set point | Safe contact, learned policies (see [[Compliance]]) |

## Worked example: gravity compensation

A single link of mass 2 kg, with its centre of mass 0.25 m from the joint, is held at 30° above horizontal. Gravity pulls with a torque of

$$\tau_g = m g\, l_c \cos\theta = 2 \times 9.81 \times 0.25 \times \cos 30° = 4.25\ \text{N·m}.$$

A PD controller with stiffness $K_p = 50$ N·m/rad settles where $K_p e = \tau_g$, so it sags by $e = 4.25/50 = 0.085$ rad (4.9°). Options:
- raise $K_p$, which makes the arm stiffer and less safe, and amplifies noise;
- add an integral term, which is slow and can wind up;
- **add the model's gravity torque as feedforward**: $\tau = K_p e + K_d \dot e + \mathbf g(\mathbf q)$. The sag disappears and the gains can stay low, which keeps the arm compliant.

This is the simplest model-based controller. [[Computed_Torque_Control]] extends it to cancel the whole dynamics.

## The 2026 picture

| Layer | Who does it |
|---|---|
| Hard real-time joint and contact control | Classical model-based control (PD + gravity, impedance, whole-body QP or [[Model_Predictive_Control_(MPC)\|MPC]]) |
| Skills: walking, grasping, insertion | Increasingly learned policies ([[Reinforcement_Learning_for_Robots]], [[Diffusion_Policy]]) sending targets or impedance set points |
| Tasks | Planners and vision-language-action models |

Learned policies benefit from good low-level control: a policy that outputs impedance set points is easier to transfer from simulation, because contact behaviour is governed by known stiffness rather than raw torques.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Robot_Control]])
```
