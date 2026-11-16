---
title: Compliance (Robotics)
description: "Compliance is how much a robot gives way under force. Covers passive compliance (springs, soft materials, series-elastic actuators) and active compliance (impedance and admittance control), a computed example of contact force with stiff versus compliant control, how to choose stiffness and damping, and why compliant action spaces help learned policies with contact-rich tasks in 2026."
aliases:
  - Impedance Control
  - Admittance Control
  - Compliant Control
  - Series Elastic Actuator
tags:
  - control
  - compliance
  - manipulation
  - safety
type: concept
level: intermediate
subfield:
  - control
  - manipulation
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Robot_Control]]"
  - "[[PID_Control]]"
related:
  - "[[Collaborative_Robots]]"
  - "[[Soft_Robotics]]"
  - "[[Robot_Hands]]"
  - "[[Reinforcement_Learning_for_Robots]]"
key_takeaways:
  - "A compliant robot behaves like a spring and damper around its target, so position errors at contact produce bounded forces instead of very large ones."
  - "With 500 N/m of stiffness, pushing 1 cm into a surface produces 5 N; a stiff position loop of 100,000 N/m would try to produce 1,000 N."
  - "Impedance (force from motion) suits torque-controlled robots; admittance (motion from measured force) suits stiff, geared industrial arms with a force sensor; learned policies that output impedance targets transfer to contact tasks more reliably."
sources:
  - "Lynch, K. M. and Park, F. C. (2017). Modern Robotics, chapter 11.5–11.6: force control and hybrid motion-force control. https://hades.mech.northwestern.edu/index.php/Modern_Robotics"
  - "Siciliano, B. and Khatib, O., eds. (2016). Springer Handbook of Robotics, 2nd edition, chapter on force control. https://link.springer.com/referencework/10.1007/978-3-319-32552-1"
  - "Martín-Martín, R. et al. (2019). Variable Impedance Control in End-Effector Space: An Action Space for Reinforcement Learning in Contact-Rich Tasks. https://arxiv.org/abs/1906.08880"
  - "ros-controls (2026). ros2_controllers: admittance_controller. https://github.com/ros-controls/ros2_controllers/tree/master/admittance_controller"
---

# Compliance (Robotics)

**Compliance** is the opposite of stiffness: how much a robot yields when pushed. A perfectly stiff robot holds its position whatever the force, so any position error at contact turns into enormous forces. A compliant robot gives way like a spring, which makes contact gentle, tolerant of error and safe for people.

> **Etymology.** Latin *complere*, "to fulfil", through *comply*, "to yield, to accommodate". Neville Hogan's 1985 papers on **impedance control** set out the idea of controlling the relationship between motion and force rather than either alone.

## Passive and active compliance

| Kind | How | Examples |
|---|---|---|
| **Passive** | Physical springs, soft materials, elastic elements | Series-elastic actuators (a spring between motor and joint), soft fingertips, remote centre compliance devices for peg insertion |
| **Active** | The controller makes the robot behave like a spring and damper | Impedance and admittance control on arms, legs and hands |
| **Variable** | Stiffness changed mechanically or in software during a task | Variable-stiffness actuators; variable impedance control |

## Impedance and admittance control

**Impedance control** makes the end effector behave like a mass–spring–damper around a target $\mathbf x_d$:

$$\mathbf F = K(\mathbf x_d - \mathbf x) + D(\dot{\mathbf x}_d - \dot{\mathbf x})$$

The robot *measures motion and outputs force* (joint torques $\boldsymbol\tau = J^\top \mathbf F$ plus gravity compensation). It suits torque-controlled robots: cobots with joint torque sensing, legged robots.

**Admittance control** is the dual: it *measures force* (with a force-torque sensor) and *outputs motion*, moving the position target as if the robot were a mass–spring–damper. It suits stiff, highly geared industrial arms that only accept position commands.

## Worked example: why compliance matters at contact

A robot is told to place its tool on a table but its estimate of the table height is 1 cm too low, so it tries to push 1 cm *into* the surface.

| Control | Effective stiffness | Contact force |
|---|---|---|
| Compliant (impedance) | 500 N/m | $500 \times 0.01 = 5$ N |
| Stiff position loop | ~100,000 N/m | $100{,}000 \times 0.01 = 1{,}000$ N (in practice: the motor saturates, something breaks, or a safety stop trips) |

The compliant robot presses with 5 N, gentle enough to hold a part against a surface. Choosing stiffness is a trade-off: low stiffness tolerates errors and contact, high stiffness tracks precisely in free space. Damping is usually set for near-critical behaviour, $D \approx 2\sqrt{KM}$ for apparent mass $M$, to avoid bouncing.

## Uses

- **Assembly and insertion**: pegs in holes, connectors, screws, where small misalignments must be absorbed.
- **Polishing, sanding, wiping**: maintain a set contact force while moving.
- **Safe human-robot interaction**: hand guiding and collision tolerance (see [[Collaborative_Robots]]).
- **Legged locomotion**: compliant legs absorb landing impacts.
- **Grasping**: compliant fingers conform to object shape (see [[Robot_Hands]]).

## Compliance and learning (2026)

Learned manipulation policies are often trained to output **impedance targets** (desired pose plus stiffness) rather than raw joint positions or torques. Martín-Martín et al. (2019) showed that variable impedance in end-effector space is a better action space for reinforcement learning in contact-rich tasks: the low-level controller handles contact physics predictably, which also eases sim-to-real transfer. Many 2025–26 VLA deployments on arms run under a Cartesian impedance controller for the same reason.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Compliance]])
```
