---
title: Omnidirectional Robots
description: "Omnidirectional robots can move in any direction and rotate at the same time, using mecanum wheels, omni wheels, swerve (independently steered) modules or ball drives. Covers how each works, mecanum-wheel kinematics with a computed sideways-motion example, trade-offs in efficiency and floor sensitivity, and where holonomic bases are used (factory AGVs, mobile manipulators, wheeled humanoids)."
aliases:
  - Omnidirectional Robot
  - Holonomic Robots
  - Mecanum Wheels
  - Swerve Drive
tags:
  - mobile-robot
  - kinematics
  - locomotion
type: robot-type
level: intermediate
subfield:
  - mobile-robots
  - kinematics-dynamics
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Wheeled_Mobile_Robots]]"
related:
  - "[[Mobile_Robots]]"
  - "[[ros2_control]]"
  - "[[Nav2_Navigation]]"
key_takeaways:
  - "Holonomic (omnidirectional) bases can move in x, y and rotate independently, so they can strafe into tight spots and hold an arm steady while repositioning."
  - "Mecanum wheels use angled rollers so wheel-speed combinations produce sideways motion; for pure strafing the diagonal wheel pairs spin in opposite senses."
  - "The price is efficiency, payload sensitivity and floor sensitivity (rollers dislike debris and gaps); swerve drives avoid most of this at higher mechanical cost."
sources:
  - "Lynch, K. M. and Park, F. C. (2017). Modern Robotics, chapter 13: Wheeled Mobile Robots. https://hades.mech.northwestern.edu/index.php/Modern_Robotics"
  - "Siegwart, R., Nourbakhsh, I. R. and Scaramuzza, D. (2011). Introduction to Autonomous Mobile Robots, 2nd edition. https://mitpress.mit.edu/9780262015356/introduction-to-autonomous-mobile-robots/"
  - "ros-controls (2026). ros2_controllers: mecanum_drive_controller. https://github.com/ros-controls/ros2_controllers/tree/master/mecanum_drive_controller"
---

# Omnidirectional Robots

An **omnidirectional** (holonomic) robot can move in any direction on the floor, forwards, sideways or diagonally, and rotate, all at once and without first turning. That makes it ideal for tight spaces and for **mobile manipulation**, where the base must make small precise moves while an arm works.

> **Etymology.** Latin *omnis*, "all", + *directio*. The **mecanum** wheel was invented by Bengt Ilon at the Swedish company Mecanum AB in 1973. *Holonomic* is from Greek *holos*, "whole", + *nomos*, "law": a system whose constraints restrict only position, not velocity.

## Ways to achieve it

| Mechanism | How | Pros | Cons |
|---|---|---|---|
| **Mecanum wheels** | Rollers at 45° around each wheel; four wheels in an X pattern | Compact, simple drive | Rollers wear; sensitive to floor debris and gaps; vibration |
| **Omni wheels** | Rollers at 90°; three wheels at 120° or four at 90° | Simple, smooth on flat floors | Low traction and payload |
| **Swerve (steered) drive** | Each wheel module steers and drives independently | Full traction and efficiency | Complex: two motors per wheel |
| **Ball drive** | Robot balances on a driven sphere | Graceful, very agile | Must balance actively; research platforms |

## Mecanum kinematics, computed

For four mecanum wheels of radius $r$, with $l_x$ and $l_y$ the half-length and half-width of the wheel layout, and body velocity $(v_x, v_y, \omega_z)$, the wheel speeds are (one common sign convention, X-pattern rollers):

$$
\begin{aligned}
\omega_{FL} &= \tfrac{1}{r}\,[\,v_x - v_y - (l_x + l_y)\omega_z\,] &\qquad \omega_{FR} &= \tfrac{1}{r}\,[\,v_x + v_y + (l_x + l_y)\omega_z\,]\\
\omega_{RL} &= \tfrac{1}{r}\,[\,v_x + v_y - (l_x + l_y)\omega_z\,] &\qquad \omega_{RR} &= \tfrac{1}{r}\,[\,v_x - v_y + (l_x + l_y)\omega_z\,]
\end{aligned}
$$

**Example: strafe left at 0.5 m/s** ($v_x = 0$, $v_y = 0.5$, $\omega_z = 0$) with $r = 0.05$ m. Each wheel turns at $0.5 / 0.05 = 10$ rad/s:
- front-left and rear-right at −10 rad/s;
- front-right and rear-left at +10 rad/s.

The diagonal pairs counter-rotate, and the roller forces cancel forward and add sideways. Signs depend on roller orientation and axis conventions, so check them against your robot's controller (ROS 2's `mecanum_drive_controller` documents its convention).

## Trade-offs

- **Efficiency.** Mecanum and omni wheels waste energy in roller slip, especially when strafing, and push less hard sideways than forwards.
- **Floors.** Rollers need clean, flat floors; cable covers and gaps cause jolts that disturb odometry.
- **Odometry.** Roller slip makes wheel odometry less reliable than on differential drives, so fuse it with an IMU and lidar.

## Where they are used (2026)

- Factory and warehouse transport of large loads in tight aisles (often swerve drives).
- **Mobile manipulators and wheeled humanoids**, where the base repositions precisely under a working arm or torso.
- Research and education robots, and robotics competitions, where omni and mecanum bases are standard.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Omnidirectional_Robots]])
```
