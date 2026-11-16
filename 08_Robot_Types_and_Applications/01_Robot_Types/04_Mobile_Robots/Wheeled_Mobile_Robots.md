---
title: Wheeled Mobile Robots
description: "Wheeled robots are the most common mobile robots: differential drive, Ackermann (car-like), skid-steer, omnidirectional and balancing designs. Covers each drive's kinematics and constraints, a computed differential-drive example (speed, turn rate and turning radius from wheel speeds), why some robots cannot move sideways (nonholonomic constraints), odometry error, and the ROS 2 controllers that implement them."
aliases:
  - Wheeled Robots
  - Differential Drive
  - Ackermann Steering
  - Skid Steer
tags:
  - mobile-robot
  - kinematics
  - state-estimation
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
  - "[[Mobile_Robots]]"
  - "[[Kinematics]]"
related:
  - "[[Omnidirectional_Robots]]"
  - "[[Nav2_Navigation]]"
  - "[[ros2_control]]"
  - "[[Localization]]"
key_takeaways:
  - "A differential-drive robot's forward speed is the average of its wheel speeds and its turn rate is their difference divided by the track width."
  - "Differential-drive and car-like robots are nonholonomic: they cannot move sideways directly, so planners must produce paths they can actually drive (for cars, with a minimum turning radius)."
  - "Wheel odometry drifts with slip and calibration error, so it is fused with an IMU, lidar or vision; ROS 2 ships standard controllers (diff_drive, ackermann, mecanum) through ros2_control."
sources:
  - "Siegwart, R., Nourbakhsh, I. R. and Scaramuzza, D. (2011). Introduction to Autonomous Mobile Robots, 2nd edition. https://mitpress.mit.edu/9780262015356/introduction-to-autonomous-mobile-robots/"
  - "Lynch, K. M. and Park, F. C. (2017). Modern Robotics, chapter 13: Wheeled Mobile Robots. https://hades.mech.northwestern.edu/index.php/Modern_Robotics"
  - "Campion, G., Bastin, G. and d'Andréa-Novel, B. (1996). Structural properties and classification of kinematic and dynamic models of wheeled mobile robots. https://ieeexplore.ieee.org/document/481750"
  - "ros-controls (2026). ros2_controllers: diff_drive_controller. https://github.com/ros-controls/ros2_controllers/tree/master/diff_drive_controller"
  - "ros-controls (2026). ros2_controllers: ackermann_steering_controller. https://github.com/ros-controls/ros2_controllers/tree/master/ackermann_steering_controller"
---

# Wheeled Mobile Robots

Wheels are the most efficient way to move on a hard, flat surface, so most mobile robots have them: robot vacuums, warehouse robots, delivery robots, research platforms and self-driving cars. The arrangement of the wheels, the **drive type**, decides how the robot can move and how it is controlled.

> **Etymology.** *Wheel* is from Old English *hweol*, from a root meaning "to turn". *Ackermann* steering is named after Rudolph Ackermann, who patented Georg Lankensperger's steering linkage in London in 1818.

## Drive types

| Drive | Layout | Moves sideways? | Typical robots |
|---|---|---|---|
| **Differential** | Two driven wheels on one axle + casters | No | Vacuums, TurtleBot, many AMRs |
| **Skid-steer** | Driven wheels or tracks on each side, turning by skidding | No | Outdoor and rough-terrain robots (Husky-style) |
| **Ackermann** | Steered front wheels, driven rear (or all) | No; minimum turning radius | Cars, robotaxis, outdoor delivery |
| **Omnidirectional** | Mecanum, omni or steered (swerve) wheels | **Yes** | Factory AGVs, mobile manipulators; see [[Omnidirectional_Robots]] |
| **Self-balancing** | Two wheels, actively balanced | No | Segway-style platforms, some wheeled humanoids |

## Differential drive, computed

With wheel radius $r$, track width (distance between wheels) $L$, and wheel angular speeds $\omega_l, \omega_r$:

$$v = \frac{r(\omega_r + \omega_l)}{2}, \qquad \omega = \frac{r(\omega_r - \omega_l)}{L}, \qquad R = \frac{v}{\omega}$$

**Example.** $r = 0.05$ m, $L = 0.30$ m, left wheel at 10 rad/s, right wheel at 12 rad/s:

- forward speed $v = 0.05 \times (12 + 10)/2 = 0.55$ m/s;
- turn rate $\omega = 0.05 \times (12 - 10)/0.30 = 0.333$ rad/s (to the left, since the right wheel is faster);
- turning radius $R = 0.55 / 0.333 = 1.65$ m.

Equal speeds give a straight line ($R = \infty$); equal and opposite speeds spin the robot in place ($R = 0$). The controller inverts these equations: given a desired $(v, \omega)$ from the planner, it computes the wheel speeds.

## Nonholonomic constraints

A differential-drive robot has three pose coordinates $(x, y, \theta)$ but only two controls. Its wheels cannot slide sideways: the constraint $\dot x \sin\theta - \dot y \cos\theta = 0$ holds at all times. It can still *reach* any pose (parallel parking proves it), but not by any path. A car is further limited by its minimum turning radius; for wheelbase $L_w$ and steering angle $\delta$, $R = L_w/\tan\delta$, so a 0.5 m wheelbase at 14° of steering turns on a 2 m radius. Planners for such robots must respect these constraints: Hybrid-A*, state lattices, Reeds–Shepp curves (see [[Nav2_Navigation]]).

## Odometry and its errors

Counting wheel rotations gives **odometry**, a running pose estimate. Small errors in wheel radius or track width, and any wheel slip, accumulate without bound. A 1% error in track width alone makes the heading estimate drift by 1% of every turn. Robots therefore fuse wheel odometry with an IMU (often in an extended Kalman filter, such as `robot_localization`) and correct it against a map with lidar or vision (see [[Localization]]).

## In ROS 2

[[ros2_control]] provides standard controllers: `diff_drive_controller`, `ackermann_steering_controller`, `tricycle_controller` and `mecanum_drive_controller`. They take a velocity command (`geometry_msgs/TwistStamped`), drive the wheels, and publish odometry and the `odom → base_link` transform.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Wheeled_Mobile_Robots]])
```
