---
title: Collision Avoidance
description: "How robots avoid hitting things and people: layered defences from global planning through local reactive avoidance (dynamic window, velocity obstacles, ORCA, MPPI) to independent safety monitors and certified stops. Includes computed time-to-collision and braking examples, control barrier functions as safety filters for any controller (including learned policies), and arm collision avoidance."
aliases:
  - Obstacle Avoidance
  - Collision Detection and Avoidance
tags:
  - obstacle-detection
  - navigation
  - safety
  - motion-planning
type: concept
level: intermediate
subfield:
  - planning
  - mobile-robots
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Autonomous_Navigation]]"
related:
  - "[[Nav2_Navigation]]"
  - "[[Collision_Detection]]"
  - "[[Collaborative_Robots]]"
  - "[[MoveIt2]]"
key_takeaways:
  - "Safety comes from layers: plan around known obstacles, react to new ones, and keep an independent monitor that can stop the robot whatever the planner or policy does."
  - "Time to collision drives the urgency: a robot at 1 m/s and a person at 1.4 m/s walking toward each other from 5 m apart have about 2.1 s."
  - "Control barrier functions turn 'never get closer than d' into a constraint that filters any controller's commands, which is how learned policies are given hard safety guarantees."
sources:
  - "Fox, D., Burgard, W. and Thrun, S. (1997). The dynamic window approach to collision avoidance. https://ieeexplore.ieee.org/document/580977"
  - "van den Berg, J., Guy, S. J., Lin, M. and Manocha, D. (2011). Reciprocal n-body collision avoidance (ORCA). https://gamma.cs.unc.edu/ORCA/"
  - "Ames, A. D. et al. (2019). Control Barrier Functions: Theory and Applications. https://arxiv.org/abs/1903.11199"
  - "Macenski, S., Moore, T., Lu, D. V., Merzlyakov, A. and Ferguson, M. (2023). From the desks of ROS maintainers: A survey of modern & capable mobile robotics algorithms in the robot operating system 2. https://arxiv.org/abs/2307.15236"
---

# Collision Avoidance

**Collision avoidance** keeps a robot from hitting obstacles, people and itself while it pursues its goal. It is the non-negotiable part of autonomy: a robot that reaches its goal 99% of the time is useful, but a robot that collides 1% of the time is not deployable.

> **Etymology.** Latin *collidere*, "to strike together". The **dynamic window approach** (Fox, Burgard and Thrun, 1997) and **velocity obstacles** (Fiorini and Shiller, 1998) made real-time reactive avoidance practical for mobile robots.

## Layers of defence

| Layer | Acts on | Typical method | Speed |
|---|---|---|---|
| Global planning | Known map | Plan paths with clearance on inflated costmaps | 1–10 Hz |
| Local planning and control | Sensed obstacles, including moving ones | Dynamic window, MPC or MPPI with obstacle costs, velocity obstacles | 10–50 Hz |
| Safety monitor | Raw sensor data | Collision monitor or safety lidar zones: slow down, stop | 50–100 Hz, independent of the planner |
| Certified safety function | Hardware | Safety-rated stops and speed limits (see [[Collaborative_Robots]]) | Milliseconds, safety PLC |

The independent lower layers are what make it acceptable to put learned planners or policies on top.

## Worked example: how much time is there?

A robot moving at 1.0 m/s and a person walking at 1.4 m/s approach each other head-on from 5 m apart. Their closing speed is 2.4 m/s, so the **time to collision** is

$$\text{TTC} = \frac{5}{2.4} = 2.1\ \text{s}.$$

If the robot reacts in 0.5 s and decelerates at 1 m/s², it travels $1.0 \times 0.5 + 1.0^2/(2 \times 1) = 1.0$ m before stopping. Stopping is easy here, but stopping is not the only goal: robots that halt whenever a person is near block corridors. Good local planners *slow and steer* early, which is where social navigation and prediction come in.

## Reactive methods

- **Dynamic window approach (DWA)**: sample velocities the robot can reach within the next control period, simulate each briefly, and choose the best that does not collide.
- **Velocity obstacles and ORCA**: compute the set of velocities that would collide with each moving obstacle within a time horizon, and choose outside it. ORCA shares the responsibility for avoiding a collision between agents, which suits multi-robot systems and crowd simulation.
- **MPC and MPPI**: optimise or sample whole trajectories with obstacle costs and constraints (see [[Nav2_Navigation]]).
- **Artificial potential fields**: repulsion from obstacles. Fast, but prone to local minima.

## Control barrier functions: safety filters

A **control barrier function** $h(\mathbf x)$ is positive in the safe set, for example $h = \text{distance}^2 - d_{min}^2$. Requiring

$$\dot h(\mathbf x, \mathbf u) \ge -\alpha\, h(\mathbf x)$$

at every step ensures $h$ never becomes negative: the robot may approach the boundary but never cross it. A small optimisation finds the command closest to the nominal controller's command that satisfies the constraint. Because the nominal command can come from anything, a planner, a teleoperator or a learned policy, CBF **safety filters** are a leading way to add hard guarantees to learned control (Ames et al., 2019).

## Arms and manipulators

For arms the questions are self-collision and collision with the scene: planners check every motion against a collision model (see [[MoveIt2]], [[Collision_Detection]]), and torque-sensing cobots detect unexpected contact and stop or yield within milliseconds.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Collision_Avoidance]])
```
