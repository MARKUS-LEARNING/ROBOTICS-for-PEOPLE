---
title: Drones
description: "Drones (uncrewed aerial vehicles) are flying robots: multirotors, fixed-wing and hybrid VTOL aircraft. Covers how a quadrotor flies and is controlled, a computed hover-power and endurance estimate, autonomy stacks (state estimation, mapping, planning), learned flight control (champion-level racing with RL, 2023), the main applications, and the regulatory picture in late 2026, including the still-pending US Part 108 rule for beyond-visual-line-of-sight flight."
aliases:
  - Drone
  - UAV
  - UAS
  - Quadrotor
  - Multirotor
tags:
  - aerial-robotics
  - control
  - reinforcement-learning
  - regulation
type: robot-type
level: beginner
subfield:
  - aerial-robotics
status: keystone
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Mobile_Robots]]"
  - "[[Control_Systems]]"
related:
  - "[[PID_Control]]"
  - "[[Reinforcement_Learning_for_Robots]]"
  - "[[SLAM]]"
  - "[[Swarm_Robots]]"
  - "[[Agricultural_Robots]]"
key_takeaways:
  - "A quadrotor controls its motion only through four rotor thrusts: tilting the thrust vector moves it sideways, and speeding up opposite pairs yaws it; cascaded PID loops (attitude inside position) are the classic controller."
  - "Hover power grows with weight to the power 1.5, so flight time is short: a 1.5 kg quadrotor with a 74 Wh battery hovers for roughly 25 minutes."
  - "Learning now matches or beats human pilots in narrow tasks (Swift, champion-level racing, 2023); regulation, not technology, mostly limits routine autonomous flight beyond line of sight."
sources:
  - "Mahony, R., Kumar, V. and Corke, P. (2012). Multirotor Aerial Vehicles: Modeling, Estimation, and Control of Quadrotor. https://ieeexplore.ieee.org/document/6289431"
  - "Kaufmann, E. et al. (2023). Champion-level drone racing using deep reinforcement learning. https://www.nature.com/articles/s41586-023-06419-4"
  - "Federal Aviation Administration (2025). Normalizing Unmanned Aircraft Systems Beyond Visual Line of Sight Operations (proposed rule, Part 108). https://www.federalregister.gov/documents/2025/08/07/2025-14992/normalizing-unmanned-aircraft-systems-beyond-visual-line-of-sight-operations"
  - "Unmanned Safety Institute (2026). Get 108 Ready (status guide, updated 4 November 2026). https://www.flyusi.org/guides/get-108-ready"
  - "EASA (2026). Civil drones. https://www.easa.europa.eu/en/domains/civil-drones"
---

# Drones

A **drone** is a flying robot: an uncrewed aircraft that flies under remote control or autonomously. Small multirotors have become the most widespread robots outside factories, used for photography, inspection, mapping, agriculture, delivery and, increasingly, warfare.

> **Etymology.** *Drone* is Old English *drān*, a male bee. Its aviation sense comes from the 1930s, when the UK's *Queen Bee* radio-controlled target aircraft inspired US "drones". Regulators say *UAS* (uncrewed aircraft system) or *UAV* (uncrewed aerial vehicle).

## Types

| Type | How it flies | Strengths | Limits |
|---|---|---|---|
| **Multirotor** (quad-, hexa-, octocopter) | Lift and control from fixed-pitch rotors | Hovers, takes off vertically, simple mechanics | Short endurance (20–45 min typical) |
| **Fixed-wing** | Wings for lift, propeller for thrust | Long range and endurance | Needs runway or launcher; cannot hover |
| **Hybrid VTOL** | Rotors to take off, wings to cruise | Range plus vertical take-off | Mechanically complex; used for delivery and mapping |

## How a quadrotor flies

A quadrotor has four rotors; diagonal pairs spin in opposite directions so their drag torques cancel. All control comes from varying the four thrusts:
- **Climb**: all four faster.
- **Roll and pitch**: speed up one side and slow the other. Tilting the total thrust vector is how the drone moves horizontally.
- **Yaw**: speed up the clockwise pair and slow the counter-clockwise pair; the unbalanced drag torque turns the body.

The classic controller is a **cascade**: an outer position loop computes the desired acceleration, which sets the desired tilt; an inner attitude loop, often [[PID_Control|PID]] at 500 Hz–8 kHz on the flight controller, tracks that tilt using the IMU. Open-source flight stacks (PX4, ArduPilot, Betaflight) implement these loops.

## Worked example: how long can it hover?

Momentum theory gives the ideal power to hover with total thrust $T$ (equal to the weight), air density $\rho$ and total rotor disc area $A$:

$$P_{ideal} = \frac{T^{3/2}}{\sqrt{2\rho A}}$$

For a 1.5 kg quadrotor ($T = 14.7$ N) with four 10-inch propellers (radius 0.127 m, so $A = 4\pi(0.127)^2 = 0.203$ m²) at sea level ($\rho = 1.225$ kg/m³):

$$P_{ideal} = \frac{14.7^{1.5}}{\sqrt{2 \times 1.225 \times 0.203}} \approx 80\ \text{W}.$$

Real rotors reach a "figure of merit" of about 0.6, so the actual power is about 80/0.6 ≈ **134 W**. A 4-cell (14.8 V) 5,000 mAh battery stores 74 Wh; using 80% of it gives 59.2 Wh, or about **27 minutes** of hover. Add a 0.5 kg payload and the power rises with weight to the 1.5 power, cutting endurance by roughly a third. Battery energy density is the main limit on multirotors.

## Autonomy

- **State estimation**: IMU + GNSS outdoors; visual-inertial odometry (camera + IMU) where GNSS fails, indoors or under bridges.
- **Mapping and avoidance**: stereo or depth cameras and lidar build local maps for obstacle avoidance (see [[SLAM]]).
- **Planning**: minimum-snap trajectories through waypoints; sampling-based or MPC planners in clutter.
- **Learned control**: in 2023 *Swift*, a quadrotor whose policy was trained with deep reinforcement learning in simulation and fine-tuned with real data, beat human world-champion pilots in head-to-head races (Nature). Learned policies now handle agile flight, perception-aware navigation and wind disturbance.
- **Swarms**: coordinated groups for light shows, mapping and search (see [[Swarm_Robots]]).

## Applications (2026)

Aerial photography and film; infrastructure inspection (power lines, bridges, wind turbines); surveying and mapping; precision agriculture (spraying, crop monitoring; see [[Agricultural_Robots]]); medical and parcel delivery; search and rescue; public safety. Military use, from reconnaissance to strike drones, expanded enormously in the 2020s and drives much of the technology and many of the ethical concerns (see [[Ethical_and_Social_Implications]]).

## Rules (as of November 2026)

- **United States.** Most commercial flights fall under Part 107, which requires a visual line of sight unless the operator has a waiver. The FAA proposed **Part 108**, a framework for routine beyond-visual-line-of-sight (BVLOS) operations, in August 2025. As of early November 2026 the final rule was still under White House review and not in effect.
- **European Union.** EASA's risk-based categories (open, specific, certified) apply, with the "specific" category and standard scenarios covering most BVLOS operations.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Drones]])
```
