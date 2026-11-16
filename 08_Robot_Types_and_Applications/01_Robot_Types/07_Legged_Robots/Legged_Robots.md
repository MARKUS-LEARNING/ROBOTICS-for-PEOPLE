---
title: Legged Robots
description: "Robots that walk, run and climb on legs instead of wheels or tracks. Covers the types by leg count, static versus dynamic stability, gaits, actuation, a computed cost-of-transport example for a commercial quadruped, how learned controllers trained in simulation took over legged locomotion (2019–2026), the main robots, and what legs are good and bad at."
aliases:
  - Legged Robot
  - Quadruped
  - Quadrupeds
  - Walking Robots
tags:
  - legged-robot
  - locomotion
  - quadruped
  - bipedal
  - reinforcement-learning
type: robot-type
level: beginner
subfield:
  - legged-humanoid
  - mobile-robots
status: keystone
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Robots]]"
  - "[[Mobile_Robots]]"
related:
  - "[[Humanoid_Robots]]"
  - "[[Bipedal_Locomotion]]"
  - "[[Reinforcement_Learning_for_Robots]]"
  - "[[Sim-to-Real_Transfer]]"
  - "[[GPU-Parallel_Simulation]]"
  - "[[Gait]]"
key_takeaways:
  - "Legs trade efficiency on flat ground for access to rough terrain, stairs and gaps: a leg only needs footholds, not a continuous surface."
  - "Since about 2019, and universally by 2026, legged controllers are trained with reinforcement learning in massively parallel simulation and transferred zero-shot, often with a perception-based heightmap or depth input."
  - "Quadrupeds are mature commercial products (inspection, security, research); bipeds and humanoids are where the frontier is."
sources:
  - "Raibert, M. H. (1986). Legged Robots That Balance. https://mitpress.mit.edu/9780262181174/legged-robots-that-balance/"
  - "Hwangbo, J. et al. (2019). Learning agile and dynamic motor skills for legged robots. https://arxiv.org/abs/1901.08652"
  - "Lee, J. et al. (2020). Learning Quadrupedal Locomotion over Challenging Terrain. https://arxiv.org/abs/2010.11251"
  - "Rudin, N., Hoeller, D., Reist, P. and Hutter, M. (2021). Learning to Walk in Minutes Using Massively Parallel Deep Reinforcement Learning. https://arxiv.org/abs/2109.11978"
  - "Miki, T. et al. (2022). Learning robust perceptive locomotion for quadrupedal robots in the wild. https://arxiv.org/abs/2201.08117"
  - "Cheng, X., Shi, K., Agarwal, A. and Pathak, D. (2023). Extreme Parkour with Legged Robots. https://arxiv.org/abs/2309.14341"
  - "Boston Dynamics (2026). Spot specifications. https://bostondynamics.com/products/spot/"
---

# Legged Robots

A **legged robot** moves by placing feet on the ground one after another, as animals do. Wheels need a continuous surface; legs only need **footholds**. That makes legged robots the choice for stairs, rubble, forests, construction sites and industrial plants with gratings, pipes and steps.

> **Etymology.** *Leg* is from Old Norse *leggr*. Marc Raibert's hopping and running machines at Carnegie Mellon and MIT in the 1980s, described in *Legged Robots That Balance* (1986), showed that dynamic balance could be simple and robust. His lab became Boston Dynamics in 1992.

## Types by number of legs

| Legs | Stability | Examples |
|---|---|---|
| 1 (hoppers) | Only dynamic: must keep hopping | Raibert's 1980s hoppers |
| 2 (bipeds) | Mostly dynamic | [[Humanoid_Robots\|Humanoids]], Cassie |
| 4 (quadrupeds) | Static when walking slowly, dynamic when trotting or running | Spot, ANYmal, Unitree Go2 and B2, MIT Cheetah |
| 6 (hexapods) | Static: always a tripod of support | Research and inspection crawlers |

## Stability: static versus dynamic

- **Statically stable**: the centre of mass stays above the support polygon (the area spanned by the feet on the ground), so the robot could freeze mid-step without falling. This is slow but simple.
- **Dynamically stable**: the robot is always slightly falling and catches itself with the next step, like a running animal. This is faster and more efficient, but needs fast control (see [[Bipedal_Locomotion]] for the capture point).

Quadrupeds mostly **trot** (diagonal legs in pairs), and can walk, pace, bound or gallop. Gait choice trades speed, stability and energy (see [[Gait]]).

## Worked example: cost of transport

The dimensionless **cost of transport**, $\text{CoT} = P/(m g v)$, measures energy per unit weight per unit distance; lower is better.

Spot's published figures are 33.8 kg, a maximum speed of 1.6 m/s, a 564 Wh battery and an average runtime of 90 min. The average power is $564\ \text{Wh} / 1.5\ \text{h} = 376$ W, so at full speed

$$\text{CoT} \approx \frac{376}{33.8 \times 9.81 \times 1.6} \approx 0.71.$$

This is a rough upper bound: the 376 W average includes sensors and computers and periods of standing. Even so, it shows why legs cost energy. A wheeled robot on a smooth floor is typically several times more efficient. Use legs where wheels cannot go.

## Actuation

Modern legged robots use electric motors. **Quasi-direct-drive** actuators (a high-torque motor with a low gear ratio, popularised by the MIT Cheetah) let the leg feel and absorb impacts and allow torque control without separate force sensors. Hydraulics survive in very heavy machines. Series-elastic actuators add a spring for compliance and energy storage.

## How legged robots learn to walk (2019–2026)

1. **Model-based control** (2000s–2010s): simplified models (inverted pendulum, centroidal dynamics), footstep planning and [[Model_Predictive_Control_(MPC)|MPC]]. MIT Cheetah 3 and early ANYmal used this approach, and it is still used for its predictability.
2. **Learning in simulation** (2019): Hwangbo et al. trained ANYmal's controller with RL in simulation, with a learned model of the actuators, and transferred it directly.
3. **Robust and perceptive** (2020–22): a teacher policy with privileged information (terrain, friction) trains a student that infers them from proprioceptive history (Lee et al. 2020). Perception (heightmaps, depth) added rough-terrain hiking (Miki et al. 2022).
4. **Massive parallelism** (2021): thousands of simulated robots on one GPU learn to walk in minutes (Rudin et al. 2021; see [[GPU-Parallel_Simulation]]).
5. **Agility** (2023–26): parkour, jumping and climbing from depth cameras (Cheng et al. 2023), and loco-manipulation with arms or legs used as hands. The same methods now drive humanoids.

## The main robots (2026)

| Robot | Maker | Notes |
|---|---|---|
| **Spot** | Boston Dynamics | 33.8 kg, 1.6 m/s, IP54, 90 min average runtime; optional arm; widely used for industrial inspection |
| **ANYmal** | ANYbotics (ETH Zurich spin-off) | Inspection in hazardous industrial sites; explosion-proof version |
| **Go2, B2** | Unitree | Low-cost quadrupeds; Go2 is a common research and hobby platform |
| **Vision 60** | Ghost Robotics | Rugged outdoor and defence use |

## Strengths and limits

| Strengths | Limits |
|---|---|
| Stairs, rubble, gaps, steep slopes | Energy: shorter range than wheels |
| Can step over obstacles and recover from slips | Mechanical complexity, maintenance |
| Can carry sensors through spaces built for people | Payload is small relative to mass |
| Learned controllers are robust to pushes and terrain changes | Hard to certify learned controllers for safety |

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Legged_Robots]])
```
