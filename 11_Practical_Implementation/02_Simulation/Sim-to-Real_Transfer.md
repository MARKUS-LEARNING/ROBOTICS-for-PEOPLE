---
title: Sim-to-Real Transfer
description: "Making behaviour learned or tuned in simulation work on the real robot: the four gaps (looks, physics, actuation, task), domain randomisation, system identification, privileged teacher-student training, online adaptation, real-to-sim digital twins, and how to measure whether simulation predicts reality."
aliases:
  - Sim2Real
  - Sim-to-Real
  - Sim to Real
  - Reality Gap
  - Domain Randomization
  - Domain Randomisation
  - Real2Sim2Real
tags:
  - sim-to-real
  - simulation
  - reinforcement-learning
  - robot-learning
  - system-identification
type: method
level: advanced
subfield:
  - robot-learning
  - software
status: keystone
author: Jordan_Smith
date: 2026-11-15
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Robot_Simulation]]"
  - "[[Reinforcement_Learning_for_Robots]]"
related:
  - "[[Physics_Engines]]"
  - "[[GPU-Parallel_Simulation]]"
  - "[[Sim2Real_Experiments]]"
  - "[[World_Models]]"
  - "[[Transfer_Learning]]"
key_takeaways:
  - "Every sim-to-real failure traces to one of four gaps: how things look, how they behave physically, how the robot's own actuators respond, and how the task varies."
  - "Two complementary strategies: make the simulator match reality (system identification, real-to-sim) or make the policy robust to mismatch (randomisation, teacher-student, adaptation)."
  - "Transfer is measured, not assumed: success in simulation should predict success on the robot across many policies."
sources:
  - "Tobin, J. et al. (2017). Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World. https://arxiv.org/abs/1703.06907"
  - "Peng, X. B. et al. (2018). Sim-to-Real Transfer of Robotic Control with Dynamics Randomization. https://arxiv.org/abs/1710.06537"
  - "Hwangbo, J. et al. (2019). Learning Agile and Dynamic Motor Skills for Legged Robots. https://arxiv.org/abs/1901.08652"
  - "Lee, J. et al. (2020). Learning Quadrupedal Locomotion over Challenging Terrain. https://arxiv.org/abs/2010.11251"
  - "Kumar, A. et al. (2021). RMA: Rapid Motor Adaptation for Legged Robots. https://arxiv.org/abs/2107.04034"
  - "He, T. et al. (2025). ASAP: Aligning Simulation and Real-World Physics for Learning Agile Humanoid Whole-Body Skills. https://arxiv.org/abs/2502.01143"
  - "Zhao, W., Peña Queralta, J. and Westerlund, T. (2020). Sim-to-Real Transfer in Deep Reinforcement Learning for Robotics: a Survey. https://arxiv.org/abs/2009.13303"
  - "Google DeepMind (2026). MuJoCo 4: Studio & Filament, Deformables, Speculative Contacts. https://github.com/google-deepmind/mujoco/discussions/3773"
---

# Sim-to-Real Transfer

**Sim-to-real transfer** is getting a behaviour that works in simulation to work on the physical robot. It matters because simulation is where robots can practise millions of times safely and cheaply (see [[Reinforcement_Learning_for_Robots]]), but a simulator is never exactly the robot. The difference between them is called the **reality gap**, and sim-to-real is the craft of crossing it.

> **Etymology.** *Sim-to-real* is robotics shorthand from the mid-2010s, when deep reinforcement learning made the gap impossible to ignore; "sim2real" is the hashtag form. *Gap* is from Old Norse *gap*, "chasm". The phrase "reality gap" was used in evolutionary robotics in the 1990s by Nick Jakobi and colleagues, who evolved controllers in simulation and watched them fail on real robots.

## The four gaps

| Gap | What differs | How it is closed |
|---|---|---|
| **Looks** | Lighting, textures, camera pose, lens, motion blur | Randomise appearance; photoreal rendering; rebuild the scene from real images; train on real images too |
| **Physics** | Mass, friction, contact, deformable objects | Measure or estimate parameters; randomise them; better contact models (see [[Physics_Engines]]) |
| **Body (actuation)** | Motor dynamics, gear friction, backlash, latency, sensor noise | Model or learn the actuator; identify parameters from logs; randomise delays |
| **Task** | Object variety, layouts, clutter, people | Randomise layouts and objects; generate variants; collect diverse real data |

Every failure on the real robot can be traced to one of these. Diagnosing *which* one is the first step to fixing it.

## Two strategies, usually combined

**1. Make the simulator match reality.**
- **System identification**: run the real robot through test motions, log what happens, and fit the simulator's parameters (masses, friction, motor gains, delays) to reproduce it. MuJoCo 4.0 (November 2026) ships a system-identification toolbox, `mujoco.sysid`.
- **Actuator networks**: learn the real motor's response and put that model in the simulator (Hwangbo et al. 2019).
- **Real-to-sim**: build a digital twin of the real scene from photos or video (3D reconstruction, object scanning), so the simulation starts from the real world.
- **Learned residuals**: learn a correction from real data, for example a model of the difference between commanded and achieved motion. ASAP (He et al. 2025) learns such a "delta action" model from real humanoid motion and uses it to fine-tune the policy in simulation.

**2. Make the policy robust to mismatch.**
- **Domain randomisation**: vary appearance (Tobin et al. 2017) and dynamics (Peng et al. 2018) in training, so the real world looks like one more variation.
- **Teacher-student training**: a teacher policy learns with privileged information only a simulator has (exact terrain, friction); a student learns to imitate it from realistic sensors and history (Lee et al. 2020).
- **Online adaptation**: the policy infers the current dynamics from its recent observations and adjusts within moments (RMA, Kumar et al. 2021).
- **History and memory**: feeding a sequence of observations lets the policy implicitly identify the environment as it acts.

## The real-to-sim-to-real loop

```
 REAL ──(1) measure: scan the scene, log the robot──► SIM ──(2) train or tune──► POLICY
   ▲                                                   ▲                         │
   │                                                   └──(4) fix the gap found ─┤
   └──────────────────(3) deploy and measure success on the robot ◄──────────────┘
```

Real measurements build the simulator; real failures improve it. The loop has worked when **success in simulation predicts success on the robot**.

## Worked example: does the simulator predict reality?

Evaluate five policies (or checkpoints) on the same task, with enough trials each, in simulation and on the robot:

| Policy | Success in sim | Success on robot |
|---|---|---|
| A | 0.92 | 0.70 |
| B | 0.80 | 0.65 |
| C | 0.61 | 0.30 |
| D | 0.45 | 0.35 |
| E | 0.30 | 0.10 |

The Pearson correlation between the two columns is $r \approx 0.95$. Every policy does worse on the robot, but the simulator *ranks* them correctly, so it is useful for choosing between policies. Now suppose policy E scores 0.60 on the robot instead of 0.10: $r$ drops to about $0.45$. One policy that behaves very differently in reality is the clue that the simulator misses something E's behaviour depends on. Look at what E does differently, attribute it to one of the four gaps, and fix that.

## How much to randomise

Too little randomisation and the policy is brittle; too much and it becomes conservative (slow, cautious) or fails to learn at all. A practical rule is to randomise each parameter over the range it plausibly takes on the real robot, measured where possible: payload mass with and without a gripper attached, floor friction across the surfaces you will meet, latency as measured on the real control stack. Widen only the parameters that real failures point to.

## State of the art (as of November 2026)

- **Locomotion transfers routinely** with randomisation, teacher-student training and history-conditioned policies; humanoid whole-body skills are the active frontier.
- **Simulation is gaining deformables and fluids** (MuJoCo 4's implicit soft bodies and cloth; Newton's VBD and MPM solvers in Isaac Lab 3.0), widening what can be learned in simulation.
- **Real-to-sim pipelines** that build interactive scenes from phone video, and video [[World_Models]] that re-render simulated episodes realistically, attack the looks and task gaps.
- **Hybrid training** mixes simulated data with real demonstrations, so neither gap dominates.

## Reading list

- Tobin et al. 2017; Peng et al. 2018: domain randomisation for vision and dynamics.
- Lee et al. 2020; Kumar et al. 2021: teacher-student training and rapid adaptation.
- He et al. 2025 (ASAP): learning the residual between simulated and real humanoid motion.
- Zhao, Peña Queralta and Westerlund 2020: survey of sim-to-real methods.
- Hands-on: [[Sim2Real_Experiments]].

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Sim-to-Real_Transfer]])
```
