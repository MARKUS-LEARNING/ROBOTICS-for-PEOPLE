---
title: Reinforcement Learning for Robots
description: "How reinforcement learning actually reaches real robots: training in massively parallel simulation, domain randomisation, teacher-student distillation, rapid adaptation, and RL on hardware. Why locomotion became RL's big success and where manipulation still needs other methods."
tags:
  - reinforcement-learning
  - robot-learning
  - sim-to-real
  - locomotion
  - legged-robot
  - deep-learning
type: method
level: intermediate
subfield:
  - robot-learning
  - legged-humanoid
  - control
status: keystone
author: Jordan_Smith
date: 2025-04-28
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Reinforcement_Learning]]"
  - "[[Neural_Networks]]"
  - "[[Robot_Simulation]]"
related:
  - "[[Policy]]"
  - "[[Imitation_Learning]]"
  - "[[Locomotion]]"
  - "[[Legged_Robots]]"
  - "[[World_Models]]"
  - "[[Sim-to-Real_Transfer]]"
  - "[[Sim2Real_Experiments]]"
key_takeaways:
  - "Robot RL is mostly trained in simulation: thousands of parallel robots on a GPU, then transferred to hardware."
  - "The transfer works because of domain randomisation, privileged teacher policies distilled into realistic students, and online adaptation to the real dynamics."
  - "Locomotion is RL's clearest success; contact-rich manipulation increasingly combines imitation pretraining with RL fine-tuning."
sources:
  - "Hwangbo, J. et al. (2019). Learning Agile and Dynamic Motor Skills for Legged Robots. https://arxiv.org/abs/1901.08652"
  - "Lee, J. et al. (2020). Learning Quadrupedal Locomotion over Challenging Terrain. https://arxiv.org/abs/2010.11251"
  - "Kumar, A. et al. (2021). RMA: Rapid Motor Adaptation for Legged Robots. https://arxiv.org/abs/2107.04034"
  - "Rudin, N. et al. (2021). Learning to Walk in Minutes Using Massively Parallel Deep Reinforcement Learning. https://arxiv.org/abs/2109.11978"
  - "Radosavovic, I. et al. (2023). Real-World Humanoid Locomotion with Reinforcement Learning. https://arxiv.org/abs/2303.03381"
  - "Tobin, J. et al. (2017). Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World. https://arxiv.org/abs/1703.06907"
  - "OpenAI et al. (2019). Solving Rubik's Cube with a Robot Hand. https://arxiv.org/abs/1910.07113"
  - "Kaufmann, E. et al. (2023). Champion-level drone racing using deep reinforcement learning. https://www.nature.com/articles/s41586-023-06419-4"
  - "Kalashnikov, D. et al. (2018). QT-Opt: Scalable Deep Reinforcement Learning for Vision-Based Robotic Manipulation. https://arxiv.org/abs/1806.10293"
  - "NVIDIA, Mittal, M. et al. (2025). Isaac Lab: A GPU-Accelerated Simulation Framework for Multi-Modal Robot Learning. https://arxiv.org/abs/2511.04831"
  - "NVIDIA (2026). Isaac Lab 3.0 release notes. https://github.com/isaac-sim/IsaacLab/releases/tag/v3.0.0"
---

# Reinforcement Learning for Robots

[[Reinforcement_Learning]] promises robots that learn skills nobody knows how to program: running over rubble, catching themselves after a shove, spinning an object in the fingers. The catch is experience. RL needs millions of attempts, and a real robot that tries a million times will break itself, its surroundings and its operator's patience. Robot RL is therefore mostly the craft of **learning in simulation and arriving on hardware intact**.

> **Etymology.** *Robot* comes from Czech *robota*, "forced labour", coined for Karel Čapek's 1920 play *R.U.R.* (the word was his brother Josef's suggestion). A century later, robots trained by reinforcement learn their labour the way Čapek's could not: by being rewarded for doing it well.

## The standard pipeline

```
 design reward + randomisation ─► train in GPU simulation (PPO, 1000s of robots) ─► distil / adapt ─► deploy on hardware
          ▲                                                                                          │
          └──────────────────────── watch failures on the real robot, fix the simulation ◄───────────┘
```

**1. Write the task as an MDP.** Observations (joint angles and velocities, base orientation, previous action, optionally height maps or camera images), actions (usually joint position targets for a PD controller running at 500–1000 Hz) and a reward made of many weighted terms: track the commanded velocity, keep the body level, penalise energy, foot slip and jerky actions.

**2. Train massively in parallel.** On one GPU, simulators such as Isaac Lab or MuJoCo Playground step thousands of robots at once. Rudin et al. (2021) showed a quadruped learning to walk in minutes this way. PPO is the usual algorithm because its updates are stable and it scales with batch size.

**3. Close the sim-to-real gap.** A simulator is never exactly the robot. The main tools:

| Technique | Idea | Landmark |
|---|---|---|
| **Domain randomisation** | Vary masses, friction, motor strength, latency, terrain and sensor noise in training, so the real robot looks like one more variation | Tobin et al. 2017; OpenAI's Rubik's Cube hand, 2019 |
| **Actuator modelling** | Learn the real motor's response from logged data and put it in the simulator | Hwangbo et al. 2019 (ANYmal) |
| **Teacher-student distillation** | A teacher trained with privileged information (exact terrain, friction) is imitated by a student that only sees real sensors | Lee et al. 2020 |
| **Rapid adaptation** | The policy infers the current dynamics from recent history and adjusts within a fraction of a second | Kumar et al. 2021 (RMA) |
| **System identification** | Measure the real robot's parameters and calibrate the simulator before training | Classical; see [[Sim-to-Real_Transfer]] |

**4. Deploy and iterate.** Watch where the real robot fails, attribute the failure to a gap (physics, sensing, actuation, latency), fix the simulator, retrain.

## Worked example: what a locomotion reward looks like

A velocity-tracking reward for a quadruped, of the kind used in Isaac Lab's locomotion tasks, sums terms like these each control step:

| Term | Expression (sketch) | Weight sign |
|---|---|---|
| Track commanded forward/sideways speed | $\exp(-\lVert v^{cmd}_{xy} - v_{xy} \rVert^2 / 0.25)$ | + |
| Track commanded turning rate | $\exp(-(\omega^{cmd}_z - \omega_z)^2 / 0.25)$ | + |
| Keep the body from bouncing | $v_z^2$ | − |
| Low joint torques (energy) | $\lVert \tau \rVert^2$ | − |
| Smooth actions | $\lVert a_t - a_{t-1} \rVert^2$ | − |
| Feet in the air long enough (a real gait) | air time per footstep | + |
| Body or thighs touch the ground | contact indicator | − |

No term says "trot". The gait emerges because it is the cheapest way to satisfy all of them at once, and changing a single weight can change the gait. This is why reward design is still skilled work.

## Where RL works, and where it struggles

| Domain | Status (late 2026) | Why |
|---|---|---|
| **Legged and humanoid locomotion** | Routine. Sim-trained policies run on commercial quadrupeds and humanoids | Contact is mostly between feet and ground, simulators model it well, and the reward is easy to state |
| **Agile flight** | Champion-level drone racing has been demonstrated | Dynamics are well modelled; the reward is lap time |
| **In-hand dexterity** | Demonstrated, with heavy randomisation | Many contacts and fingers are hard to simulate accurately |
| **Everyday manipulation** | Mostly [[Imitation_Learning]] first, RL to refine | Rewards are hard to write for "fold the shirt", and soft objects are hard to simulate |

On real hardware, RL is used where samples are cheap enough: QT-Opt (Kalashnikov et al. 2018) learned grasping from 580,000 real attempts on a fleet of arms. More recently, RL is increasingly used *after* imitation: a policy that already roughly works is improved from its own successes and failures, on the real robot or in simulation.

## Humanoids

Radosavovic et al. (2023) trained a transformer policy on a history of observations entirely in simulation and deployed it zero-shot on the Digit humanoid outdoors; it adapted its gait to terrain from context alone. By 2026 that recipe (large-scale simulation, randomisation, a history-conditioned policy) is the default starting point for humanoid walking, and research has moved to whole-body skills that combine locomotion with manipulation.

## State of the art (as of November 2026)

- **Simulation platforms.** Isaac Lab 3.0 (generally available 20 October 2026) offers one task API over several physics backends, including PhysX and the open-source Newton engine with its MuJoCo-Warp, VBD and MPM solvers. Isaac Sim is now optional. MuJoCo 4.0 and MuJoCo Playground 1.0 (November 2026) target the same GPU-parallel training.
- **Deformables and fluids enter the loop.** Newton's VBD and MPM solvers bring cloth, cables and liquids into GPU-parallel RL, the kind of object that kept manipulation out of reach.
- **RL as fine-tuning.** Generalist [[Vision-Language-Action_Models]] are trained by imitation, then improved with reward, mirroring the pretrain-then-RL recipe of language models.
- **Learned simulators.** Training inside video [[World_Models]] is a research direction to watch, not yet a production tool.

## Tooling

| Tool | Role |
|---|---|
| Isaac Lab 3.0 | GPU-parallel robot learning framework (BSD-3), RSL-RL and other RL libraries |
| MuJoCo Playground | GPU-parallel environments on MuJoCo / MJX / MuJoCo Warp |
| Genesis | GPU multi-physics simulator for robot learning |
| RSL-RL | Fast PPO used for most locomotion work |
| LeRobot | Hugging Face library spanning imitation and RL on real hardware |

## Reading list

- Hwangbo et al. 2019, *Learning Agile and Dynamic Motor Skills for Legged Robots*.
- Lee et al. 2020, *Learning Quadrupedal Locomotion over Challenging Terrain*.
- Kumar et al. 2021, *RMA: Rapid Motor Adaptation for Legged Robots*.
- Rudin et al. 2021, *Learning to Walk in Minutes Using Massively Parallel Deep RL*.
- Radosavovic et al. 2023, *Real-World Humanoid Locomotion with Reinforcement Learning*.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Reinforcement_Learning_for_Robots]])
```
