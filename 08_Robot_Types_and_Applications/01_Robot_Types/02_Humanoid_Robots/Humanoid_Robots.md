---
title: Humanoid Robots
description: "Humanoid robots have a human-like body (legs, torso, arms, hands, head) so they can work in spaces and with tools built for people. Covers why the form is attempted, the subsystems, balance with a computed capture-point example, how humanoids are trained in 2026 (RL in simulation, motion tracking, teleoperation, VLAs), the main robots and their published specs (Atlas, Figure 03, NEO, Unitree G1, Digit), and the honest state of deployment."
aliases:
  - Humanoid
  - Humanoids
tags:
  - humanoid
  - legged-robot
  - locomotion
  - manipulation
  - robot-learning
type: robot-type
level: beginner
subfield:
  - legged-humanoid
status: keystone
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Robots]]"
  - "[[Legged_Robots]]"
related:
  - "[[Bipedal_Locomotion]]"
  - "[[Robot_Hands]]"
  - "[[Reinforcement_Learning_for_Robots]]"
  - "[[Vision-Language-Action_Models]]"
  - "[[Teleoperation_and_Data_Collection]]"
  - "[[Sim-to-Real_Transfer]]"
key_takeaways:
  - "The case for humanoids is the world they fit: stairs, doors, shelves and tools made for people, and human video and teleoperation as training data; the case against is cost, complexity and safety."
  - "Since 2023 humanoid locomotion is learned with reinforcement learning in GPU simulation, and whole-body skills increasingly come from tracking human motion and from vision-language-action models."
  - "In late 2026 humanoids work in early factory and logistics deployments and are taking first home orders; most published numbers are vendor claims, and reliability over long shifts is the open question."
sources:
  - "Boston Dynamics (2026). Atlas product page and specifications. https://bostondynamics.com/atlas/"
  - "Figure AI (2025). Introducing Figure 03. https://www.figure.ai/news/introducing-figure-03"
  - "1X Technologies (2026). NEO home robot specifications and ordering. https://www.1x.tech/neo"
  - "Unitree Robotics (2026). G1 humanoid specifications. https://www.unitree.com/g1"
  - "Radosavovic, I. et al. (2023). Real-World Humanoid Locomotion with Reinforcement Learning. https://arxiv.org/abs/2303.03381"
  - "He, T. et al. (2024). HOVER: Versatile Neural Whole-Body Controller for Humanoid Robots. https://arxiv.org/abs/2410.21229"
  - "NVIDIA (2025). GR00T N1: An Open Foundation Model for Generalist Humanoid Robots. https://arxiv.org/abs/2503.14734"
  - "Google DeepMind (2026). Gemini Robotics 2 brings whole body intelligence to robots. https://deepmind.google/blog/gemini-robotics-2-brings-whole-body-intelligence-to-robots/"
---

# Humanoid Robots

A **humanoid robot** has a body shaped roughly like a person's: two legs, a torso, two arms with hands, and a head carrying sensors. The appeal is simple. The world is built for human bodies, so a robot with one could in principle use the same stairs, doors, shelves, carts and tools without changing the environment.

> **Etymology.** Latin *humanus*, "human", + Greek *-oeidēs*, "having the form of". Early examples are Waseda University's WABOT-1 (1973) and Honda's ASIMO (2000). The current wave began around 2022–23, when learning-based control and investment in "general-purpose" robots arrived together.

## Why the human form, and why not

| For | Against |
|---|---|
| Fits human spaces: stairs, narrow aisles, standard shelving | Two legs are harder to keep upright than wheels; falls are costly |
| Uses human tools and fixtures | Many joints means more motors, cost and failure points |
| Human video and teleoperation map naturally onto the body, giving training data | A wheeled base with arms is often cheaper for the same job |
| People find it intuitive to work alongside | Being close to people raises safety and certification demands |

Many deployments in fact use **wheeled humanoids**, a humanoid upper body on a mobile base, where floors are flat.

## Subsystems

- **Actuation.** Mostly electric motors with gearboxes (quasi-direct-drive or harmonic). Hydraulics, as in the original Atlas, gave way to electric designs around 2024.
- **Hands.** From simple grippers to 20+ DoF dexterous hands with tactile sensing (see [[Robot_Hands]]).
- **Sensing.** Cameras (often 360°), depth, IMU, joint encoders and torque sensing, tactile skins.
- **Compute.** An onboard GPU module (for example NVIDIA Jetson Thor) running perception and policies, sometimes with cloud assistance.
- **Power.** Batteries of roughly 0.5–2 kWh, giving 2–4 h of runtime; some robots swap their own batteries.

## Balance, computed

A walking biped is like an inverted pendulum. In the linear inverted pendulum model, with the centre of mass at height $z$, the natural time constant is $T = \sqrt{z/g}$. For $z = 0.9$ m, $T = \sqrt{0.9/9.81} = 0.30$ s.

The **capture point**, where the robot must step to come to a stop, lies at $x_{cp} = x + \dot x\,T$ ahead of the centre of mass. Walking at 1.0 m/s, that is $1.0 \times 0.30 = 0.30$ m ahead; at the 1.4 m/s of brisk human walking it is 0.42 m. Push the robot faster than its legs can reach the capture point and it must take several steps or fall. This is why push recovery is about *stepping* strategy, and why learned controllers that react within tens of milliseconds made humanoids so much more robust. See [[Bipedal_Locomotion]].

## How humanoids learn to move and work (2026)

| Layer | Method | Examples |
|---|---|---|
| **Locomotion** | Reinforcement learning in GPU simulation with domain randomisation, transferred zero-shot to hardware | Radosavovic et al. 2023 (transformer policy), most commercial humanoids |
| **Whole-body motion** | Train a policy to *track* human motion-capture or video-derived motion; one controller serves many skills | HOVER (2024), BeyondMimic (2025), HumanPlus and OmniH2O (2024) |
| **Data collection** | Teleoperation with VR or motion capture; human video | See [[Teleoperation_and_Data_Collection]] |
| **Task intelligence** | Vision-language-action models for manipulation and instructions, sometimes controlling the whole body | Figure's Helix, NVIDIA GR00T N1.x, Gemini Robotics 2, 1X Redwood |
| **Safety** | Classical limits, monitors, safe stops beneath the learned layers | Required for work near people |

## The main robots (published specifications, late 2026)

| Robot | Maker | Published figures | Status (as stated by the maker) |
|---|---|---|---|
| **Atlas** (electric) | Boston Dynamics | 1.9 m, 90 kg, 56 DoF; lifts 50 kg instantly, 30 kg sustained; 4 h battery, swaps its own; IP67 | Product version for "a select number of early adopters"; first customer pilot with Hyundai |
| **Figure 03** | Figure AI | 9% less mass than Figure 02; 2 kW wireless charging; Helix VLA | Announced October 2025; BotQ line designed for up to 12,000 robots per year |
| **NEO** | 1X | 1.68 m (5′6″), 30 kg (66 lb), 22-DoF hands, 842 Wh, 4 h; Jetson Thor; Redwood AI | Home robot; $20,000 or $499 per month; US deliveries from 2026 |
| **G1** | Unitree | ~35 kg, 23–43 DoF, ~2 h battery | Sold from US$13.5K; widely used in research |
| **Digit** | Agility Robotics | Bipedal tote-handling robot | In commercial logistics use (GXO, Schaeffler, Toyota customers listed); Digit 5 in development |

Tesla (Optimus), Apptronik (Apollo), UBTech, Fourier, AgiBot and others are also building humanoids. Reports on their production volumes conflict, so treat counts you read online with care.

**Reading the specs.** All of these figures come from the companies themselves. Runtime depends heavily on the task: NEO's 842 Wh over 4 h implies an average draw of about 210 W, which is plausible for light household work but not for continuous heavy lifting.

## The honest state of play (November 2026)

- **Working:** robust walking on varied terrain, recovery from pushes, dynamic motions learned from human motion, and repetitive pick-and-place tasks in structured factory and warehouse cells.
- **Early:** multi-hour autonomous shifts without human help; general household tasks; dexterous manipulation of arbitrary objects; certified safety for close work with the public.
- **Open questions:** cost per useful hour compared with simpler robots, reliability (mean time between failures), and how much of the "generalist" capability shown in demonstrations holds up in daily operation.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Humanoid_Robots]])
```
