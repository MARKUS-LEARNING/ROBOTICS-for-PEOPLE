---
title: Bio-Inspired and Biomimetic Robotics
description: "Section overview for robotics that learns from biology: locomotion, sensing, materials, control and collective behaviour. Includes a computed Froude-number example that predicts when walking animals and robots switch to running, a map of related notes, and the 2026 frontier: biohybrids, neuromorphic computing, and learning that rediscovers animal behaviour."
aliases:
  - Biomimetic Robotics
  - Biorobotics
tags:
  - bio-inspired-robotics
  - locomotion
type: overview
level: intermediate
subfield:
  - bio-inspired
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Bio-inspired_Robotics]]"
related:
  - "[[Soft_Robotics]]"
  - "[[Swarm_Robots]]"
  - "[[Biomaterials]]"
  - "[[Event_Based_Sensing_and_Processing]]"
  - "[[Legged_Robots]]"
key_takeaways:
  - "Biology offers tested solutions in five areas: moving, sensing, building (materials and morphology), controlling, and cooperating."
  - "The Froude number v²/(gL) predicts gait changes across animals of different size; walk-to-run happens near 0.5, which for 0.5 m legs is about 1.6 m/s."
  - "The 2026 frontier includes biohybrid robots, neuromorphic sensing and computing, and reinforcement learning that reproduces animal-like gaits without being told to."
sources:
  - "Ijspeert, A. J., Crespi, A., Ryczko, D. and Cabelguen, J.-M. (2007). From swimming to walking with a salamander robot driven by a spinal cord model. https://www.science.org/doi/10.1126/science.1138353"
  - "Webster-Wood, V. A. et al. (2023). Biohybrid robots: recent progress, challenges, and perspectives. https://iopscience.iop.org/article/10.1088/1748-3190/ac9c3b"
  - "Gallego, G. et al. (2019). Event-based Vision: A Survey. https://arxiv.org/abs/1904.08405"
  - "Rus, D. and Tolley, M. T. (2015). Design, fabrication and control of soft robots. https://www.nature.com/articles/nature14543"
---

# Bio-Inspired and Biomimetic Robotics

This section collects robotics that borrows from living things. The principles and landmark robots are in [[Bio-inspired_Robotics]]; this overview maps the related notes and the frontier.

> **Etymology.** See [[Bio-inspired_Robotics]]. *Biomimetic* (imitating life) usually means closer copying; *bio-inspired* means borrowing principles.

## Five areas of inspiration

| Area | Biological model | Robotics | Notes |
|---|---|---|---|
| Moving | Legs, fins, wings, wriggling | Legged, swimming, flying, snake robots | [[Legged_Robots]], [[Drones]] |
| Sensing | Eyes, whiskers, skin, lateral lines | Event cameras, whisker sensors, tactile skins | [[Event_Based_Sensing_and_Processing]], [[Robot_Hands]] |
| Building | Muscles, tendons, soft bodies, materials | Soft robots, artificial muscles, biomaterials | [[Soft_Robotics]], [[Electroactive_Polymers]], [[Biomaterials]] |
| Controlling | Central pattern generators, reflexes, learning | CPGs, reflex layers, RL | [[Bio-inspired_Robotics]] |
| Cooperating | Ant colonies, flocks, bee swarms | Swarms, multi-robot coordination | [[Swarm_Robots]], [[Multi_Robot_Systems]] |

## Worked example: when to switch from walking to running

The **Froude number** $\mathrm{Fr} = v^2/(gL)$ compares speed with leg length $. Animals of very different sizes change gait at similar Froude numbers; walking gives way to running near $\mathrm{Fr} \approx 0.5$, a result popularised by zoologist R. McNeill Alexander. For a robot or animal with 0.5 m legs:

58113v = \sqrt{0.5 \times 9.81 \times 0.5} \approx 1.57\ \text{m/s}.58113

Above this, an inverted-pendulum walking gait becomes inefficient, or physically impossible, because gravity can no longer hold the body on the stance leg, so running is better. Engineers use the same scaling to compare robots of different sizes and to choose gaits.

## The frontier in 2026

- **Biohybrid robots** that use living muscle cells or microorganisms as actuators (see [[Biomaterials]]).
- **Neuromorphic computing and sensing**: event cameras and spiking networks on low-power chips.
- **Learning that rediscovers biology**: RL in simulation reproduces gait transitions and reflex-like recoveries with no explicit biological model; comparing learned and animal behaviour is a growing research theme.
- **Robots as scientific instruments** for biology: robotic models test hypotheses about how animals move and sense.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Bio_Inspired_and_Biomimetic_Robotics]])
```
