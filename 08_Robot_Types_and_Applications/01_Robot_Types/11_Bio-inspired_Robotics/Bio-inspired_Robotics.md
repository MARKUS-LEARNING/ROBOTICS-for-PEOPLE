---
title: Bio-inspired Robotics
description: "Bio-inspired robotics borrows principles from animals and plants: how they move, sense, grip and control their bodies. Covers copying versus abstracting a principle, landmark robots (salamander, RoboBee, Octobot, legged robots), central pattern generators, a computed scaling example explaining why small fliers differ from large ones, and how learning has partly replaced hand-copied biology."
aliases:
  - Biomimetics
  - Bioinspired Robotics
  - Biomimetic Robots
tags:
  - bio-inspired-robotics
  - locomotion
  - soft-robotics
type: concept
level: beginner
subfield:
  - bio-inspired
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Robots]]"
related:
  - "[[Soft_Robotics]]"
  - "[[Legged_Robots]]"
  - "[[Drones]]"
  - "[[Swarm_Robots]]"
  - "[[Exoskeletons]]"
key_takeaways:
  - "The useful move is to abstract a principle from biology (compliance, passive dynamics, rhythmic pattern generators, distributed control), not to copy an animal's appearance."
  - "Scale changes the physics: weight grows with length cubed but wing or foot area with length squared, so insect-sized robots fly, stick and survive falls in ways large robots cannot."
  - "Since about 2020 much 'biological' behaviour, such as gaits and agile reflexes, emerges from reinforcement learning in simulation rather than from hand-copied neural circuits; biology still guides body design."
sources:
  - "Ijspeert, A. J., Crespi, A., Ryczko, D. and Cabelguen, J.-M. (2007). From swimming to walking with a salamander robot driven by a spinal cord model. https://www.science.org/doi/10.1126/science.1138353"
  - "Ma, K. Y., Chirarattananon, P., Fuller, S. B. and Wood, R. J. (2013). Controlled flight of a biologically inspired, insect-scale robot. https://www.science.org/doi/10.1126/science.1231806"
  - "Wehner, M. et al. (2016). An integrated design and fabrication strategy for entirely soft, autonomous robots. https://www.nature.com/articles/nature19100"
  - "Rus, D. and Tolley, M. T. (2015). Design, fabrication and control of soft robots. https://www.nature.com/articles/nature14543"
  - "Hwangbo, J. et al. (2019). Learning agile and dynamic motor skills for legged robots. https://arxiv.org/abs/1901.08652"
---

# Bio-inspired Robotics

**Bio-inspired robotics** designs robots using principles found in living things. Evolution has solved locomotion, grasping, sensing and control under tight energy budgets for hundreds of millions of years. Engineers borrow those solutions: legs that store energy in springy tendons, fish-like tails, insect wings, gecko-like adhesives, octopus-like soft arms.

> **Etymology.** Greek *bios*, "life". *Biomimetics*, from *mimesis*, "imitation", was coined by Otto Schmitt in the 1950s. *Bionics* (biology + electronics) dates from 1958 and gave its name to Festo's long-running Bionic Learning Network.

## Copy or abstract?

| Approach | Example | Lesson |
|---|---|---|
| **Copy the form** | Robotic fish that look like fish | Appealing, often inefficient |
| **Abstract the principle** | Velcro from burrs; compliant legs from tendons; rhythmic controllers from spinal circuits | Usually more useful |
| **Use robots to study biology** | Salamander robot testing a spinal-cord model (Ijspeert et al., 2007) | Robots as scientific instruments |

## Landmark robots

- **Salamander robot (2007)**: a spinal-cord-inspired central pattern generator switched between swimming and walking gaits, mirroring how a real salamander's nervous system might.
- **RoboBee (2013)**: an 80 mg insect-scale robot made the first controlled flight of its kind, using flapping wings driven by piezoelectric actuators (tethered for power).
- **Octobot (2016)**: the first entirely soft, autonomous robot, see [[Soft_Robotics]].
- **Legged robots**: cheetah-inspired and dog-like quadrupeds, with compliant legs that absorb impacts; see [[Legged_Robots]].
- **Gecko grippers**: dry adhesives with microscopic structures, used for perching drones and space-debris capture concepts.
- **Swarms**: ant- and bee-inspired coordination of many simple robots; see [[Swarm_Robots]].

## Central pattern generators

Many animals produce rhythmic movements (walking, swimming, breathing) with **central pattern generators**: neural circuits that oscillate on their own and are shaped by sensory feedback. In robots, coupled oscillators generate gait rhythms with few parameters, and a single "drive" signal can shift between gaits. CPGs remain popular in swimming and crawling robots. In legged robots they are often combined with, or replaced by, learned policies.

## Worked example: why size changes everything

Scale a creature or robot by a length factor $L$:
- mass and weight grow like $L^3$ (volume);
- wing area, foot area and muscle cross-section grow like $L^2$.

So **wing loading** (weight ÷ wing area) grows like $L^3/L^2 = L$. A robot 10× smaller has 1/10 the wing loading, which is why insect-scale robots can fly on tiny, inefficient wings while large aircraft need speed. The same ratio explains why:
- small animals and robots survive falls: impact energy per unit of supporting area is smaller;
- geckos and small robots can climb with adhesion, which scales with area, while their weight shrinks faster;
- large legged robots need disproportionately strong actuators.

## Biology and learning in 2026

Hand-copying biological controllers has partly given way to **learning**. Reinforcement learning in simulation discovers gaits, recovery reflexes and agile behaviours (see [[Reinforcement_Learning_for_Robots]]), and often reproduces animal-like features, such as gait transitions with speed, without being told to. Biology still shapes the hardware: compliant actuators, tendon-like elastic elements, soft skins, and sensor placement.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Bio-inspired_Robotics]])
```
