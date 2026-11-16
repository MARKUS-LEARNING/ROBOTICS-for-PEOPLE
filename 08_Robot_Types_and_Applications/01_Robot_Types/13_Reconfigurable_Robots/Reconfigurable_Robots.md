---
title: Reconfigurable Robots
description: "Reconfigurable robots change their own shape or structure to suit the task: modular robots that rearrange identical units, reconfigurable manufacturing cells, and robots with morphing bodies. Covers types, how reconfiguration is planned, a computed example of how fast the number of possible shapes grows, landmark systems (M-TRAN, M-Blocks, Roombots), and why practical use is still limited."
aliases:
  - Self-Reconfigurable Robots
  - Reconfigurable Robot
  - Morphing Robots
tags:
  - reconfigurability
  - modularity
  - robot-design
  - multi-robot-systems
type: concept
level: advanced
subfield:
  - modular-reconfigurable
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Modular_Robotics]]"
related:
  - "[[Swarm_Robots]]"
  - "[[Soft_Robotics]]"
  - "[[Bio-inspired_Robotics]]"
key_takeaways:
  - "A self-reconfigurable robot is many modules that can detach, move and reattach, so one system can become a snake, a walker or a manipulator."
  - "The space of possible configurations explodes combinatorially (a chain of 10 modules with 4 attachment choices per joint already has 262,144 variants), so planning reconfiguration is a hard search problem."
  - "After two decades of research, connectors, power and reliability still limit practical use; morphing bodies and reconfigurable factory cells are where the idea is most used in 2026."
sources:
  - "Yim, M. et al. (2007). Modular Self-Reconfigurable Robot Systems [Grand Challenges of Robotics]. https://ieeexplore.ieee.org/document/4141037"
  - "Seo, J., Paik, J. and Yim, M. (2019). Modular Reconfigurable Robotics. https://www.annualreviews.org/doi/10.1146/annurev-control-053018-023834"
  - "MIT News (2013). Surprisingly simple scheme for self-assembling robots (M-Blocks). https://news.mit.edu/2013/simple-scheme-for-self-assembling-robots-1004"
  - "Brambilla, M., Ferrante, E., Birattari, M. and Dorigo, M. (2013). Swarm robotics: a review from the swarm engineering perspective. https://link.springer.com/article/10.1007/s11721-012-0075-2"
---

# Reconfigurable Robots

A **reconfigurable robot** can change its own structure: rearrange its parts, change its shape, or reorganise how it is assembled, to suit a new task or terrain. The ambition is one robot that can crawl through a pipe as a snake, cross rubble as a walker, and then become an arm to turn a valve.

> **Etymology.** Latin *re-*, "again", + *configurare*, "to shape after a pattern". Toshio Fukuda's CEBOT (cellular robot, 1988) is usually credited as the first self-reconfigurable robot concept.

## Types

| Type | What changes | Examples |
|---|---|---|
| **Lattice** | Modules move between positions on a grid, like pixels | M-Blocks (cubes that pivot using internal flywheels, MIT 2013) |
| **Chain** | Modules form chains and limbs that bend | PolyBot, CKBot |
| **Hybrid** | Both lattice and chain moves | M-TRAN, SMORES, Roombots (furniture-forming modules) |
| **Morphing bodies** | One robot changes shape without detaching parts | Wheel-leg hybrids, drones with folding arms, soft morphing robots |
| **Reconfigurable manufacturing** | A production cell's machines and robots are rearranged for new products | Flexible factory cells |

## Worked example: why planning reconfiguration is hard

Consider a chain of $n$ modules where each new module can attach to the previous one in 4 orientations. The number of distinct chains is $4^{\,n-1}$:
- 3 modules: 16;
- 10 modules: $4^9 = 262{,}144$;
- 20 modules: about $2.7 \times 10^{11}$.

That counts only chains. Branching shapes are far more numerous. Planning a sequence of module moves from one shape to another means searching this enormous space while keeping the structure connected and stable. Planners use hierarchical decompositions, lattice-specific rules, and increasingly learned heuristics.

## Why it has not taken over

- **Connectors** must be strong, precise, quick and reliable through thousands of cycles; they are the weak point.
- **Power and communication** must pass through every joint.
- **Cost per module**, and **performance compared with purpose-built robots** for any single task.
- **Control**: every configuration needs its own locomotion or manipulation controller. Learning in simulation can now train controllers per morphology, or one controller for many morphologies, which addresses part of this.

## Where the idea lives in 2026

Robots with morphing bodies (transforming wheel-legs, foldable drones), reconfigurable industrial cells, and research systems that co-design body and controller in simulation. The original vision of a general-purpose shape-shifting robot remains a research goal.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Reconfigurable_Robots]])
```
