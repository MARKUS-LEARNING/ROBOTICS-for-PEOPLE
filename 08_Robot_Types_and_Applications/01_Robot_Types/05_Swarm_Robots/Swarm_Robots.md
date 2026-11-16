---
title: Swarm Robots
description: "Swarm robotics coordinates many simple robots through local rules and local communication, so useful group behaviour emerges with no central controller. Covers the principles, coordination mechanisms (flocking, stigmergy, consensus), landmark systems (Kilobots, TERMES, micro-drone swarms), a computed communication-scaling example, and how swarms differ from centrally managed fleets."
aliases:
  - Swarm Robotics
  - Robot Swarms
  - Swarm Intelligence
tags:
  - swarm-robotics
  - multi-robot-systems
  - bio-inspired-robotics
type: robot-type
level: intermediate
subfield:
  - multi-robot
  - bio-inspired
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Mobile_Robots]]"
related:
  - "[[Bio-inspired_Robotics]]"
  - "[[Drones]]"
  - "[[Modular_Robotics]]"
  - "[[Agents_and_Environments]]"
key_takeaways:
  - "A swarm has no leader: each robot follows simple rules using what it senses nearby, and group behaviour (flocking, aggregation, construction) emerges."
  - "Local interaction is what makes swarms scale: all-to-all communication grows with n², local-neighbour communication only with n."
  - "Most real deployments are centrally coordinated fleets (warehouses, drone light shows); fully decentralised swarms are proven in research, from 1,024 Kilobots (2014) to micro-drone swarms flying through forests (2022)."
sources:
  - "Brambilla, M., Ferrante, E., Birattari, M. and Dorigo, M. (2013). Swarm robotics: a review from the swarm engineering perspective. https://link.springer.com/article/10.1007/s11721-012-0075-2"
  - "Rubenstein, M., Cornejo, A. and Nagpal, R. (2014). Programmable self-assembly in a thousand-robot swarm. https://www.science.org/doi/10.1126/science.1254295"
  - "Werfel, J., Petersen, K. and Nagpal, R. (2014). Designing collective behavior in a termite-inspired robot construction team. https://www.science.org/doi/10.1126/science.1245842"
  - "Zhou, X. et al. (2022). Swarm of micro flying robots in the wild. https://www.science.org/doi/10.1126/scirobotics.abm5954"
---

# Swarm Robots

**Swarm robotics** studies how large numbers of relatively simple robots can achieve together what none could alone, by following local rules and interacting with neighbours, with no central controller. The inspiration is social insects: ants find short paths to food, termites build mounds, bees choose nest sites, all without a leader.

> **Etymology.** *Swarm* is from Old English *swearm*, a mass of bees. *Swarm intelligence* was coined by Gerardo Beni and Jing Wang in 1989 in the context of cellular robotic systems.

## Principles

| Principle | Meaning | Why it helps |
|---|---|---|
| **Decentralisation** | Each robot decides for itself | No single point of failure |
| **Locality** | Robots sense and talk only to neighbours | Scales to large numbers |
| **Simplicity** | Cheap individuals with simple rules | Low cost per unit; losses are tolerable |
| **Emergence** | Group behaviour arises from interactions | Complex outcomes from simple programs |
| **Redundancy** | Many interchangeable robots | Robust to failures |

## Coordination mechanisms

- **Flocking**: Reynolds's three rules (separation, alignment, cohesion, 1987) produce bird-like flocks.
- **Stigmergy**: indirect coordination through the environment, as termites do; robots react to what others have built or marked.
- **Consensus**: robots average their estimates or decisions with neighbours until they agree.
- **Pattern formation and self-assembly**: robots move to form shapes using only local distance measurements.
- **Learned policies**: multi-agent reinforcement learning trains decentralised policies in simulation.

## Worked example: why locality matters

If every robot must talk to every other robot, the number of messages per round is $n(n-1)$. If each robot talks only to about 6 nearest neighbours, it is about $6n$.

| Robots $n$ | All-to-all $n(n-1)$ | Local, 6 neighbours |
|---|---|---|
| 10 | 90 | 60 |
| 100 | 9,900 | 600 |
| 1,000 | 999,000 | 6,000 |

At 1,000 robots, all-to-all needs about 166 times more messages. Radio bandwidth and power run out long before. Local rules are not a limitation of swarm robotics: they are what makes it scale.

## Landmark systems

- **Kilobots (2014)**: 1,024 coin-sized robots self-assembled into programmed shapes using only infrared communication with neighbours.
- **TERMES (2014)**: termite-inspired robots built structures from blocks using local rules and stigmergy.
- **Micro-drone swarms (2022)**: palm-sized drones flew as a swarm through a dense forest, each planning with onboard sensing and limited communication.
- **Light shows**: hundreds to thousands of drones fly choreographed shows. These are centrally planned trajectories, not true swarms.

## Swarms versus fleets

Most commercial multi-robot systems, such as warehouse robots, delivery fleets and drone shows, are **centrally coordinated fleets**: a server assigns tasks and plans traffic (Amazon's DeepFleet model coordinates its warehouse robots, for example). True swarms trade optimality for robustness and scalability, and are used where central control is impossible or fragile: underwater, underground, in contested or disaster areas, and in space.

## Open problems

- **Designing local rules** that provably produce the desired global behaviour.
- **Real-world sensing** and communication limits.
- **Human supervision** of hundreds of robots.
- **Safety**: also the dual-use concerns of drone swarms (see [[Ethical_and_Social_Implications]]).

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Swarm_Robots]])
```
