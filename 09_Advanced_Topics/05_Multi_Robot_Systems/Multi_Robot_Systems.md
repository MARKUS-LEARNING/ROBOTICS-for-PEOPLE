---
title: Multi-Robot Systems
description: "Section overview for systems of many robots: task allocation (with a computed example where greedy assignment costs twice the optimum), multi-agent path finding, coordination architectures from central fleet managers to decentralised swarms, communication, interoperability (Open-RMF), and multi-robot learning in 2026."
aliases:
  - Multi-Robot Coordination
  - Fleet Management
  - Multi-Agent Robotics
tags:
  - multi-robot-systems
  - path-planning
type: overview
level: advanced
subfield:
  - multi-robot
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Mobile_Robots]]"
  - "[[Path_Planning_Algorithms]]"
related:
  - "[[Swarm_Robots]]"
  - "[[Agents_and_Environments]]"
  - "[[Nav2_Navigation]]"
key_takeaways:
  - "Multi-robot problems add three questions to single-robot ones: who does which task, how robots avoid each other, and how they share information."
  - "Task allocation is an optimisation, not a greedy choice: in a three-robot example greedy assignment costs 12 while the optimum (Hungarian algorithm) costs 6."
  - "Most deployed fleets are centrally managed with learned and optimised traffic control; interoperability standards such as Open-RMF let robots from different vendors share buildings."
sources:
  - "Gerkey, B. P. and Matarić, M. J. (2004). A Formal Analysis and Taxonomy of Task Allocation in Multi-Robot Systems. https://journals.sagepub.com/doi/10.1177/0278364904045564"
  - "Open Robotics and contributors (2026). Open-RMF: robotics middleware framework for multi-fleet interoperability. https://github.com/open-rmf/rmf"
  - "Brambilla, M., Ferrante, E., Birattari, M. and Dorigo, M. (2013). Swarm robotics: a review from the swarm engineering perspective. https://link.springer.com/article/10.1007/s11721-012-0075-2"
  - "TechCrunch (2025). Amazon deploys its one millionth robot, releases generative AI model. https://techcrunch.com/2025/07/01/amazon-deploys-its-one-millionth-robot-releases-generative-ai-model/"
---

# Multi-Robot Systems

A **multi-robot system** uses several robots together: warehouse fleets, delivery robots sharing a building, drones mapping a forest, robot teams exploring a cave. More robots can do more work, cover more ground and survive failures. But they must decide who does what, avoid each other, and share what they know.

> **Etymology.** *Multi-* from Latin *multus*, "many". Research on multi-robot coordination grew in the 1990s; Gerkey and Matarić's 2004 taxonomy of task allocation is a standard reference.

## The three extra problems

| Problem | Question | Methods |
|---|---|---|
| **Task allocation** | Which robot does which task? | Optimal assignment (Hungarian algorithm), auctions and market methods, integer programming |
| **Multi-agent path finding (MAPF)** | How do many robots move without collisions or deadlocks? | Prioritised planning, conflict-based search, reservation tables, learned traffic models |
| **Information sharing** | How do robots build common maps and beliefs? | Multi-robot SLAM, consensus, map merging |

## Worked example: greedy versus optimal assignment

Three robots, three tasks; the table gives travel cost (minutes):

| | Task A | Task B | Task C |
|---|---|---|---|
| Robot 1 | 2 | 3 | 9 |
| Robot 2 | 1 | 8 | 9 |
| Robot 3 | 9 | 9 | 2 |

- **Greedy**: robot 1 takes its cheapest task, A (2); robot 2 is left with B or C and takes B (8); robot 3 takes C (2). Total **12**.
- **Optimal**: robot 1 → B (3), robot 2 → A (1), robot 3 → C (2). Total **6**.

The greedy choice that looked best for robot 1 cost the team double. The Hungarian algorithm finds the optimum in (n^3)$; auction methods reach similar results with decentralised bidding when robots cannot all talk to a centre.

## Architectures

| Architecture | Pros | Cons | Typical use |
|---|---|---|---|
| **Centralised** fleet manager | Near-optimal plans, simple to reason about | Single point of failure; communication needed | Warehouses, hospitals |
| **Decentralised** / swarm | Robust, scalable | Suboptimal, harder to guarantee | Exploration, swarms (see [[Swarm_Robots]]) |
| **Hybrid** | Central goals, local avoidance | Integration complexity | Most real fleets |

## In 2026

- **Large fleets**: warehouse operators run fleets of thousands of robots per site; Amazon described a generative model (DeepFleet) that predicts traffic to coordinate its robots.
- **Interoperability**: buildings host robots from many vendors (cleaning, delivery, security). **Open-RMF** provides shared traffic schedules, door and lift integration, and task dispatch across fleets.
- **Multi-robot learning**: multi-agent reinforcement learning trains decentralised policies; fleets also pool data so each robot learns from all (fleet learning).
- **Heterogeneous teams**: drones scout for ground robots; humanoids and mobile manipulators share work with fixed automation.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Multi_Robot_Systems]])
```
