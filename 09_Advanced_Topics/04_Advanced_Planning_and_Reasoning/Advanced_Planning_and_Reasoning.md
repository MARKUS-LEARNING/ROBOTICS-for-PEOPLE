---
title: Advanced Planning and Reasoning
description: "Section overview for planning beyond paths: task and motion planning (TAMP), planning under uncertainty, long-horizon reasoning, and the 2023–2026 combination of language models with planners. Includes a computed example of why task plans explode combinatorially, and a map of the related notes."
aliases:
  - Task and Motion Planning
  - TAMP
  - Robot Reasoning
tags:
  - motion-planning
  - reasoning
  - artificial-intelligence
type: overview
level: advanced
subfield:
  - planning
  - robot-learning
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Path_Planning_Algorithms]]"
  - "[[Knowledge_Representation_and_Reasoning]]"
related:
  - "[[Optimal_Control]]"
  - "[[Formal_Methods_in_Control]]"
  - "[[Vision-Language-Action_Models]]"
  - "[[Active_Perception_and_Sensing]]"
key_takeaways:
  - "Real tasks need both symbolic decisions (what to do, in what order) and geometric ones (where to grasp, how to move); TAMP plans them together."
  - "The space explodes: five objects with four grasp choices each give 5! × 4⁵ = 122,880 combinations before any motion is planned, so planners prune with heuristics, sampling and learned guidance."
  - "Language models now propose plans and supply common sense, while TAMP solvers, simulators and checks verify feasibility."
sources:
  - "Garrett, C. R. et al. (2021). Integrated Task and Motion Planning. https://arxiv.org/abs/2010.01083"
  - "Garrett, C. R., Lozano-Pérez, T. and Kaelbling, L. P. (2018). PDDLStream: Integrating Symbolic Planners and Blackbox Samplers via Optimistic Adaptive Planning. https://arxiv.org/abs/1802.08705"
  - "Liu, B. et al. (2023). LLM+P: Empowering Large Language Models with Optimal Planning Proficiency. https://arxiv.org/abs/2304.11477"
  - "Ahn, M. et al. (2022). Do As I Can, Not As I Say: Grounding Language in Robotic Affordances. https://arxiv.org/abs/2204.01691"
---

# Advanced Planning and Reasoning

Path planning answers "how do I get from A to B?". Real tasks ask much more: "unload the dishwasher" means deciding the order, which grasp to use for each item, where each goes, what to move out of the way, and how to recover when a plate slips. This section covers planning and reasoning at that level.

> **Etymology.** See [[Knowledge_Representation_and_Reasoning]]. Task and motion planning as a named field dates from the early 2010s, when symbolic AI planning and geometric motion planning were joined.

## Task and motion planning (TAMP)

Symbolic planners (STRIPS, PDDL) choose action sequences but ignore geometry. Motion planners find collision-free motions but do not choose goals. **TAMP** couples them: a candidate symbolic plan ("pick cup, place cup in cupboard") is checked by sampling continuous parameters (grasps, placements, trajectories). If geometry fails, for example when the cupboard shelf is too full, the planner backtracks or adds actions such as "move bowl first". PDDLStream (2018) is a widely used framework.

### Worked example: why it explodes

Clearing a table of 5 objects, with 4 possible grasps per object:
- orderings: ! = 120$;
- grasp combinations: ^5 = 1{,}024$;
- together:  \times 1{,}024 = 122{,}880$ discrete combinations, before choosing placements or planning a single motion.

Exhaustive search is impossible for realistic tasks. Planners use heuristics, lazy evaluation (check geometry only for promising plans), learned guidance (predict which grasps will work) and, increasingly, language-model priors (sensible orderings).

## Planning under uncertainty

When the robot is unsure (an object hidden, a grasp may slip), plans must include sensing and contingencies: POMDPs and belief-space planning, information-gathering actions (see [[Active_Perception_and_Sensing]]), and re-planning when observations disagree with expectations.

## Language models and planning (2022–2026)

| Approach | Example |
|---|---|
| LLM proposes skills, weighted by feasibility | SayCan (2022) |
| LLM writes code calling skills | Code as Policies, ProgPrompt (2022) |
| LLM translates to PDDL, classical planner solves | LLM+P (2023) |
| Vision-language reasoning model supervises VLA policies | Hierarchical systems in 2025–26 (see [[Vision-Language-Action_Models]]) |

The emerging consensus: language models are good at proposing and explaining plans and poor at guaranteeing them; formal planners, simulators and preconditions check them (see [[Formal_Methods_in_Control]]).

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Advanced_Planning_and_Reasoning]])
```
