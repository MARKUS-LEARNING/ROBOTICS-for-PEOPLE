---
title: Knowledge Representation and Reasoning
description: "How a robot stores what it knows and draws conclusions from it: logic, rules, ontologies, probabilistic models, planning languages (STRIPS, PDDL) and 3D scene graphs. Includes a worked STRIPS-style plan, and explains how 2026 systems combine language models (broad but unreliable knowledge) with explicit representations and planners (narrow but checkable)."
aliases:
  - KRR
  - Knowledge Representation
  - Task Planning
  - PDDL
  - Scene Graphs
tags:
  - artificial-intelligence
  - knowledge-representation
  - reasoning
  - decision-making
type: concept
level: intermediate
subfield:
  - planning
  - robot-learning
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Artificial_Intelligence]]"
  - "[[Agents_and_Environments]]"
related:
  - "[[Decision_Trees_and_Rule-Based_Systems]]"
  - "[[Natural_Language_Processing_Basics]]"
  - "[[Embodied_AI]]"
  - "[[SLAM]]"
key_takeaways:
  - "Knowledge representation chooses how facts are stored (logic, rules, graphs, probabilities); reasoning derives new facts or plans from them."
  - "Classical task planning describes actions by preconditions and effects (STRIPS, PDDL) so a planner can search for a sequence that reaches a goal, with guarantees that each step is valid."
  - "In 2026 language models supply broad common-sense knowledge and translate instructions, while explicit structures (3D scene graphs, PDDL domains, skill libraries) keep plans grounded and checkable."
sources:
  - "Fikes, R. E. and Nilsson, N. J. (1971). STRIPS: A new approach to the application of theorem proving to problem solving. https://www.sciencedirect.com/science/article/pii/0004370271900105"
  - "Russell, S. and Norvig, P. (2020). Artificial Intelligence: A Modern Approach, 4th edition, parts III and IV. https://aima.cs.berkeley.edu/"
  - "Hughes, N., Chang, Y. and Carlone, L. (2022). Hydra: A Real-time Spatial Perception System for 3D Scene Graph Construction and Optimization. https://arxiv.org/abs/2201.13360"
  - "Gu, Q. et al. (2023). ConceptGraphs: Open-Vocabulary 3D Scene Graphs for Perception and Planning. https://arxiv.org/abs/2309.16650"
  - "Liu, B. et al. (2023). LLM+P: Empowering Large Language Models with Optimal Planning Proficiency. https://arxiv.org/abs/2304.11477"
  - "Singh, I. et al. (2022). ProgPrompt: Generating Situated Robot Task Plans using Large Language Models. https://arxiv.org/abs/2209.11302"
---

# Knowledge Representation and Reasoning

A robot that tidies a kitchen must know that cups go in cupboards, that the cupboard must be open before a cup can go in, and where the cupboard is. **Knowledge representation** is the choice of how such facts are stored; **reasoning** is how new conclusions and plans are derived from them.

> **Etymology.** *Represent* is Latin *repraesentare*, "to make present again": a representation makes the world present inside the machine. *Reason* is from *ratio*, "reckoning, calculation".

## Ways to represent knowledge

| Representation | Stores | Reasoning | Robotics use |
|---|---|---|---|
| **Logic** (propositional, first-order) | Facts and general rules | Deduction | Safety rules, symbolic task conditions |
| **Rules / production systems** | If-then rules | Forward or backward chaining | Diagnostics, interlocks (see [[Decision_Trees_and_Rule-Based_Systems]]) |
| **Ontologies and knowledge graphs** | Classes, properties, relations ("a mug *is-a* container") | Classification, queries | Object and task knowledge (e.g. KnowRob, OWL) |
| **Probabilistic models** | Uncertain beliefs (Bayesian networks, filters) | Inference | Localisation, SLAM, fault diagnosis |
| **Planning domains** (STRIPS, PDDL) | Actions with preconditions and effects | Search for a valid plan | Task and mission planning |
| **3D scene graphs** | Objects, places, rooms and relations, anchored in a metric map | Graph queries + planning | "Bring the red mug from the kitchen" |

## Worked example: a STRIPS-style plan

Describe each action by **preconditions** (must be true) and **effects** (what becomes true or false):

| Action | Preconditions | Effects |
|---|---|---|
| `open(cupboard)` | at(cupboard), closed(cupboard), hand_empty | open(cupboard), ¬closed(cupboard) |
| `pick(cup)` | at(cup), hand_empty | holding(cup), ¬hand_empty |
| `place(cup, cupboard)` | at(cupboard), holding(cup), open(cupboard) | in(cup, cupboard), hand_empty, ¬holding(cup) |
| `goto(x)` | — | at(x), ¬at(previous) |

Start: the robot is at the table with the cup, the cupboard is closed, and its hand is empty. Goal: in(cup, cupboard).

A planner searches and finds:

`goto(cupboard)` → `open(cupboard)` → `goto(cup)` → `pick(cup)` → `goto(cupboard)` → `place(cup, cupboard)`.

Note what it worked out by itself: the cupboard must be opened *before* picking the cup, because opening needs an empty hand. That ordering came from the preconditions, not from a programmer. **PDDL** (1998 onward) is the standard language for writing such domains, and modern planners solve large ones in milliseconds.

## Reasoning under uncertainty

Real robots rarely know facts for certain. Probabilistic representations keep **beliefs**: "the cup is in the left cupboard with probability 0.7". Bayesian updating revises them as evidence arrives, the principle behind localisation and SLAM (see [[SLAM]]). Planning then becomes choosing actions with the best expected outcome, including information-gathering actions such as "look in the left cupboard first".

## Language models and explicit knowledge (2023–2026)

Large language models know an enormous amount of everyday knowledge: cups go in cupboards, eggs are fragile. But they can produce plans that are plausible yet invalid (skipping "open the cupboard"), and they do not know *this* kitchen. Current systems combine the two:

- **LLM → formal plan.** LLM+P (2023) translates a natural-language request into PDDL, lets a classical planner solve it, and translates the plan back, gaining the planner's correctness guarantees.
- **LLM → program over skills.** ProgPrompt and Code as Policies (2022) have the model write programs that call the robot's skills, with assertions that check preconditions.
- **3D scene graphs as memory.** Systems such as Hydra (2022) and ConceptGraphs (2023) build graphs of rooms, objects and relations from perception, with open-vocabulary labels from vision-language models. A language model can query the graph instead of guessing where things are.
- **Checking.** Explicit preconditions and simulators verify plans before execution.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Knowledge_Representation_and_Reasoning]])
```
