---
title: Agents and Environments
description: "The agent–environment loop is the frame that connects AI, control and robotics: an agent perceives, decides and acts in an environment that responds. Covers the PEAS description of a task, the properties that make environments hard (partial observability, stochasticity, continuous time, other agents), agent architectures from reflexes to learning agents, a worked expected-utility decision, and 2026 agents that plan with language models."
aliases:
  - Intelligent Agents
  - Agent
  - Rational Agent
tags:
  - artificial-intelligence
  - autonomous-systems
  - decision-making
type: concept
level: beginner
subfield:
  - robot-learning
  - foundations
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Artificial_Intelligence]]"
related:
  - "[[Markov_Decision_Process]]"
  - "[[Reinforcement_Learning]]"
  - "[[Policy]]"
  - "[[Embodied_AI]]"
  - "[[Control_Systems]]"
key_takeaways:
  - "An agent maps its history of percepts to actions; a rational agent picks the action with the best expected outcome under its performance measure."
  - "What makes robotics hard is the environment: partially observable, stochastic, continuous, dynamic and shared with people."
  - "The same loop describes a thermostat, a PID controller, an RL policy and a language-model agent; 2026 robot agents stack them, with a slow reasoning agent calling fast learned and classical skills."
sources:
  - "Russell, S. and Norvig, P. (2020). Artificial Intelligence: A Modern Approach, 4th edition, chapter 2: Intelligent Agents. https://aima.cs.berkeley.edu/"
  - "Sutton, R. S. and Barto, A. G. (2018). Reinforcement Learning: An Introduction, 2nd edition. http://incompleteideas.net/book/the-book-2nd.html"
  - "Brooks, R. A. (1991). Intelligence without representation. https://people.csail.mit.edu/brooks/papers/representation.pdf"
  - "Ahn, M. et al. (2022). Do As I Can, Not As I Say: Grounding Language in Robotic Affordances. https://arxiv.org/abs/2204.01691"
  - "Yao, S. et al. (2022). ReAct: Synergizing Reasoning and Acting in Language Models. https://arxiv.org/abs/2210.03629"
---

# Agents and Environments

An **agent** is anything that perceives its environment through sensors and acts on it through actuators. The idea is the common frame of AI and robotics: a thermostat, a cruise controller, a chess program, a robot vacuum and a humanoid running a learned policy are all agents. What differs is how they choose actions and how hard their environments are.

> **Etymology.** Latin *agens*, "one who acts" (from *agere*, "to do, drive"). *Environment* is from Old French *environner*, "to surround". Russell and Norvig's textbook (1995 onwards) made the agent–environment view the organising idea of AI.

## The loop

```
          percept (sensor data)
   ┌──────────────────────────────┐
   │                              ▼
Environment                     Agent
   ▲                              │
   └──────────────────────────────┘
          action (motor command)
```

Formally, an agent is a function from the **history of percepts** to an **action**. A **rational agent** chooses the action that maximises the *expected* value of its performance measure, given what it has perceived and what it knows. Rational does not mean all-knowing: it means doing the best with the information available.

## Describing a task: PEAS

| | Robot vacuum | Warehouse picking arm |
|---|---|---|
| **Performance** | Area cleaned per charge, no collisions | Picks per hour, drops, damage |
| **Environment** | Rooms, furniture, pets, people | Bins of varied items, conveyor |
| **Actuators** | Wheels, brushes, suction | Arm joints, gripper or suction cup |
| **Sensors** | Bumpers, cliff sensors, lidar or camera, wheel encoders | Cameras, depth, force-torque, gripper sensors |

## What makes an environment hard

| Property | Easy end | Hard end (most robots) |
|---|---|---|
| Observability | Fully observable (chess board) | **Partially observable**: occlusion, sensor noise, hidden friction |
| Determinism | Deterministic | **Stochastic**: slips, wind, people |
| Episodes | Episodic (each decision independent) | **Sequential**: actions have long consequences |
| Change | Static | **Dynamic**: the world changes while you think |
| State and time | Discrete | **Continuous** |
| Other agents | Single agent | **Multi-agent**: people, other robots |

Partial observability is why robots keep beliefs (filters, maps, recurrent memory). Stochasticity is why they plan for expected outcomes. Dynamics are why fast reactive layers sit under slow deliberative ones.

## Agent architectures

| Architecture | Chooses actions by | Example |
|---|---|---|
| **Simple reflex** | Condition–action rules on the current percept | Bumper hit → reverse and turn |
| **Model-based reflex** | Rules plus an internal state tracking the unseen world | Obstacle avoidance using a local map |
| **Goal-based** | Search or planning toward a goal | A* path to the kitchen |
| **Utility-based** | Maximising expected utility over outcomes | Choosing a route trading time against risk |
| **Learning agent** | Improving any of the above from experience | RL locomotion policy, VLA fine-tuned on corrections |

Rodney Brooks's *behaviour-based* robotics (1991) argued that layered reflexes, tightly coupled to the world, beat slow symbolic reasoning for real robots. Modern robots combine both: reactive layers for safety and speed, deliberative layers for goals.

## Worked example: a rational choice depends on the utility

A delivery robot can take route A (90% chance of getting through, 60 s) or route B (99%, 90 s). Utility: +100 for a delivery, −0.5 per second.

- Ignoring failure cost: $U_A = 0.9(100) - 0.5(60) = 60$, $U_B = 0.99(100) - 0.5(90) = 54$, so **A** is better.
- If a failed attempt costs −200 (blocked, needs rescue): $U_A = 90 + 0.1(-200) - 30 = 40$, $U_B = 99 + 0.01(-200) - 45 = 52$, so **B** is better.

Same robot, same probabilities, opposite decision. Rational behaviour is only as good as the performance measure, which is why designing rewards and costs is a central, and risky, part of building agents (see [[Reinforcement_Learning]]).

## Agents in 2026: stacks of agents

The [[Markov_Decision_Process]] formalises the loop for learning agents, and [[Reinforcement_Learning]] trains them. Since 2022, **language-model agents** add a new top layer:
- SayCan (2022) let a language model propose skills, weighted by learned estimates of whether each skill could succeed;
- ReAct-style agents interleave reasoning with tool calls.

On robots, these "tools" are skills: navigate, pick, open door. A typical 2026 stack is a slow reasoning agent, then learned or classical skills, then fast controllers, with safety monitors able to override any layer (see [[Embodied_AI]]).

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Agents_and_Environments]])
```
