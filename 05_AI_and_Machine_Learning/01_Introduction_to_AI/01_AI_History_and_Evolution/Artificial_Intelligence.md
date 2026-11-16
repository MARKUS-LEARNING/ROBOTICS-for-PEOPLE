---
title: Artificial Intelligence
description: "Artificial intelligence is the effort to build machines that perceive, reason, learn and act. Covers what the term means, the main approaches (search and logic, probability, learning), a short history from Turing and Dartmouth to foundation models, how AI shows up inside a robot in 2026, and the limits that remain."
aliases:
  - AI
tags:
  - artificial-intelligence
  - machine-learning
  - robotics-history
  - foundation-models
type: overview
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
  - "[[Robots]]"
related:
  - "[[Machine_Learning]]"
  - "[[Agents_and_Environments]]"
  - "[[Embodied_AI]]"
  - "[[Artificial_Intelligence_vs_Machine_Learning_vs_Deep_Learning]]"
  - "[[Ethical_and_Social_Implications]]"
key_takeaways:
  - "AI is a goal (machines that behave intelligently), not one technique; search, logic, probability and learning are all AI."
  - "Since about 2012 learning from data, and since about 2020 large pretrained 'foundation' models, have driven most progress, now including robot control."
  - "A 2026 robot combines learned components (perception, language, policies) with classical ones (estimation, control, safety), because learned components still fail in ways that are hard to predict."
sources:
  - "Turing, A. M. (1950). Computing Machinery and Intelligence. https://academic.oup.com/mind/article/LIX/236/433/986238"
  - "McCarthy, J., Minsky, M., Rochester, N. and Shannon, C. (1955). A Proposal for the Dartmouth Summer Research Project on Artificial Intelligence. http://jmc.stanford.edu/articles/dartmouth/dartmouth.pdf"
  - "Russell, S. and Norvig, P. (2020). Artificial Intelligence: A Modern Approach, 4th edition. https://aima.cs.berkeley.edu/"
  - "SRI International (2026). Shakey the Robot. https://www.sri.com/hoi/shakey-the-robot/"
  - "LeCun, Y., Bengio, Y. and Hinton, G. (2015). Deep learning. https://www.nature.com/articles/nature14539"
  - "Stanford HAI (2026). The 2026 AI Index Report. https://hai.stanford.edu/ai-index/2026-ai-index-report"
---

# Artificial Intelligence

**Artificial intelligence (AI)** is the science and engineering of machines that do things we would call intelligent if a person did them: perceive, reason, plan, learn, communicate and act. For robotics, AI is what turns a programmable machine into one that can cope with situations its programmers did not foresee.

> **Etymology.** *Artificial* is from Latin *artificium*, "craftsmanship"; *intelligence* is from *intellegere*, "to choose between, to understand" (*inter* + *legere*, "to pick out"). John McCarthy coined *artificial intelligence* in the 1955 proposal for the Dartmouth summer workshop of 1956, which is usually taken as the field's founding.

## Four ways to define it

Russell and Norvig's textbook sorts definitions along two axes: thinking versus acting, and like humans versus rationally. Modern AI, and robotics in particular, mostly aims at **acting rationally**: an **agent** that perceives its environment and chooses actions that best achieve its goals given what it knows (see [[Agents_and_Environments]]). A robot is an agent with a body.

## The main approaches

| Approach | Idea | Robotics example |
|---|---|---|
| **Search and planning** | Explore possible action sequences to reach a goal | A* path planning, task planners |
| **Logic and knowledge** | Represent facts and rules; infer new facts | Early planners such as STRIPS; ontologies for tasks |
| **Probability** | Reason with uncertainty using probability theory | Kalman filters, particle filters, SLAM |
| **Machine learning** | Fit models to data instead of writing rules (see [[Machine_Learning]]) | Object detection, learned locomotion, VLAs |

These are complementary. A 2026 delivery robot plans paths with search, localises with probability, recognises people with learned vision, and may take instructions through a language model.

## A short history

| Years | Milestone |
|---|---|
| 1950 | Alan Turing asks "Can machines think?" and proposes the imitation game |
| 1956 | Dartmouth workshop names the field |
| 1966–72 | **Shakey** (SRI): the first mobile robot to reason about its actions; gives us A* search and STRIPS planning |
| 1970s–80s | Expert systems; "AI winters" when promises outrun results |
| 1986 | Backpropagation popularised for training neural networks |
| 1990s–2000s | Probabilistic robotics: Kalman and particle filters, SLAM; DARPA Grand Challenges (2004–07) |
| 2012 | Deep learning wins ImageNet; learned perception spreads |
| 2016 | AlphaGo combines deep learning with search and reinforcement learning |
| 2017 | The transformer architecture |
| 2020–23 | Large language models and vision-language models; "foundation models" |
| 2023–26 | Robot foundation models (vision-language-action models), world models, RL-trained humanoid locomotion |

## AI inside a robot (2026)

```
cameras, lidar, joints ──> perception (learned) ──> world state (probabilistic estimation)
language instruction ──> reasoning / task planning (language & vision-language models)
                         ──> skills: learned policies (VLA, RL) + classical planners (MoveIt, Nav2)
                         ──> control (classical: PID, MPC) ──> motors
                                    safety monitors (classical, verified) watch every layer
```

The learned parts give flexibility: new objects, new instructions, messy scenes. The classical parts give guarantees: stability, joint limits, emergency stops. See [[Embodied_AI]] and [[Vision-Language-Action_Models]].

## What is still hard

- **Reliability.** Learned systems can fail suddenly on inputs unlike their training data.
- **Common sense about the physical world.** Weight, friction, fragility and what happens next.
- **Long tasks.** Chaining many steps without compounding errors.
- **Data for robots.** Unlike text and images, robot experience is expensive to collect.
- **Verification and trust.** Proving that a learned controller is safe is an open research problem; see [[Ethical_and_Social_Implications]].

## Where to go next

[[Artificial_Intelligence_vs_Machine_Learning_vs_Deep_Learning]] → [[Machine_Learning]] → [[Deep_Learning]] → [[Embodied_AI]]

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Artificial_Intelligence]])
```
