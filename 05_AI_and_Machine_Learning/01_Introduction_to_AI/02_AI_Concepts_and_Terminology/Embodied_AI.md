---
title: Embodied AI
description: "Intelligence that has a body: AI that perceives and acts in the physical world through sensors and actuators, and learns from the consequences. From Brooks's behaviour-based robots to 2026's vision-language-action models and world models."
aliases:
  - Physical AI
  - Embodied Intelligence
tags:
  - embodied-ai
  - artificial-intelligence
  - robot-learning
  - foundation-models
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
  - "[[Vision-Language-Action_Models]]"
  - "[[World_Models]]"
  - "[[Reinforcement_Learning_for_Robots]]"
  - "[[Robotics]]"
  - "[[Perception]]"
key_takeaways:
  - "Embodied AI learns and acts through a body, so intelligence is tested against physics, not just text."
  - "The idea runs from Brooks's 1980s behaviour-based robots to today's 'physical AI' foundation models."
  - "Its central difficulty is data: the physical world cannot be scraped like the web, so simulation, teleoperation and human video fill the gap."
sources:
  - "Brooks, R. (1991). Intelligence without representation. https://people.csail.mit.edu/brooks/papers/representation.pdf"
  - "Duan, J. et al. (2022). A Survey of Embodied AI: From Simulators to Research Tasks. https://arxiv.org/abs/2103.04918"
  - "Brohan, A. et al. (2023). RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control. https://arxiv.org/abs/2307.15818"
  - "Google DeepMind (2026). Gemini Robotics 2 brings whole body intelligence to robots. https://deepmind.google/blog/gemini-robotics-2-brings-whole-body-intelligence-to-robots/"
---

# Embodied AI

**Embodied AI** is artificial intelligence with a body. Instead of answering questions about the world from text and images, an embodied agent **perceives** the world through sensors, **acts** on it through motors, and learns from what its actions actually do. A robot that learns to open a drawer is embodied AI; a chatbot describing how to open one is not.

Since 2024 the industry often says **physical AI** for the same idea, especially for robots and autonomous machines driven by large learned models.

> **Etymology.** *Embody* is "em-" (in) + *body*, Old English *bodig*: to give a body to. In cognitive science, *embodied cognition* (1990s; Varela, Thompson and Rosch's *The Embodied Mind*, 1991) argued that thinking is shaped by having a body that moves and senses. Rodney Brooks's 1991 paper *Intelligence without representation* carried the argument into robotics.

## Why a body changes the problem

| Disembodied AI | Embodied AI |
|---|---|
| Learns from a fixed dataset | Generates its own data by acting |
| Mistakes cost a wrong answer | Mistakes can break things or hurt people |
| Time is not a constraint | Must act in real time (tens of milliseconds) |
| The world is text, images, numbers | The world has friction, weight, contact and noise |
| Data is abundant (the web) | Physical interaction data is scarce and expensive |

## A short history

- **1980s–90s, behaviour-based robotics.** Brooks's robots at MIT, such as the six-legged Genghis, walked using layers of simple reactive behaviours with no central world model: intelligence "without representation".
- **2010s, embodied AI in simulation.** Agents learned navigation and object search in 3D simulated homes (Habitat, AI2-THOR, iGibson); surveyed by Duan et al. (2022).
- **2023–26, foundation models get bodies.** [[Vision-Language-Action_Models]] such as RT-2 connected web-trained vision-language models to robot actions. By 2026, Google DeepMind's Gemini Robotics 2 controls whole humanoid bodies, and [[World_Models]] let agents imagine the consequences of actions.

## The perception-action loop

```
        ┌──────────── sense (cameras, joints, touch) ◄────────────┐
        ▼                                                          │
   think (policy, planner, world model) ──► act (motors) ──► the world changes
```

Everything in robotics sits somewhere on this loop. Embodied AI's claim is that learning *around the whole loop* (with the body in it) produces abilities that learning any one part in isolation cannot.

## Open questions

- **Data.** The physical world cannot be scraped. Simulation, teleoperation, handheld data-collection tools and human video each fill part of the gap (see [[Teleoperation_and_Data_Collection]]).
- **Safety.** A model that can act can do harm; benchmarks for physical safety, such as Google DeepMind's ASIMOV family, are an emerging research area.
- **Generalisation.** Do skills learned on one body transfer to another? Cross-embodiment training suggests partly yes.

## Reading list

- Brooks 1991, *Intelligence without representation*.
- Duan et al. 2022, *A Survey of Embodied AI: From Simulators to Research Tasks*.
- Brohan et al. 2023, *RT-2*.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Embodied_AI]])
```
