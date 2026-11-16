---
title: Advanced Human-Robot Interaction (HRI)
description: "Section overview for advanced HRI: shared autonomy (blending human and robot control, with a computed arbitration example), intent and motion prediction, learning from human feedback and corrections, teleoperation for data collection, augmented-reality interfaces, and conversational robots built on language models in 2026."
aliases:
  - Advanced HRI
  - Shared Autonomy
  - Shared Control
tags:
  - human-robot-interaction
  - teleoperation
  - robot-learning
type: overview
level: advanced
subfield:
  - human-robot-interaction
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Human-Robot_Interaction]]"
related:
  - "[[Teleoperation_and_Data_Collection]]"
  - "[[Imitation_Learning]]"
  - "[[Natural_Language_Processing_Basics]]"
  - "[[Exoskeletons]]"
key_takeaways:
  - "Shared autonomy blends what the human commands with what the robot infers the human wants, giving more help when the robot is confident about the goal."
  - "With arbitration weight α tied to confidence, a 0.8-confident robot moving 0.3 m/s toward the inferred goal while the human commands 0.1 m/s sideways produces 0.24 m/s toward the goal and 0.02 m/s sideways."
  - "Human interaction is also a data source: corrections, preferences and teleoperated demonstrations train the policies that later need less help."
sources:
  - "Javdani, S., Srinivasa, S. S. and Bagnell, J. A. (2015). Shared Autonomy via Hindsight Optimization. https://arxiv.org/abs/1503.07619"
  - "Zhao, T. et al. (2023). Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware. https://arxiv.org/abs/2304.13705"
  - "Bartneck, C. et al. (2020). Human-Robot Interaction: An Introduction. https://www.human-robot-interaction.org/"
  - "Ahn, M. et al. (2022). Do As I Can, Not As I Say: Grounding Language in Robotic Affordances. https://arxiv.org/abs/2204.01691"
---

# Advanced Human-Robot Interaction (HRI)

The basics of HRI, such as proxemics, legibility and interfaces, are in [[Human-Robot_Interaction]]. This section covers the advanced topics where control, learning and interaction meet: robots that share control with people, predict what they will do, learn from them, and talk with them.

> **Etymology.** See [[Human-Robot_Interaction]]. *Shared autonomy* became a named research area in the 2010s, especially for assistive arms used by people with disabilities.

## Shared autonomy

Teleoperating a robot arm with a joystick is slow and tiring. In **shared autonomy**, the robot infers which goal the person is heading for (which cup, which button) and helps. A common form blends the two commands:

58088\mathbf u = \alpha\, \mathbf u_{robot} + (1 - \alpha)\, \mathbf u_{human},58088

with $\alpha$ rising as the robot becomes confident about the goal. Hindsight optimisation (Javdani et al., 2015) instead acts to help across *all* likely goals, without needing to commit to one.

**Worked example.** The robot is 80% confident ($\alpha = 0.8$) and would move at 0.3 m/s toward the inferred goal (the $ direction). The person pushes the joystick at 0.1 m/s sideways (the $ direction). Then

58088\mathbf u = 0.8\,(0.3, 0) + 0.2\,(0, 0.1) = (0.24,\ 0.02)\ \text{m/s}.58088

The robot mostly follows its inference, but the person's input still steers it. If the person keeps pushing sideways, the robot's belief shifts toward a goal in that direction and $\alpha$ adapts. Too much assistance frustrates people who feel their control taken away, so good systems keep the human clearly in charge.

## Predicting people

Robots in shared spaces predict where people will walk, what they will reach for, and when they will hand an object over, using motion forecasting (trajectory prediction networks), gaze, body pose and context. Prediction feeds planning (see [[Collision_Avoidance]]).

## Learning from people

- **Demonstrations**: teleoperation and kinesthetic teaching produce training data (see [[Imitation_Learning]], [[Teleoperation_and_Data_Collection]]).
- **Corrections**: a person nudging the arm during a task signals a better trajectory, and the robot updates.
- **Preferences and feedback**: comparisons ("this one was better") train reward models, as in reinforcement learning from human feedback for language models.
- **Language**: instructions and corrections in words ("a bit to the left, more gently").

## Interfaces at the frontier (2026)

- **VR and AR teleoperation** for data collection and remote work, often at scale across many operators.
- **Augmented reality** overlays showing the robot's planned path and intentions.
- **Conversational robots** using language models, with grounding in the robot's skills (see [[Natural_Language_Processing_Basics]]).
- **Physical human-robot interaction**: exoskeletons and assistive arms (see [[Exoskeletons]]).

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Advanced_Human_Robot_Interaction_(HRI)]])
```
