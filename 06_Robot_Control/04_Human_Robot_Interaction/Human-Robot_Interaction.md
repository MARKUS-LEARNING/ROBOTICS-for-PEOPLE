---
title: Human-Robot Interaction
description: "Human-robot interaction (HRI) studies how people and robots communicate, share space and work together. Covers physical and social interaction, proxemics with a computed approach-distance example, legibility and trust, interfaces (speech, gesture, screens, AR, teleoperation), how language models changed robot conversation in 2023–2026, and safety and evaluation."
aliases:
  - HRI
  - Human Robot Interaction
  - Social Robotics
tags:
  - human-robot-interaction
  - safety
  - natural-language-processing
type: overview
level: beginner
subfield:
  - human-robot-interaction
status: reviewed
author: Jordan_Smith
date: 2025-05-11
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Robots]]"
related:
  - "[[Collaborative_Robots]]"
  - "[[Natural_Language_Processing_Basics]]"
  - "[[Teleoperation_and_Data_Collection]]"
  - "[[Ethical_and_Social_Implications]]"
  - "[[Collision_Avoidance]]"
key_takeaways:
  - "HRI covers physical interaction (sharing space, touching, handing over) and social interaction (communication, expectations, trust); robots that work near people need both."
  - "Design around human expectations: comfortable distances, legible motion that signals intent, and clear feedback about what the robot will do next."
  - "Language models made natural conversation with robots routine by 2026, which raises new problems: grounding what the robot says in what it can do, and keeping instructions safe."
sources:
  - "Bartneck, C. et al. (2020). Human-Robot Interaction: An Introduction. https://www.human-robot-interaction.org/"
  - "Goodrich, M. A. and Schultz, A. C. (2007). Human-Robot Interaction: A Survey. https://www.nowpublishers.com/article/Details/HCI-005"
  - "Ahn, M. et al. (2022). Do As I Can, Not As I Say: Grounding Language in Robotic Affordances. https://arxiv.org/abs/2204.01691"
  - "Dragan, A. D., Lee, K. C. T. and Srinivasa, S. S. (2013). Legibility and predictability of robot motion. https://ieeexplore.ieee.org/document/6483603"
---

# Human-Robot Interaction

**Human-robot interaction (HRI)** is the study and design of how people and robots communicate, share space and work together. As robots leave fenced factory cells for warehouses, hospitals, sidewalks and homes, HRI decides whether people find them useful, trustworthy and safe, or confusing and unwelcome.

> **Etymology.** The field named itself in the early 2000s; the ACM/IEEE International Conference on Human-Robot Interaction began in 2006. It draws on robotics, psychology, design and human-computer interaction.

## Kinds of interaction

| Kind | Examples | Key concerns |
|---|---|---|
| **Physical** | Cobots working beside people, hand-overs, exoskeletons, assistive feeding | Force limits, contact safety (see [[Collaborative_Robots]]) |
| **Shared space** | Mobile robots in corridors, sidewalks, hospitals | Proxemics, right of way, legibility |
| **Social and communicative** | Receptionist, education and companion robots; voice assistants with bodies | Expectations, trust, turn-taking |
| **Remote** | Teleoperation, supervision of autonomous fleets | Latency, situational awareness (see [[Teleoperation_and_Data_Collection]]) |

## Proxemics: how close is comfortable?

Anthropologist Edward Hall described distance zones people use: intimate (under about 0.45 m), personal (0.45–1.2 m), social (1.2–3.6 m) and public (beyond 3.6 m). People generally expect robots to respect at least the personal zone unless invited closer.

**Worked example.** A robot approaching a seated person at 1 m/s should stop at the edge of the personal zone, 1.2 m away. With 0.5 s of reaction and processing delay and a comfortable deceleration of 1 m/s², it travels $1.0 \times 0.5 + 1.0^2 / (2 \times 1.0) = 1.0$ m while stopping. So it must begin stopping **2.2 m** from the person, and better still slow down earlier, since abrupt stops close to people feel aggressive.

## Legibility and trust

- **Legible motion** makes the robot's goal obvious early: reaching in a slightly exaggerated arc toward the object it intends to pick (Dragan et al., 2013), or a mobile robot turning its "head" and lights toward where it will go.
- **Calibrated trust**: people should trust the robot as much as it deserves, no more and no less. Explaining failures and showing uncertainty help.
- **Feedback**: lights, sounds, screens and speech saying what the robot is doing and about to do.

## Interfaces

Speech and natural language; gestures and gaze; touchscreens and apps; augmented reality showing planned paths; physical guidance (moving the robot by hand); teleoperation consoles.

## Language and HRI (2023–2026)

Language models turned "talking to a robot" from scripted commands into open conversation:
- people give instructions in everyday words;
- robots ask clarifying questions and explain what they are doing.

The challenges moved accordingly:
- **Grounding**: the robot must only promise what it can perceive and do (see [[Natural_Language_Processing_Basics]]).
- **Safety and authority**: who may instruct the robot, and which instructions it must refuse.
- **Anthropomorphism**: fluent speech in a humanoid body can make people overestimate the robot's abilities.

## Evaluating HRI

User studies with standardised questionnaires (perceived safety, usability, workload), task performance measures, field deployments over weeks rather than minutes, and attention to people beyond the direct user, such as bystanders and co-workers. See [[Ethical_and_Social_Implications]].

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Human-Robot_Interaction]])
```
