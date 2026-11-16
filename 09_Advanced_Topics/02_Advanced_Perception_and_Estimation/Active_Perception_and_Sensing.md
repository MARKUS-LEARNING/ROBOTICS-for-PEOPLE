---
title: Active Perception and Sensing
description: "Active perception chooses where to look, move or touch in order to perceive better, instead of passively processing whatever sensors receive. Covers the idea, information gain with a computed entropy example, next-best-view planning for 3D reconstruction, active search and interactive perception (pushing and probing), and how learned policies and vision-language models decide where to look."
aliases:
  - Active Perception
  - Active Vision
  - Next-Best-View
  - Interactive Perception
tags:
  - perception
  - computer-vision
  - uncertainty
type: concept
level: advanced
subfield:
  - perception
  - planning
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Probabilistic_Robotics]]"
related:
  - "[[Semantic_Scene_Understanding]]"
  - "[[Knowledge_Representation_and_Reasoning]]"
  - "[[Robot_Hands]]"
key_takeaways:
  - "Perception improves when the robot acts to gather information: moving the camera, changing focus, touching or pushing objects."
  - "Information gain measures the value of a look: a cell known only at 50/50 holds 1 bit of uncertainty; a view that makes it 90/10 removes 0.53 bits."
  - "Classical methods plan views by expected information gain; 2026 policies often learn where to look, and language models decide what to look for."
sources:
  - "Bajcsy, R. (1988). Active perception. https://ieeexplore.ieee.org/document/5968"
  - "Thrun, S., Burgard, W. and Fox, D. (2005). Probabilistic Robotics, chapter 17: Exploration. https://mitpress.mit.edu/9780262201629/probabilistic-robotics/"
  - "Gu, Q. et al. (2023). ConceptGraphs: Open-Vocabulary 3D Scene Graphs for Perception and Planning. https://arxiv.org/abs/2309.16650"
---

# Active Perception and Sensing

A camera bolted to a wall sees what happens in front of it. A robot can do better: turn its head, step around an obstacle, open a drawer to look inside, or poke an object to see if it moves. **Active perception** treats sensing as something the robot *does*, choosing actions for the information they will bring.

> **Etymology.** Ruzena Bajcsy's 1988 paper *Active perception* defined the field: "we do not just see, we look". It built on psychologist James Gibson's view that animals perceive by acting.

## Why act to perceive?

| Problem | Passive result | Active solution |
|---|---|---|
| Occlusion | Object hidden | Move to a better viewpoint |
| Ambiguity | Is it a picture of a cup or a cup? | Move sideways (parallax) or touch it |
| Insufficient resolution | Text too small | Zoom or approach |
| Unknown properties | Weight, friction, articulation unknown | Lift, push or pull |
| Search | Object not in view | Plan where to look first |

## Worked example: value of a look

A cell in an occupancy map has a 50% chance of being occupied. Its uncertainty is the binary entropy $H(0.5) = 1$ bit. A view from a new pose would, on average, push the estimate to 90/10, with entropy $H(0.9) = -0.9\log_2 0.9 - 0.1\log_2 0.1 = 0.469$ bits. That view gains $1 - 0.469 = 0.53$ bits for this cell.

**Next-best-view** planners sum this expected gain over all the cells a candidate view would see, subtract the cost of getting there, and pick the best: "the most uncertainty removed per metre travelled". The same calculation drives exploration (frontier-based and information-theoretic) and inspection planning.

## Families of methods

- **Next-best-view** for 3D reconstruction and inspection.
- **Active search**: find a target object, using priors about where things usually are (mugs near sinks).
- **Active localization**: move to disambiguate where you are.
- **Interactive perception**: push, pull and probe to segment objects and discover joints, mass and friction.
- **Gaze control**: humanoid head and camera pointing to support the current task.

## Active perception in 2026

- **Learned policies**: end-to-end policies with movable cameras (wrist cameras, active necks) learn to look where it helps the task.
- **Language-guided search**: vision-language models supply priors ("cereal is usually in a cupboard near the fridge") and evaluate views, while scene graphs keep track of what has been seen (see [[Semantic_Scene_Understanding]]).
- **Touch as active sensing**: tactile exploration with vision-based tactile sensors (see [[Robot_Hands]]).

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Active_Perception_and_Sensing]])
```
