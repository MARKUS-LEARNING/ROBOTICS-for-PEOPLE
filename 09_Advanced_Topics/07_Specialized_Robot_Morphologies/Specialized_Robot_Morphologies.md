---
title: Specialized Robot Morphologies
description: "Robot body plans beyond arms, wheels and legs: continuum and snake robots, tensegrity robots, wheel-leg hybrids, cable and parallel robots, modular and reconfigurable robots, micro and soft robots. Covers what each body is good at, a computed continuum-robot curvature example, how body and controller are co-designed, and the 2026 trend toward morphology chosen by optimisation and learning."
aliases:
  - Robot Morphologies
  - Unconventional Robots
  - Continuum Robots
  - Snake Robots
tags:
  - mechanism-design
  - locomotion
type: overview
level: intermediate
subfield:
  - mechanisms-actuation
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Robots]]"
related:
  - "[[Soft_Robotics]]"
  - "[[Cable-Driven_Robots]]"
  - "[[Parallel_Robots]]"
  - "[[Modular_Robotics]]"
  - "[[Reconfigurable_Robots]]"
  - "[[Medical_Robotics]]"
key_takeaways:
  - "A robot's body (morphology) decides what it can do as much as its software: continuum and snake bodies reach through clutter, tensegrity survives drops, wheel-legs combine speed and stepping."
  - "A continuum segment 0.3 m long bent through 90° has constant curvature 5.24 rad/m, a bend radius of 0.19 m; modelling bending this way underlies continuum-robot kinematics."
  - "Morphology is increasingly co-designed with control by optimisation and learning in simulation, rather than chosen first by hand."
sources:
  - "Rus, D. and Tolley, M. T. (2015). Design, fabrication and control of soft robots. https://www.nature.com/articles/nature14543"
  - "Yim, M. et al. (2007). Modular Self-Reconfigurable Robot Systems [Grand Challenges of Robotics]. https://ieeexplore.ieee.org/document/4141037"
  - "Pott, A. (2018). Cable-Driven Parallel Robots: Theory and Application. https://link.springer.com/book/10.1007/978-3-319-76138-1"
  - "Merlet, J.-P. (2006). Parallel Robots, 2nd edition. https://link.springer.com/book/10.1007/1-4020-4133-0"
---

# Specialized Robot Morphologies

**Morphology** is a robot's body plan: its shape, joints, materials and how it touches the world. Most robots are arms, wheeled bases or legged walkers, but many tasks are better served by other bodies. This overview maps the unusual ones and links to their notes.

> **Etymology.** Greek *morphē*, "form", + *logos*, "study". *Morphological computation*, the idea that a well-designed body does part of the control work for free, became a theme in robotics in the 2000s.

## A gallery of body plans

| Morphology | Strength | Typical use | See |
|---|---|---|---|
| **Continuum / tentacle** | Bends continuously, reaches around obstacles | Surgery (steerable catheters, endoscopes), inspection inside engines | [[Medical_Robotics]] |
| **Snake robots** | Move through pipes, rubble and narrow gaps | Search and rescue, inspection | — |
| **Soft robots** | Safe, conforming, robust | Gripping delicate objects, wearables | [[Soft_Robotics]] |
| **Tensegrity** | Rods and cables in tension; light, deformable, survives impacts | Planetary landers and rovers (research) | — |
| **Wheel-leg hybrids** | Fast on flat ground, can step over obstacles | Wheeled quadrupeds and humanoids | [[Legged_Robots]] |
| **Cable-driven** | Huge workspaces, light | Stadium cameras, telescopes | [[Cable-Driven_Robots]] |
| **Parallel** | Stiff, fast, precise | Delta pickers, hexapod positioners | [[Parallel_Robots]] |
| **Modular / reconfigurable** | Change shape for the task | Research, adaptable systems | [[Modular_Robotics]], [[Reconfigurable_Robots]] |
| **Micro and nano** | Go where nothing else fits | Medicine | [[Nanorobots]] |

## Worked example: continuum-robot curvature

Many continuum robots are modelled as segments of **constant curvature**: each segment bends into a circular arc. A segment of length  = 0.3$ m bent through $\theta = 90° = \pi/2$ rad has curvature

58113\kappa = \frac{\theta}{s} = \frac{1.571}{0.3} = 5.24\ \text{rad/m}, \qquad r = \frac{1}{\kappa} = 0.19\ \text{m}.58113

Its tip ends \sin\theta = 0.19$ m along the original axis and (1 - \cos\theta) = 0.19$ m to the side. Chaining several such arcs gives the forward kinematics of a multi-segment continuum robot; tendon lengths or tube rotations set the curvatures.

## Co-designing body and brain (2026)

Instead of choosing a body and then writing a controller, researchers increasingly **optimise both together** in simulation: evolutionary and gradient-based morphology search, RL policies trained across body variations, and differentiable simulators that compute how a change in link length or stiffness affects performance. Commercial humanoids and quadrupeds also show morphological trends, such as wheel-legs and parallel-linkage ankles, chosen to make learned controllers more effective.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Specialized_Robot_Morphologies]])
```
