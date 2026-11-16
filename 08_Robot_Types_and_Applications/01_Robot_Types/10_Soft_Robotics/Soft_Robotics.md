---
title: Soft Robotics
description: "Soft robots are made of compliant materials (silicones, fabrics, gels) that bend and stretch instead of rigid links and joints. Covers why softness helps, materials and actuation (pneumatic, cable, smart materials), a computed actuator-force and stiffness comparison, sensing, modelling and control (Cosserat rods, learned models, deformable simulation), applications, and limits."
aliases:
  - Soft Robots
  - Soft Robot
tags:
  - soft-robotics
  - actuators
  - materials
  - bio-inspired-robotics
type: concept
level: intermediate
subfield:
  - soft-robotics
  - mechanisms-actuation
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Robots]]"
related:
  - "[[Robot_Hands]]"
  - "[[Bio-inspired_Robotics]]"
  - "[[Exoskeletons]]"
  - "[[MuJoCo]]"
  - "[[Physics_Engines]]"
key_takeaways:
  - "Soft robots trade precision and force for safety and adaptability: a body about a million times less stiff than steel conforms to objects and to people instead of fighting them."
  - "Most are driven by air pressure or cables; a 50 kPa chamber pushing on 2 cm² produces about 10 N, enough for gentle grasping."
  - "Their infinite degrees of freedom make modelling and control hard; Cosserat-rod models, learned models and deformable-body simulation (MuJoCo 4, 2026) are closing the gap."
sources:
  - "Rus, D. and Tolley, M. T. (2015). Design, fabrication and control of soft robots. https://www.nature.com/articles/nature14543"
  - "Wehner, M. et al. (2016). An integrated design and fabrication strategy for entirely soft, autonomous robots. https://www.nature.com/articles/nature19100"
  - "Laschi, C., Mazzolai, B. and Cianchetti, M. (2016). Soft robotics: Technologies and systems pushing the boundaries of robot abilities. https://www.science.org/doi/10.1126/scirobotics.aah3690"
  - "Della Santina, C., Duriez, C. and Rus, D. (2021). Model Based Control of Soft Robots: A Survey of the State of the Art and Open Challenges. https://arxiv.org/abs/2110.01358"
  - "Google DeepMind (2026). MuJoCo 4.0.0 release notes (deformables). https://github.com/google-deepmind/mujoco/releases/tag/4.0.0"
---

# Soft Robotics

**Soft robotics** builds robots from materials that bend, stretch and squeeze, such as silicone rubber, fabric, gels and foams, instead of from rigid links connected by joints. A soft gripper wraps around a strawberry without bruising it; a soft robot arm can bump into a person harmlessly; a soft crawler squeezes through gaps narrower than its body.

> **Etymology.** *Soft* from Old English *sōfte*, "gentle, agreeable". The field took its current form around 2010–2015, inspired by octopuses, elephant trunks and worms. The 2016 **Octobot** (Harvard) was the first robot made entirely of soft materials, powered by a chemical reaction and controlled by a soft microfluidic circuit.

## Why soft?

| Advantage | Because |
|---|---|
| **Safety near people** | Low stiffness limits impact forces |
| **Grasping without precise models** | The body conforms to object shape, so approximate positioning is enough |
| **Robustness** | Survives impacts and crushing |
| **Squeezing and reaching** | Continuous bending reaches around obstacles |

## Stiffness, computed

Material stiffness is measured by Young's modulus $E$:
- soft silicones used in soft robots: around $10^5$ Pa (100 kPa);
- structural steel: about $2 \times 10^{11}$ Pa.

Steel is roughly **two million times** stiffer. Human soft tissue is in the $10^3$–$10^6$ Pa range, which is why soft robots feel and behave more like living bodies than like machines.

## Actuation

| Method | How | Example numbers and notes |
|---|---|---|
| **Pneumatic / hydraulic** | Pressurise chambers in an elastomer so it bends or extends | A chamber at 50 kPa acting on 2 cm² (2 × 10⁻⁴ m²) pushes with $F = pA = 50{,}000 \times 0.0002 = 10$ N; the most common method |
| **Cable / tendon** | Motors pull cables routed through the soft body | Precise and fast; motors stay outside the soft part |
| **Shape-memory alloys** | Wires contract when heated | Compact; slow, inefficient |
| **Dielectric elastomers** | Voltage squeezes a thin rubber film | Fast, muscle-like; need kilovolts |
| **HASEL and electrohydraulic** | Voltage displaces liquid in a flexible pouch | Muscle-like speed and strength |
| **Fin-ray and passive designs** | Geometry that bends around objects under contact | Simple, cheap, widely used industrial grippers |

## Sensing

Soft bodies need soft sensors: stretchable conductive elastomers, liquid-metal traces, optical waveguides, and cameras watching deformation from inside (the principle behind vision-based tactile sensors; see [[Robot_Hands]]).

## Modelling and control

A soft body has, in principle, infinitely many degrees of freedom. Approaches, from simple to rich:
- **Piecewise constant curvature**: model each segment as an arc, which is easy and approximate;
- **Cosserat rod models**: accurate bending, twisting and stretching of slender bodies;
- **Finite elements** (e.g. SOFA): accurate and slow;
- **Learned models**: neural networks fitted to data, combined with physical priors;
- **Simulation for learning**: deformable-body support in physics engines lets policies for soft and deformable manipulation be trained in simulation (MuJoCo 4, released November 2026, added implicit deformables and plasticity; see [[MuJoCo]]).

Della Santina, Duriez and Rus (2021) survey model-based control. Learned control, often with reinforcement learning in simulation, is the most active area.

## Applications

Gentle grippers for food, produce and e-commerce; wearable assistive suits (see [[Exoskeletons]]); minimally invasive medical devices and steerable catheters; search-and-rescue crawlers; underwater robots that handle fragile marine life; and soft skins that make rigid robots safer to touch.

## Limits

Low force and precision, slow pneumatic response, fatigue and punctures, bulky pumps and valves, and difficult modelling. Most practical systems are **hybrids**: rigid arms with soft hands, skins or joints.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Soft_Robotics]])
```
