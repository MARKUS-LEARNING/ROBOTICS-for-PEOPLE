---
title: Smart Materials
description: "Smart materials change shape, stiffness or properties in response to a stimulus (heat, electric or magnetic field, light, moisture) and can act as actuators or sensors without conventional motors. Covers the main families (shape-memory alloys and polymers, piezoelectrics, electroactive polymers, magnetostrictive, liquid-crystal elastomers, magnetorheological fluids), a comparison table, computed examples, and where they are used in robots in 2026."
aliases:
  - Active Materials
  - Responsive Materials
tags:
  - materials
  - actuators
  - soft-robotics
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
  - "[[Material_Science]]"
related:
  - "[[Shape_Memory_Alloys]]"
  - "[[Electroactive_Polymers]]"
  - "[[Soft_Robotics]]"
  - "[[MEMS]]"
key_takeaways:
  - "A smart material converts a stimulus directly into motion or a signal, so the material can be the actuator or the sensor."
  - "Each family trades strain, stress, speed and efficiency differently: piezoelectrics are fast and precise but move micrometres; shape-memory alloys are strong but slow and inefficient; dielectric elastomers stretch a lot but need kilovolts."
  - "In 2026 they are used where motors are too big or too stiff: micro-positioning, haptics, small grippers, soft and microrobots, and adaptive dampers."
sources:
  - "Zhang, L. et al. (2021). A comparative review of artificial muscles for microsystem applications. https://www.nature.com/articles/s41378-021-00323-5"
  - "Maksimkin, A. V. et al. (2022). Electroactive Polymer-Based Composites for Artificial Muscle-like Actuators: A Review. https://pmc.ncbi.nlm.nih.gov/articles/PMC9268644/"
  - "Dewang, Y. et al. (2025). Research Progress in Electroactive Polymers for Soft Robotics and Artificial Muscle Applications. https://pmc.ncbi.nlm.nih.gov/articles/PMC11945207/"
---

# Smart Materials

**Smart materials** respond to a stimulus with a useful change: a wire that contracts when heated, a crystal that expands when a voltage is applied, a rubber film that spreads under an electric field, a fluid that turns nearly solid in a magnetic field. They let the material itself act as an actuator or a sensor.

> **Etymology.** *Smart* in the sense of "responsive", a 1980s–90s engineering label (also *intelligent*, *adaptive* or *active* materials). The piezoelectric effect, the oldest, was discovered by Pierre and Jacques Curie in 1880.

## Families

| Material | Stimulus → response | Strain | Speed | Robot uses |
|---|---|---|---|---|
| **Shape-memory alloy (NiTi)** | Heat → contraction (phase change) | ~4% | Slow (cooling limits) | Small actuators, grippers, valves; see [[Shape_Memory_Alloys]] |
| **Shape-memory polymer** | Heat → recovers a programmed shape | Large | Slow | Deployable structures, medical devices |
| **Piezoelectric ceramics** | Voltage ↔ strain | ~0.1% | Very fast (kHz–MHz) | Nanopositioners, ultrasonic motors, sensors, insect-scale robots |
| **Dielectric elastomers** | Voltage → area expansion | 10–100%+ | Fast | Artificial muscles; see [[Electroactive_Polymers]] |
| **Ionic polymers (IPMC)** | Low voltage → bending | Bending | Moderate | Underwater and biomedical actuators |
| **Liquid-crystal elastomers** | Heat or light → contraction | Large | Moderate | Light-driven soft robots |
| **Magnetorheological fluids/elastomers** | Magnetic field → stiffness change | — | Fast | Adaptive dampers, variable-stiffness grippers |

## Worked example: why piezoelectrics move so little

A piezoelectric stack 20 mm long with a maximum strain of 0.1% extends by  \times 0.001 = 0.02$ mm, which is 20 µm. That is perfect for nanometre-precise positioning and very fast motion, useless for a robot joint without mechanical amplification (levers, flexures) or stepping motion (inchworm and ultrasonic motors). Every smart material has a similar trade-off that shapes where it is used.

## Uses in 2026

Precision stages and camera autofocus (piezo); haptic feedback in teleoperation; compact grippers and valves (SMA); soft and microrobots (dielectric elastomers, LCEs, magnetic composites); adaptive suspension and variable-stiffness joints (MR fluids); and sensing skins, where many of these materials work in reverse, generating signals from deformation.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Smart_Materials]])
```
