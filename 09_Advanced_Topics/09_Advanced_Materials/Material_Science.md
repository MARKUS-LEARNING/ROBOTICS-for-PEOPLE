---
title: Material Science
description: "Materials science for robot builders: the properties that matter (stiffness, strength, density, fatigue, damping, thermal and electrical behaviour), choosing materials by performance indices with a computed specific-stiffness comparison of aluminium, steel, titanium and carbon fibre, and the materials trends shaping robots in 2026 (composites, engineering polymers, soft and active materials)."
aliases:
  - Materials Science
  - Materials for Robotics
tags:
  - materials
  - mechanical-design
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
  - "[[Composite_Materials]]"
  - "[[Smart_Materials]]"
  - "[[Soft_Robotics]]"
  - "[[Biomaterials]]"
key_takeaways:
  - "Robots care about stiffness and strength per unit mass, because every kilogram in an arm or leg must be accelerated by the motors behind it."
  - "Aluminium, steel and titanium all have about the same specific stiffness (~25 MJ/kg); quasi-isotropic carbon fibre reaches ~44 MJ/kg, which is why it appears in fast arms, drones and legs."
  - "Active materials (shape-memory alloys, electroactive polymers, piezoelectrics) blur the line between structure and actuator."
sources:
  - "Zhang, L. et al. (2021). A comparative review of artificial muscles for microsystem applications. https://www.nature.com/articles/s41378-021-00323-5"
  - "Siciliano, B. and Khatib, O., eds. (2016). Springer Handbook of Robotics, 2nd edition (mechanisms and actuation). https://link.springer.com/referencework/10.1007/978-3-319-32552-1"
  - "Rus, D. and Tolley, M. T. (2015). Design, fabrication and control of soft robots. https://www.nature.com/articles/nature14543"
---

# Material Science

**Materials science** studies how a material's structure determines its properties, and how processing changes both. For robot builders it answers practical questions: what to make an arm link, a gear, a gripper finger or a battery enclosure from, so that it is stiff, strong, light, durable and affordable.

> **Etymology.** Latin *materia*, "the stuff of which something is made" (from *mater*, "mother"). Materials science emerged as a discipline in the 1950s–60s, joining metallurgy, ceramics and polymer science.

## Properties that matter for robots

| Property | Why it matters |
|---|---|
| **Stiffness** (Young's modulus $) | Deflection and vibration of links decide precision |
| **Strength** (yield, ultimate) | Survive peak loads and impacts |
| **Density** $\rho$ | Moving mass costs motor torque and energy |
| **Fatigue** | Robots repeat motions millions of times |
| **Damping** | Absorbs vibration and impacts |
| **Thermal** | Motors and electronics need heat paths |
| **Electrical** | Insulation, conductivity, EMI shielding |
| **Manufacturability and cost** | Machining, casting, moulding, printing |

## Worked example: specific stiffness

For a link that must not bend, mass for a given stiffness scales with $\rho/E$, so the figure of merit is **specific stiffness** /\rho$:

| Material | $ (GPa) | $\rho$ (kg/m³) | /\rho$ (MJ/kg) |
|---|---|---|---|
| Aluminium alloy | 70 | 2,700 | 25.9 |
| Steel | 200 | 7,850 | 25.5 |
| Titanium alloy | 110 | 4,500 | 24.4 |
| Carbon-fibre composite (quasi-isotropic) | ~70 | ~1,600 | ~43.8 |

The three metals are nearly identical; their differences lie in strength, cost and machinability. A quasi-isotropic carbon-fibre tube of the same stiffness weighs about 1600/2700 = 59% of the aluminium one, and more can be saved by aligning fibres with the load (see [[Composite_Materials]]). Choosing by such *performance indices*, and plotting materials on property charts, is Michael Ashby's method of materials selection.

## Materials trends in 2026 robots

- **Lightweight structures**: carbon fibre and magnesium in drones, legs and fast arms; topology-optimised aluminium.
- **Engineering polymers and 3D printing**: glass- and carbon-filled nylons for housings and grippers; printed parts in production humanoids and research robots.
- **Soft materials**: silicones and urethanes for skins, fingertips and soft actuators (see [[Soft_Robotics]]).
- **Active materials**: [[Shape_Memory_Alloys]], [[Electroactive_Polymers]] and piezoelectrics that act as actuators or sensors (see [[Smart_Materials]]).
- **Magnet and battery materials**: rare-earth magnets in motors and battery chemistry set robots' torque density and runtime, and their supply chains.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Material_Science]])
```
