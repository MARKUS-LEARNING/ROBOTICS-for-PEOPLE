---
title: Composite Materials
description: "Composites combine materials, typically strong fibres in a polymer matrix, to get properties neither has alone. Covers fibre types, lay-up and anisotropy, the rule of mixtures with a computed example, why composites suit robot links, legs and drone frames, manufacturing, and failure modes such as delamination and impact damage."
aliases:
  - Composites
  - Carbon Fibre
  - CFRP
tags:
  - materials
  - mechanical-design
type: concept
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
  - "[[Drones]]"
  - "[[Legged_Robots]]"
  - "[[Manipulator_Arm]]"
key_takeaways:
  - "Fibres (carbon, glass, aramid) carry load; the matrix (usually epoxy) holds them together and transfers load; properties depend strongly on fibre direction."
  - "Along the fibres, the rule of mixtures gives E ≈ V_f E_f + (1 − V_f) E_m: 60% carbon fibre at 230 GPa in 3.5 GPa epoxy gives about 139 GPa, a stiffness that drops sharply across the fibres."
  - "Composites save mass in arms, legs and drone frames, but are anisotropic, can delaminate under impact, and are harder to join and inspect than metals."
sources:
  - "Yuan, M. et al. (2024). Progress and prospects of recycling technology for carbon fiber reinforced polymer. https://www.frontiersin.org/journals/materials/articles/10.3389/fmats.2024.1484544/full"
  - "Siciliano, B. and Khatib, O., eds. (2016). Springer Handbook of Robotics, 2nd edition. https://link.springer.com/referencework/10.1007/978-3-319-32552-1"
  - "Ateeq, M. et al. (2023). A state of art review on recycling and remanufacturing of the carbon fiber from carbon fiber polymer composite. https://ira.lib.polyu.edu.hk/handle/10397/108544"
---

# Composite Materials

A **composite** combines two or more materials into one with better properties than either alone. The most important for robots is **carbon-fibre-reinforced polymer (CFRP)**: very stiff, strong carbon fibres embedded in an epoxy matrix.

> **Etymology.** Latin *compositus*, "put together". Straw-reinforced mud bricks are an ancient composite; modern carbon fibre was developed in the late 1950s and 1960s, notably at the UK's Royal Aircraft Establishment.

## Ingredients

| Fibre | Strengths | Typical use |
|---|---|---|
| Carbon | Highest stiffness and strength per weight | Robot links, drone frames, legs |
| Glass | Cheap, tough, electrically insulating | Housings, covers, printed composites |
| Aramid (Kevlar) | Impact and cut resistance | Protective parts |

The **matrix** (epoxy, or thermoplastics such as nylon or PEEK) holds fibres in place and transfers load between them.

## Worked example: rule of mixtures

Along the fibre direction, stiffness is roughly the volume-weighted average:

57768E_\parallel \approx V_f E_f + (1 - V_f)E_m = 0.6 \times 230 + 0.4 \times 3.5 = 139.4\ \text{GPa}.57768

Across the fibres the matrix dominates and stiffness falls to roughly 8–10 GPa. This **anisotropy** is why laminates stack plies at 0°, ±45° and 90°. A balanced "quasi-isotropic" layup ends up near 70 GPa in every in-plane direction: about aluminium's stiffness at 60% of its density (see [[Material_Science]]).

## Why robots use them

- Lighter links mean less inertia, so smaller motors, faster motion and safer collisions.
- Higher natural frequencies, so less vibration and better precision.
- Tubes and shells made of composites resist bending and torsion efficiently.

Typical uses: drone frames and propellers, arm links of fast or collaborative arms, legs of quadrupeds and humanoids, end-effector tooling.

## Manufacturing and pitfalls

Made by prepreg lay-up and autoclave curing, filament winding (tubes), pultrusion (constant profiles), resin transfer moulding, or 3D printing with chopped or continuous fibre.

Pitfalls:
- **delamination** from impacts, which may be invisible on the surface;
- difficult **joints**: metal inserts and bonded fittings concentrate stress;
- electrical conductivity of carbon (galvanic corrosion against aluminium);
- harder **repair, inspection and recycling** than metals. Thermoset (epoxy) composites are especially hard to recycle; pyrolysis, solvolysis and mechanical methods recover fibres at reduced quality, and recycled-fibre 3D printing is an active area (reviews 2023–2024).

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Composite_Materials]])
```
