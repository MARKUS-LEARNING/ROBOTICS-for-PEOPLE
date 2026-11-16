---
title: Nanotechnology
description: "Nanotechnology works with structures from about 1 to 100 nanometres, where surface forces, quantum effects and viscosity dominate. Covers the scales involved, why physics changes when things shrink (a computed Reynolds-number example), fabrication approaches, molecular machines (Nobel Prize 2016), nanomaterials used in robots (sensors, batteries, coatings), and the honest state of nanorobotics."
aliases:
  - Nanotech
  - Nanoscale Engineering
tags:
  - nanotechnology
  - materials
type: overview
level: intermediate
subfield:
  - micro-nano
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Material_Science]]"
related:
  - "[[Nanorobots]]"
  - "[[MEMS]]"
  - "[[Smart_Materials]]"
key_takeaways:
  - "At the nanometre scale, gravity and inertia become negligible while surface forces, Brownian motion and viscosity dominate."
  - "A 10 µm swimmer moving 10 µm/s in water has a Reynolds number of about 0.0001: it cannot coast, and must keep pushing like a person swimming in thick honey."
  - "Nanotechnology already improves robots indirectly (sensors, battery electrodes, coatings, flexible electronics); autonomous nanometre-scale robots remain research."
sources:
  - "Feynman, R. P. (1959). There's Plenty of Room at the Bottom. https://web.pa.msu.edu/people/yang/RFeynman_plentySpace.pdf"
  - "Nobel Prize Outreach (2016). The Nobel Prize in Chemistry 2016: the design and synthesis of molecular machines. https://www.nobelprize.org/prizes/chemistry/2016/summary/"
  - "Zhou, J. et al. (2023). System integration of magnetic medical microrobots: from design to control. https://pmc.ncbi.nlm.nih.gov/articles/PMC10758462/"
---

# Nanotechnology

**Nanotechnology** is the science and engineering of matter at the scale of nanometres (billionths of a metre): roughly 1–100 nm, between single molecules and the smallest features of a living cell. At that scale materials behave differently, and the rules that govern robots built from metal and motors stop applying.

> **Etymology.** Greek *nanos*, "dwarf". Richard Feynman's 1959 talk *There's Plenty of Room at the Bottom* is the usual starting point; Norio Taniguchi coined *nanotechnology* in 1974.

## Scales

| Object | Size |
|---|---|
| Atom | ~0.1–0.3 nm |
| DNA helix width | ~2 nm |
| Virus | ~20–300 nm |
| Transistor features (2026 chips) | a few nm to tens of nm |
| Bacterium | ~1–2 µm |
| Human hair width | ~70 µm |

## Why physics changes

Shrink an object and its volume (mass, inertia) falls like ^3$ while its surface falls like ^2$: surface forces (adhesion, friction, electrostatics) win. Thermal jostling (Brownian motion) becomes significant, and fluids behave as if extremely viscous.

**Worked example: the Reynolds number.** $\mathrm{Re} = \rho v L/\mu$ compares inertia with viscosity. For a 10 µm swimmer at 10 µm/s in water ($\rho = 1000$ kg/m³, $\mu = 10^{-3}$ Pa·s):

57905\mathrm{Re} = \frac{1000 \times 10^{-5} \times 10^{-5}}{10^{-3}} = 10^{-4}.57905

A person swimming has $\mathrm{Re} \sim 10^6$. At ^{-4}$, inertia is irrelevant: stop pushing and you stop almost instantly, and reciprocal strokes (open–close) achieve no net motion, so propulsion needs corkscrews or travelling waves, as bacteria use. Physicist E. M. Purcell's 1977 lecture *Life at Low Reynolds Number* made this famous.

## Making things small

- **Top-down**: lithography and etching (chips, MEMS, NEMS).
- **Bottom-up**: chemistry and self-assembly (molecules, DNA origami, nanoparticles).
- **Molecular machines**: synthetic molecular switches, shuttles and rotary motors, recognised by the 2016 Nobel Prize in Chemistry (Sauvage, Stoddart, Feringa).

## Nanotechnology in robots today

Nanomaterials improve sensors (graphene and nanowire strain sensors, tactile skins), batteries (nanostructured electrodes), coatings (wear, friction, water repellency) and flexible electronics. Nanotechnology also underpins the chips that run robot AI. Autonomous robots *at* the nanoscale are not yet practical; the closest working systems are micro-scale, externally driven devices (see [[Nanorobots]]).

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Nanotechnology]])
```
