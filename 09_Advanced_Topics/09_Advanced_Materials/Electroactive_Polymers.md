---
title: Electroactive Polymers
description: "Electroactive polymers (EAPs) change shape in response to electric fields: dielectric elastomers that act like stretchy capacitors, and ionic polymers that bend at a few volts. Covers how each works, the Maxwell-pressure equation with a computed example, performance compared with natural muscle, HASEL actuators, and uses and open problems in 2026."
aliases:
  - EAP
  - Dielectric Elastomer Actuators
  - Artificial Muscles
tags:
  - materials
  - actuators
  - soft-robotics
type: concept
level: advanced
subfield:
  - mechanisms-actuation
  - soft-robotics
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Smart_Materials]]"
related:
  - "[[Soft_Robotics]]"
  - "[[Robot_Hands]]"
  - "[[Shape_Memory_Alloys]]"
key_takeaways:
  - "Dielectric elastomers squeeze a soft insulating film between compliant electrodes; the electrostatic Maxwell pressure p = ε₀εᵣE² thins it and makes it spread."
  - "At 50 V/µm in an elastomer with εᵣ = 3 the pressure is about 66 kPa, comparable to natural muscle stress, but it needs kilovolts across practical film thicknesses."
  - "Ionic EAPs work at 1–5 V and bend, but are slow and weak; high voltage, durability and electrode reliability are the main barriers to wider use."
sources:
  - "Maksimkin, A. V. et al. (2022). Electroactive Polymer-Based Composites for Artificial Muscle-like Actuators: A Review. https://pmc.ncbi.nlm.nih.gov/articles/PMC9268644/"
  - "Dewang, Y. et al. (2025). Research Progress in Electroactive Polymers for Soft Robotics and Artificial Muscle Applications. https://pmc.ncbi.nlm.nih.gov/articles/PMC11945207/"
  - "Zhang, L. et al. (2021). A comparative review of artificial muscles for microsystem applications. https://www.nature.com/articles/s41378-021-00323-5"
---

# Electroactive Polymers

**Electroactive polymers (EAPs)** are plastics and rubbers that move when an electric field is applied. Because they are soft, light and can strain a lot, they are the leading candidates for **artificial muscles** in soft robots, haptics and prosthetics.

> **Etymology.** *Electro-* + *active* + *polymer* (Greek *poly*, "many", + *meros*, "part"). Work on dielectric elastomer actuators at SRI International in the late 1990s showed strains above 100%, launching the field as it is today.

## Two classes

| Class | Mechanism | Voltage | Strain | Speed | Downsides |
|---|---|---|---|---|---|
| **Electronic (field-driven)**: dielectric elastomers, ferroelectric polymers, liquid-crystal elastomers | Electrostatic forces squeeze the material | kV | Large (10–100%+) | Fast (ms) | High voltage, breakdown, electrode fatigue |
| **Ionic**: ionic polymer–metal composites, conducting polymers, ionic gels | Ion migration swells one side, causing bending | 1–5 V | Bending | Slow (s) | Weak force, often need to stay wet, efficiency |

## Worked example: Maxwell pressure

A dielectric elastomer actuator is a compliant capacitor: a soft insulating film between stretchable electrodes. The effective electrostatic pressure squeezing the film is

57839p = \varepsilon_0 \varepsilon_r E^2.57839

For an acrylic or silicone film with $\varepsilon_r = 3$ at a field of  = 50$ V/µm  5\times10^7$ V/m:

57839p = 8.85\times10^{-12} \times 3 \times (5\times10^7)^2 \approx 66\ \text{kPa}.57839

That is in the range of natural skeletal muscle's typical working stress. But a 50 µm film needs 2.5 kV to reach that field, so stacking many thin layers, thinner films, and compact high-voltage electronics are central engineering problems.

## Related actuators

**HASEL** (hydraulically amplified self-healing electrostatic) actuators use electrostatic zipping to push liquid within flexible pouches. They combine muscle-like speed and force with tolerance of dielectric breakdown (they self-heal), and are an active direction in 2020s soft robotics.

## Uses and open problems (2026)

Haptic displays, tunable lenses, soft grippers, small swimming and crawling robots, wearable assistive devices, and sensors (the same capacitor measures stretch). Open problems: lower driving voltages, long-term reliability of electrodes and films, compact power electronics, and scaling force up for limbs.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Electroactive_Polymers]])
```
