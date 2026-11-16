---
title: Shape Memory Alloys
description: "Shape-memory alloys (mostly nickel–titanium, Nitinol) return to a remembered shape when heated, through a solid-state phase change, producing large forces from small wires. Covers how the effect works, superelasticity, computed force and stroke for a typical wire actuator, why they are slow and inefficient, control, and robotic and medical uses in 2026."
aliases:
  - SMA
  - Nitinol
  - Shape Memory Alloy
tags:
  - materials
  - actuators
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
  - "[[Smart_Materials]]"
related:
  - "[[Robot_Hands]]"
  - "[[Soft_Robotics]]"
  - "[[Medical_Robotics]]"
key_takeaways:
  - "Heating a deformed NiTi wire above its transformation temperature turns it back to austenite and it recovers its shape, contracting by up to about 4% with high force."
  - "A 0.25 mm wire at 200 MPa pulls about 9.8 N; a 100 mm length contracts about 4 mm, so mechanisms often amplify stroke."
  - "Cooling limits speed (often under 1 Hz) and efficiency is a few per cent; SMAs suit compact, silent, low-duty actuators and medical devices, not high-power joints."
sources:
  - "Zhang, L. et al. (2021). A comparative review of artificial muscles for microsystem applications. https://www.nature.com/articles/s41378-021-00323-5"
  - "Smart Materials and Structures (2023). A review of shape memory alloy artificial muscles in bionic applications. https://iopscience.iop.org/article/10.1088/1361-665X/acf1e8"
  - "Rus, D. and Tolley, M. T. (2015). Design, fabrication and control of soft robots. https://www.nature.com/articles/nature14543"
---

# Shape Memory Alloys

A **shape-memory alloy (SMA)** is a metal that "remembers" a shape. Bend a nickel–titanium wire when it is cool, heat it a few tens of degrees, and it snaps back, pulling with considerable force. Passing an electric current through the wire heats it, so a thin SMA wire becomes a silent, compact, muscle-like actuator.

> **Etymology.** *Nitinol* = **Ni**ckel **Ti**tanium **N**aval **O**rdnance **L**aboratory, where William Buehler and Frederick Wang discovered its shape memory in 1962–63.

## How it works

NiTi has two crystal phases:
- **martensite** (low temperature): soft and easily deformed, by twinning rather than permanent slip;
- **austenite** (high temperature): stiff, with the "remembered" shape.

Heating through the transformation range turns deformed martensite into austenite, recovering up to about 4–8% strain; practical cyclic actuators use about 3–4%. Cooling reverses the phase, and a bias spring or opposing wire stretches the soft martensite again. **Superelasticity** is the same transformation driven by stress at body temperature: NiTi can be bent dramatically and spring back, which is why it is used for medical stents and guidewires.

## Worked example: a wire actuator

- **Force.** A 0.25 mm diameter wire actuated at a recovery stress of 200 MPa pulls  = \sigma A = 200\times10^6 \times \pi(0.125\times10^{-3})^2 \approx 9.8$ N, about 1 kgf from a wire thinner than a guitar string.
- **Stroke.** At 4% strain, a 100 mm wire contracts 4 mm. Designs use long folded wires, levers or bow springs to get more motion.

## Limits

- **Speed**: heating is fast but cooling is slow (passive cooling of thin wires takes on the order of a second), so cycles are often below 1 Hz; thinner wires, forced air or liquid cooling help.
- **Efficiency**: a few per cent of electrical energy becomes mechanical work.
- **Control**: hysteresis and temperature dependence make precise position control difficult; resistance feedback helps.
- **Fatigue**: large strains shorten life; small strains allow millions of cycles.

## Uses (2026)

Compact grippers and fingers, camera and lens actuators, valves and latches, insect-scale and soft robots, deployable mechanisms, and extensive medical devices (stents, guidewires, orthodontic wires) using superelasticity.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Shape_Memory_Alloys]])
```
