---
title: Nanorobots
description: "Nanorobots and microrobots are tiny machines, usually driven by external magnetic fields, light or ultrasound, or by chemistry or living cells, intended for medicine and micromanipulation. Covers the honest size range (mostly micrometres, not nanometres), propulsion at low Reynolds number, magnetic actuation, imaging and control, the 2025 large-animal results, and barriers to clinical use."
aliases:
  - Microrobots
  - Nanorobot
  - Medical Microrobots
tags:
  - nanotechnology
  - medical-robots
type: robot-type
level: advanced
subfield:
  - micro-nano
  - medical-robotics
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Nanotechnology]]"
related:
  - "[[Medical_Robotics]]"
  - "[[MEMS]]"
  - "[[Smart_Materials]]"
  - "[[Swarm_Robots]]"
key_takeaways:
  - "Most working 'nanorobots' are microrobots (micrometres to millimetres) with no onboard power or computer; they are steered from outside by magnetic fields, light, ultrasound or by carried cells."
  - "Low-Reynolds-number physics makes them swim with corkscrews, rolling or undulation rather than strokes, and control is closed through medical imaging."
  - "By 2025 magnetic microrobot platforms were demonstrated in large animals under clinical conditions; biocompatibility, tracking deep in the body, retrieval and regulation remain the barriers."
sources:
  - "Zhou, J. et al. (2023). System integration of magnetic medical microrobots: from design to control. https://pmc.ncbi.nlm.nih.gov/articles/PMC10758462/"
  - "Landers, F. C. et al. (2025). Clinically ready magnetic microrobots for targeted therapies. https://www.science.org/doi/10.1126/science.adx1708"
  - "Webster-Wood, V. A. et al. (2023). Biohybrid robots: recent progress, challenges, and perspectives. https://iopscience.iop.org/article/10.1088/1748-3190/ac9c3b"
  - "Feynman, R. P. (1959). There's Plenty of Room at the Bottom. https://web.pa.msu.edu/people/yang/RFeynman_plentySpace.pdf"
---

# Nanorobots

**Nanorobots**, and more accurately **microrobots**, are machines small enough to travel through blood vessels, tissue or microfluidic channels. The vision, from Feynman's 1959 talk and 1966's *Fantastic Voyage*, is of tiny devices that deliver drugs exactly where they are needed, clear blockages or perform surgery from inside the body.

> **Etymology.** *Nano-* (Greek *nanos*, "dwarf") + *robot*. In practice most systems are micrometres to a millimetre across; "nanorobot" is often used loosely for any untethered microscale device.

## What they are really like

| Property | Typical microrobot |
|---|---|
| Size | ~1 µm to 1 mm |
| Power and computation | None onboard: energy and control come from outside |
| Actuation | Magnetic fields (most common), light, ultrasound, chemical fuels, or living cells (bacteria, sperm, algae) as motors |
| Sensing | External imaging: fluoroscopy, ultrasound, MRI, magnetic particle imaging, photoacoustics |
| Materials | Magnetic particles in biocompatible or biodegradable polymers and hydrogels |

## How they move

At low Reynolds number (see [[Nanotechnology]]) inertia is negligible, so microrobots use:
- **helical (corkscrew) swimmers** rotated by a rotating magnetic field;
- **surface rollers**, which roll along vessel walls;
- **magnetic gradient pulling** (stronger field one way);
- **flexible undulating tails**, like sperm;
- **swarms** steered together (see [[Swarm_Robots]]).

Magnetic actuation systems, made of electromagnet arrays or moving permanent magnets, generate the fields, and the control loop is closed through imaging: locate the robot, compute the field, apply it, repeat.

## Where things stand (2026)

- **Large-animal demonstrations (2025).** A modular magnetic microrobot platform delivered drug-loaded microrobots through realistic vascular models and in sheep and pigs under clinical conditions (*Science*, 2025).
- **Targeted delivery** of drugs, cells and clot-dissolving agents is the leading application; urology, ophthalmology and the gastrointestinal tract are early targets.
- **Biohybrid systems** use bacteria or cells as motors and sensors.

## Barriers

- Tracking small robots deep in the body in real time.
- Biocompatibility, toxicity and retrieval or degradation after use.
- Delivering enough dose with tiny payloads.
- Scaling manufacture with consistent quality.
- Regulatory pathways for a new class of device.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Nanorobots]])
```
