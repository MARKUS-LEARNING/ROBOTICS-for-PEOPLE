---
title: Biomaterials
description: "Biomaterials are materials designed to work with living tissue: in implants, prosthetics, medical robots and soft robots that touch people, and in biohybrid robots that use living cells. Covers biocompatibility and its testing, the main classes (metals, polymers, ceramics, hydrogels), stiffness matching between devices and tissue with a computed comparison, and biohybrid robotics in 2026."
aliases:
  - Biomaterial
  - Biocompatible Materials
  - Biohybrid Robots
tags:
  - materials
  - medical-robots
  - bio-inspired-robotics
type: concept
level: intermediate
subfield:
  - medical-robotics
  - bio-inspired
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Material_Science]]"
related:
  - "[[Medical_Robotics]]"
  - "[[Exoskeletons]]"
  - "[[Soft_Robotics]]"
  - "[[Nanorobots]]"
key_takeaways:
  - "A biomaterial must do its job without harming the body, and the body must not degrade it unexpectedly; biocompatibility is tested under the ISO 10993 series."
  - "Stiffness mismatch matters: brain tissue is around a kilopascal and skin around hundreds of kilopascals, while titanium is about 110 GPa, so soft interfaces (silicones, hydrogels) are used where devices touch tissue."
  - "Biohybrid robots go further, using living muscle cells or microorganisms as actuators; a decade of research has produced crawling, swimming and gripping bio-bots."
sources:
  - "Webster-Wood, V. A. et al. (2023). Biohybrid robots: recent progress, challenges, and perspectives. https://iopscience.iop.org/article/10.1088/1748-3190/ac9c3b"
  - "Soft Science (2025). A review: exploring the designs of bio-bots. https://www.oaepublish.com/articles/ss.2024.50"
  - "ISO (2018). ISO 10993-1: Biological evaluation of medical devices — Part 1. https://www.iso.org/standard/68936.html"
  - "Rus, D. and Tolley, M. T. (2015). Design, fabrication and control of soft robots. https://www.nature.com/articles/nature14543"
---

# Biomaterials

A **biomaterial** is a material engineered to interact with biological systems: a titanium hip implant, a silicone catheter, a hydrogel wound dressing, the soft liner of a prosthetic socket, or the tissue scaffold in which muscle cells grow. In robotics, biomaterials matter wherever robots touch, enter or are built from living tissue.

> **Etymology.** Greek *bios*, "life", + *materia*. The field formalised in the 1960s–70s as implants became common; the definition of biocompatibility as "the ability to perform with an appropriate host response in a specific application" (David Williams, 1987) is still used.

## Biocompatibility

Requirements depend on how and for how long a device contacts the body: skin contact for minutes, or blood contact for years. The **ISO 10993** series defines biological evaluation: cytotoxicity, sensitisation, irritation, and, for longer contact, systemic toxicity, genotoxicity and implantation tests.

## Classes

| Class | Examples | Uses |
|---|---|---|
| Metals | Titanium alloys, stainless steel, NiTi | Implants, surgical instruments, stents (see [[Shape_Memory_Alloys]]) |
| Polymers | Silicone, polyurethane, PEEK, PLA/PLGA (biodegradable) | Catheters, soft robot skins, sutures, scaffolds |
| Ceramics | Alumina, zirconia, hydroxyapatite | Joints, bone substitutes |
| Hydrogels | Polymer networks that hold water | Soft tissue interfaces, drug delivery, microrobots |
| Living materials | Cells, tissues, microorganisms | Biohybrid robots |

## Worked example: stiffness matching

| Material or tissue | Young's modulus (order of magnitude) |
|---|---|
| Brain tissue | ~1 kPa |
| Skin | ~0.1–1 MPa |
| Soft silicone (soft robot skin) | ~0.1 MPa |
| Bone (cortical) | ~10–20 GPa |
| Titanium alloy | ~110 GPa |

A rigid electrode in the brain is about ^8$ times stiffer than the tissue around it, which causes inflammation and scarring as the brain moves. A silicone fingertip on a robot touching skin is in the same stiffness range as the skin. Matching stiffness at the interface is a key design principle for implants, wearables and robots that handle people (see [[Exoskeletons]], [[Soft_Robotics]]).

## Biohybrid robots

Biohybrid robots use living components as actuators or sensors:
- crawling and swimming bio-bots driven by cardiomyocytes or skeletal muscle grown on soft scaffolds;
- microorganism-powered microrobots;
- muscle-actuated grippers.

A decade of research (reviewed in 2023 and 2025) has shown these work, but they face hard problems: keeping cells alive and fed, control, speed and scaling. They are research systems, valuable for medicine and for understanding biological actuation.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Biomaterials]])
```
