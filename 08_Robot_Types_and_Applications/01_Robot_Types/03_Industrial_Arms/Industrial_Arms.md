---
title: Industrial Arms
description: "Industrial robot arms are the workhorses of manufacturing: about five million were operating worldwide at the end of 2025. Covers their history from the Unimate, types and applications, the 2026 market (IFR World Robotics 2026), how they are programmed, safety under the revised ISO 10218:2025 standards, collaborative robots, and how AI is changing industrial automation."
aliases:
  - Industrial Robots
  - Industrial Robot Arm
tags:
  - industrial-robot
  - manipulator-arm
  - automation
  - safety
type: robot-type
level: beginner
subfield:
  - industrial-automation
  - manipulation
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Manipulator_Arm]]"
related:
  - "[[Manufacturing]]"
  - "[[Robot_Work_Cell_Design]]"
  - "[[Robotic_Welding]]"
  - "[[Collaborative_Robots]]"
  - "[[Ethical_and_Social_Implications]]"
  - "[[MoveIt2]]"
  - "[[CNN_in_Robotics]]"
  - "[[Parallel_Robots]]"
key_takeaways:
  - "The global stock of industrial robots reached a record 5.08 million in 2025, with 603,307 new installations; China installed 59% of them (IFR, World Robotics 2026)."
  - "Most industrial arms still run taught or offline-programmed motions in fenced cells; vision, force sensing and collaborative operation have widened what they do."
  - "Safety is governed by ISO 10218-1/-2 (revised 2025) and, in the EU from January 2027, the Machinery Regulation; AI-driven behaviour sits on top of certified safety functions, not in place of them."
sources:
  - "International Federation of Robotics (2026). Five Million Robots now Operate in Factories Globally (World Robotics 2026). https://ifr.org/ifr-press-releases/news/five-million-robots-now-operate-in-factories-globally"
  - "International Federation of Robotics (2026). World Robotics 2026: Industrial Robots, executive summary. https://ifr.org/img/worldrobotics/Executive_Summary_WR_2026_-_Industrial_Robots.pdf"
  - "International Federation of Robotics (2026). History of industrial robots. https://ifr.org/robot-history"
  - "ISO (2025). ISO 10218-1:2025 Robotics — Safety requirements — Part 1: Industrial robots. https://www.iso.org/standard/73933.html"
  - "European Union (2023). Regulation (EU) 2023/1230 on machinery products. https://eur-lex.europa.eu/eli/reg/2023/1230/oj"
---

# Industrial Arms

**Industrial arms** are [[Manipulator_Arm|robot arms]] built for factories: strong, fast, precise and able to run the same motion millions of times. They weld car bodies, paint, palletise, tend machine tools, assemble electronics and pack food.

> **Etymology.** *Industry* is from Latin *industria*, "diligence". The first industrial robot, the **Unimate**, was patented by George Devol in 1954 (as "programmed article transfer") and commercialised with Joseph Engelberger.

## History

The era of industrial robotics began with the **Unimate**, developed by George Devol and Joseph Engelberger in the late 1950s and first installed at a General Motors plant in 1961 to handle hot die-cast metal parts. Early robots like the Unimate were typically hydraulic and programmed by teaching specific joint coordinates. The automotive industry was an early adopter, using robots for tasks like spot welding by the late 1960s. Key developments include:
* **1962:** The cylindrical **Versatran** robot by AMF.
* **1969:** The first commercial painting robot by **Trallfa** (Norway). Kawasaki produces the Unimate under license in Japan.
* **1973:** KUKA develops the **Famulus**, the first robot with six electromechanically driven axes.
* **1974:** ASEA (now ABB) introduces the all-electric, microprocessor-controlled **IRB-6**. Cincinnati Milacron introduces the minicomputer-controlled **T3**.
* **1978:** The **SCARA** (Selective Compliance Assembly Robot Arm) configuration is developed in Japan, revolutionizing electronics assembly.
* **1979:** Unimation introduces the **PUMA** (Programmable Universal Machine for Assembly), which became a standard in research labs.
* **1980s onward:** Development of parallel robots like the **Delta** robot for high-speed tasks. Increasing use of computer control, offline programming, and sensors.
* **2010s onward:** Rise of [[Collaborative_Robots|Collaborative Robots (Cobots)]], beginning with Universal Robots' UR5 (2008).
* **2020s:** Vision-guided picking with deep learning; China becomes the majority market; AI policies trained from demonstrations enter pilot production lines.

## Common types and applications

| Type | Strength | Typical applications |
|---|---|---|
| Articulated (6-axis) | Flexible reach and orientation | Welding, painting, machine tending, assembly |
| SCARA | Fast, rigid in the vertical axis | Electronics assembly, small-part pick-and-place |
| Delta (parallel) | Extremely fast for light loads | Food and pharmaceutical packaging |
| Cartesian / gantry | Large, simple, accurate | Palletising, large-part handling |
| Collaborative (cobots) | Safe force and speed limits; easy to teach | Machine tending, packaging, inspection with people nearby |

## The market in 2026

From the IFR's *World Robotics 2026* (published 24 September 2026):

| Figure | Value |
|---|---|
| Operational stock worldwide, end of 2025 | **5,079,078** (+9%) |
| New installations in 2025 | **603,307** (+11%), a record |
| China | 354,226 units (+20%), 59% of the world total |
| Largest customer industries | Electrical and electronics 142,235; automotive 138,429; metal and machinery 108,571 (+22%) |
| Forecast installations | 655,000 in 2026; 806,000 in 2029 |

Electronics regained the lead from automotive in 2024 and kept it in 2025; metal and machinery grew fastest of the three. [[Manufacturing]] has the full picture, including robot density by country.

## How they are programmed

1. **Teach pendant**: jog the arm to points and build a program, in vendor languages such as RAPID (ABB), KRL (KUKA) or URScript.
2. **Offline programming**: build and verify the program in a simulated digital twin; download it to the robot.
3. **Vision guidance**: cameras and deep-learning detectors locate parts so the robot adapts to where they are (see [[CNN_in_Robotics]]).
4. **Force control**: for polishing, insertion and assembly where contact matters.
5. **Learned skills** (emerging): policies trained from demonstrations handle variable parts and deformable materials; in 2026 these are in pilots and specific cells rather than general use.

## Safety

Traditional industrial robots are fast and powerful enough to kill, so they work behind **guarding**: fences, light curtains and interlocked doors. Collaborative operation without fences relies on certified **safety functions**: speed and separation monitoring, and power and force limiting.

- **ISO 10218-1 and -2** were revised in February 2025, integrating requirements formerly in the collaborative-robot technical specification.
- In the EU, the **Machinery Regulation (2023/1230)** applies from 20 January 2027, including to machines with self-learning behaviour.
- AI components must sit **above** certified safety functions. A learned policy can choose where to move, but it cannot disable a protective stop.

## Where AI is changing industrial automation

- **Perception**: bin picking of mixed parts, defect inspection.
- **Faster deployment**: programming by demonstration, natural-language task setup, automatic path generation from CAD.
- **Flexible cells**: robots that handle part variation without reprogramming.
- **Humanoids and mobile manipulators**: piloted for tasks in existing lines that fixed arms cannot reach (see [[Humanoid_Robots]]).

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Industrial_Arms]])
```
