---
title: Collaborative Robots (Cobots)
description: "Collaborative robots share a workspace with people, without fences, using certified safety functions: safety-rated monitored stop, hand guiding, speed and separation monitoring, and power and force limiting. Covers how each works, a computed minimum-separation example, what the 2025 revision of ISO 10218 changed, typical applications, and how cobots meet AI and mobile manipulation in 2026."
aliases:
  - Cobots
  - Cobot
  - Collaborative Robot
tags:
  - collaborative-robots
  - human-robot-interaction
  - industrial-robot
  - safety
type: robot-type
level: beginner
subfield:
  - industrial-automation
  - human-robot-interaction
status: reviewed
author: Jordan_Smith
date: 2025-05-11
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Industrial_Arms]]"
related:
  - "[[Human-Robot_Interaction]]"
  - "[[Manipulator_Arm]]"
  - "[[Ethical_and_Social_Implications]]"
  - "[[Humanoid_Robots]]"
key_takeaways:
  - "'Collaborative' describes the application, not just the robot: safety depends on the whole cell, including the tool, the payload, the speed and how close people come."
  - "Four collaborative modes exist: safety-rated monitored stop, hand guiding, speed and separation monitoring, and power and force limiting; most cobot cells use the last two."
  - "The 2025 revision of ISO 10218 brought the collaborative-robot requirements into the main robot-safety standard; every collaborative application still needs a risk assessment."
sources:
  - "ISO (2025). ISO 10218-2:2025 Robotics — Safety requirements — Part 2: Industrial robot applications and robot cells. https://www.iso.org/standard/73934.html"
  - "ISO (2025). ISO 10218-1:2025 Robotics — Safety requirements — Part 1: Industrial robots. https://www.iso.org/standard/73933.html"
  - "ISO (2016). ISO/TS 15066:2016 Robots and robotic devices — Collaborative robots. https://www.iso.org/standard/62085.html"
  - "International Federation of Robotics (2026). History of industrial robots (Universal Robots UR5, 2008). https://ifr.org/robot-history"
  - "European Union (2023). Regulation (EU) 2023/1230 on machinery products. https://eur-lex.europa.eu/eli/reg/2023/1230/oj"
---

# Collaborative Robots (Cobots)

A **collaborative robot** (cobot) is designed to work in the same space as people, without the fences that surround traditional [[Industrial_Arms|industrial robots]]. Cobots are lighter and slower, sense contact, and are easy to program by moving them by hand. Since Universal Robots sold the first commercially successful cobot, the UR5, in 2008, they have become a standard tool for small and medium manufacturers.

> **Etymology.** *Cobot* is a blend of *collaborative* and *robot*. It was coined by J. Edward Colgate and Michael Peshkin at Northwestern University in 1996, for passive devices that guided a human worker's motion.

## The four collaborative modes

| Mode | How it keeps people safe | Typical use |
|---|---|---|
| **Safety-rated monitored stop** | The robot stops (and is monitored as stopped) whenever a person enters the shared space | Loading parts into a fixture the robot then works on |
| **Hand guiding** | A person moves the robot with an enabling device; the robot follows | Teaching positions; assisted lifting |
| **Speed and separation monitoring (SSM)** | Sensors track people; the robot slows or stops as they approach | Shared cells with occasional entry |
| **Power and force limiting (PFL)** | The robot is designed and limited so that contact stays below biomechanical thresholds for the body region | Close work: assembly, screwdriving, packing |

**Collaborative is a property of the application.** A cobot carrying a knife, a hot part or a heavy load is not safe to touch whatever its specification. The integrator's risk assessment covers the whole cell.

## Worked example: how far away must the robot stop?

The general form used for safeguarding distances (ISO 13855, applied in speed and separation monitoring) is

$$S = K \cdot T + C$$

- $K = 1600$ mm/s is the assumed approach speed of a walking person;
- $T$ is the total stopping time (sensor response + controller + robot stopping);
- $C$ is an allowance for reaching in before being detected.

Suppose the sensor and controller respond in 0.1 s, the robot stops in 0.5 s at its current speed ($T = 0.6$ s), and $C = 850$ mm. Then

$$S = 1600 \times 0.6 + 850 = 1810\ \text{mm}.$$

The robot must begin its protective stop when a person is 1.81 m away. Halving the stopping time to 0.3 s by moving more slowly gives $S = 1330$ mm. This is the core trade-off of SSM: **slower robot, closer people**. Modern systems therefore slow down progressively as people approach rather than stopping outright.

## What changed in 2025

The February 2025 revision of **ISO 10218-1 and -2** folded collaborative-robot requirements, previously in the separate technical specification ISO/TS 15066 (2016), into the main industrial-robot safety standard and clarified the functional-safety requirements. In the EU, the **Machinery Regulation** applies from January 2027.

## Applications

Machine tending (loading CNC machines), packaging and palletising, screwdriving and assembly, quality inspection, laboratory automation and welding with a human operator. Cobots on mobile bases (mobile manipulators) move between stations.

## Cobots and AI (2026)

- **Easier programming**: demonstration, natural-language task setup and automatic path generation reduce setup from days to hours.
- **Perception**: vision-guided picking makes cobots tolerant of part variation.
- **Learned skills**: policies trained from demonstrations run on cobot arms in research and pilot cells. The certified safety functions (force limits, monitored stops) stay underneath and cannot be overridden by the policy.
- **Humanoids**: humanoids working near people raise the same questions at larger scale; see [[Humanoid_Robots]].

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Collaborative_Robots]])
```
