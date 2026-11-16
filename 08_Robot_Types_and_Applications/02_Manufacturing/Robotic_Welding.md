---
title: Robotic Welding
description: "Welding is one of the oldest and largest jobs for industrial robots. Covers spot, arc and laser welding, the parts of a robotic welding cell, the process parameters a robot must hold, heat input with a computed example showing why steady travel speed matters, how robots find and follow a seam, programming from teach pendants to hand-guided cobots, the welder shortage, and safety."
aliases:
  - Robot Welding
  - Welding Robots
  - Arc Welding Robots
  - Spot Welding Robots
tags:
  - welding
  - manufacturing
  - industrial-robot
  - automation
type: method
level: intermediate
subfield:
  - industrial-automation
  - manipulation
status: reviewed
author: Jordan_Smith
date: 2026-11-16
last_reviewed: 2026-11-16
as_of: 2026-11
prerequisites:
  - "[[Industrial_Arms]]"
  - "[[Robot_Work_Cell_Design]]"
related:
  - "[[Manufacturing]]"
  - "[[Collaborative_Robots]]"
  - "[[Machine_Vision_in_Manufacturing]]"
  - "[[Programmable_Logic_Controllers]]"
key_takeaways:
  - "A welding robot's value is consistency: it holds travel speed, torch angle and distance steady along the whole seam, every time."
  - "Heat input scales with 1/travel speed: at 24 V and 220 A, slowing from 400 to 300 mm/min raises it from 0.63 to 0.85 kJ/mm, a third more heat into the part."
  - "The robot repeats far better than the parts do, so finding and following the real seam (touch sensing, through-arc tracking, laser vision) is the hard part of robotic welding."
sources:
  - "TWI (2026). What is the difference between heat input and arc energy? https://www.twi-global.com/technical-knowledge/faqs/faq-what-is-the-difference-between-heat-input-and-arc-energy"
  - "International Federation of Robotics (2026). History of industrial robots. https://ifr.org/robot-history"
  - "Müller, C. (2026). World Robotics 2026 – Industrial Robots, executive summary. https://ifr.org/img/worldrobotics/Executive_Summary_WR_2026_-_Industrial_Robots.pdf"
  - "American Welding Society (2026). Welding workforce data. https://weldingworkforcedata.com/"
  - "Hägele, M., Nilsson, K., Pires, J. N. and Bischoff, R. (2016). Industrial Robotics (Springer Handbook of Robotics, chapter 54). https://link.springer.com/chapter/10.1007/978-3-319-32552-1_54"
---

# Robotic Welding

**Welding** joins metal parts by melting them together. It is hot, bright, smoky and repetitive, and its quality depends on holding a steady hand for minutes at a time: exactly the job a robot does well. Welding car bodies was one of the first mass uses of industrial robots, and welding remains one of the largest.

> **Etymology.** *Weld* is a 16th-century variant of *well*, in its old sense "to boil, to rise up" (Old English *weallan*): metal brought to a welling heat.

## A short history

| Year | Milestone |
|---|---|
| 1969 | General Motors installs the first spot-welding robots at its Lordstown plant; more than 90% of body welds are automated, against 20–40% in traditional plants |
| 1972 | FIAT and Nissan install production lines of spot-welding robots |
| 1974 | Kawasaki builds an arc-welding robot, based on the Unimate, for motorcycle frames |
| 1975 | Hitachi's "Mr. AROS", the first sensor-based arc-welding robot, corrects its path with gap sensors |
| 1979 | Nachi introduces motor-driven spot-welding robots, starting the move from hydraulic to electric drives |

## The main processes

| Process | How it joins | Typical robot use |
|---|---|---|
| **Resistance spot welding** | Copper electrodes clamp overlapping sheets and pass a large current for a fraction of a second, melting a small "nugget" between them | Car bodies: thousands of spots per vehicle, heavy weld guns on large robots |
| **Gas metal arc welding** (GMAW; MIG/MAG) | An arc burns between a continuously fed wire and the part, inside a shielding gas | Frames, structures, machinery; the most common robotic arc process |
| **Gas tungsten arc welding** (GTAW; TIG) | An arc from a tungsten electrode; filler added separately | Thin, high-quality or exotic-metal welds |
| **Laser welding** | A focused laser beam melts a narrow seam | Fast, low-distortion seams in car bodies and batteries |

## Inside an arc-welding cell

- A six-axis **robot** carrying the torch (see [[Industrial_Arms]]).
- A **power source** and **wire feeder**, controlled by the robot program.
- A **torch** with a collision sensor, and a station that cleans the nozzle and trims the wire.
- A **positioner**: extra motorised axes that turn the workpiece so each seam is welded in the best position, moving in coordination with the robot.
- **Fixtures** that hold parts in place against heat distortion.
- **Fume extraction**, and an enclosure with screens against arc flash.

See [[Robot_Work_Cell_Design]] for the design process.

## What the robot must hold steady

Voltage, current (set through wire feed speed), **travel speed**, torch angle, and the distance from contact tip to work. A skilled human welder varies these by a little along a seam and from part to part. A robot does not, which is where the quality gain comes from.

### Worked example: heat input and travel speed

The **arc energy** delivered per unit length of weld is

$$AE = \frac{V \times I \times 60}{v \times 1000}\ \ \text{kJ/mm},$$

with voltage $V$ in volts, current $I$ in amperes and travel speed $v$ in mm/min. The **heat input** that actually reaches the part is $HI = \eta \times AE$, where the process efficiency factor $\eta$ is 0.8 for MIG/MAG welding and 0.6 for TIG.

A MAG weld at 24 V and 220 A:

| Travel speed | Arc energy | Heat input ($\eta = 0.8$) |
|---|---|---|
| 500 mm/min | 0.634 kJ/mm | 0.507 kJ/mm |
| 400 mm/min | 0.792 kJ/mm | 0.634 kJ/mm |
| 300 mm/min | 1.056 kJ/mm | 0.845 kJ/mm |

Slowing from 400 to 300 mm/min puts a third more heat into every millimetre of the joint ($400/300 = 1.33$). On thin material that means distortion or burning through. Speeding up to 500 mm/min cuts the heat by a fifth and risks a weld that does not fuse. Welding procedures specify a heat-input range for this reason, and a robot's constant speed keeps the weld inside it. A 600 mm seam at 400 mm/min takes 90 s of arc time.

## Finding and following the seam

A robot returns to a taught point to within a few hundredths of a millimetre. Cut, bent and tacked parts vary by a millimetre or more, and they move as they heat. So the main error is in the part, and the robot must find the real joint:

| Method | How it works |
|---|---|
| **Touch sensing** | Before welding, the wire or nozzle touches the part at a few points; the program shifts to match |
| **Through-arc seam tracking** | While the torch weaves across the joint, the welding current changes with the distance to each wall; the controller keeps the weave centred |
| **Laser seam tracking** | A laser-line sensor ahead of the torch measures the joint's position and gap in real time |
| **Camera-based and learned methods** | Vision models locate seams and adapt the path and parameters to the gap; see [[Machine_Vision_in_Manufacturing]] |

## Programming

- **Teach pendant**: jog the torch along the seam and record points; accurate, but slow for small batches.
- **Offline programming**: generate paths from the CAD model in a simulated cell, then correct on the real part with touch sensing.
- **Hand guiding**: on a collaborative robot the welder moves the torch by hand to mark the start, corner and end points, and picks a weld recipe (see [[Collaborative_Robots]]). This has brought robotic welding to small fabricators with short runs.

## Why the demand keeps growing

- **People.** The American Welding Society projects that the United States will need 320,500 new welding professionals by 2029, about 80,000 openings a year.
- **Industry.** The metal and machinery industry, where much of the robot work is welding and machine tending, installed 108,571 robots in 2025, up 22%, making it the third-largest customer industry (IFR).
- **Quality and safety.** Consistent welds, and people moved away from fumes and arc light.

## Safety

Arc welding adds its own hazards to those of any robot cell: intense ultraviolet light, fumes, sparks and hot metal. Cells are enclosed with light-tight screens, fumes are extracted at the source, and access is interlocked. The robot-cell standard ISO 10218-2 applies as for any application.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Robotic_Welding]])
```
