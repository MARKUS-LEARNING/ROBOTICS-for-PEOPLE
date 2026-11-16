---
title: Manufacturing
description: "Manufacturing is where robots first went to work and where industrial robots still work: about 5.08 million at the end of 2025. Covers why factories suit robots, the 2025 numbers by region and industry (IFR), what robots do on a production line, the anatomy of a robot cell, takt time and OEE with a computed example, a payback calculation, and what AI does and does not yet do in factories."
aliases:
  - Robotics in Manufacturing
  - Factory Automation
  - Industrial Automation
  - Smart Manufacturing
tags:
  - manufacturing
  - automation
  - industrial-robot
  - workcell-design
type: overview
level: beginner
subfield:
  - industrial-automation
status: keystone
author: Jordan_Smith
date: 2026-11-15
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Robots]]"
related:
  - "[[Industrial_Arms]]"
  - "[[Robot_Work_Cell_Design]]"
  - "[[Programmable_Logic_Controllers]]"
  - "[[Robotic_Welding]]"
  - "[[Machine_Vision_in_Manufacturing]]"
  - "[[Collaborative_Robots]]"
  - "[[Humanoid_Robots]]"
key_takeaways:
  - "Factories suit robots because the work is repetitive, the surroundings can be arranged for the machine, and precision and uptime are worth money."
  - "In 2025 factories installed a record 603,307 industrial robots, taking the operational stock to 5,079,078; electronics, automotive and metal and machinery took 64.5% of them (IFR)."
  - "A cell that meets its takt time on paper can still miss demand: a 38 s cycle against a 45 s takt delivers only 1,190 of 1,200 parts a day at 83.8% OEE."
sources:
  - "Müller, C. (2026). World Robotics 2026 – Industrial Robots, executive summary. https://ifr.org/img/worldrobotics/Executive_Summary_WR_2026_-_Industrial_Robots.pdf"
  - "International Federation of Robotics (2026). Five Million Robots now Operate in Factories Globally (World Robotics 2026). https://ifr.org/ifr-press-releases/news/five-million-robots-now-operate-in-factories-globally"
  - "International Federation of Robotics (2026). Robot Density Highest in South Korea, Singapore and Germany. https://ifr.org/ifr-press-releases/news/robot-density-highest-in-south-korea-singapore-and-germany"
  - "International Federation of Robotics (2026). Global Sales of Professional Service Robots Surge 24%. https://ifr.org/ifr-press-releases/news/global-sales-of-professional-service-robots-surge-24-percent"
  - "Lean Enterprise Institute (2026). Lexicon: Takt Time. https://www.lean.org/lexicon-terms/takt-time/"
  - "Lean Enterprise Institute (2026). Lexicon: Overall Equipment Effectiveness. https://www.lean.org/lexicon-terms/overall-equipment-effectiveness/"
  - "Hägele, M., Nilsson, K., Pires, J. N. and Bischoff, R. (2016). Industrial Robotics (Springer Handbook of Robotics, chapter 54). https://link.springer.com/chapter/10.1007/978-3-319-32552-1_54"
---

# Manufacturing

**Manufacturing** is the making of goods at scale, and it is the home ground of robotics. The first industrial robot went to work in a car-parts plant in 1961. At the end of 2025 about 5.08 million industrial robots were operating in factories worldwide. Most of the engineering in this textbook, from kinematics to control to machine vision, was first proven on a production line.

> **Etymology.** Latin *manu factum*, "made by hand" (*manus*, "hand", + *facere*, "to make"). The word kept its name as the work moved from hands to machines. *Factory* comes from Latin *factor*, "maker".

## Why factories suit robots

| Property of factory work | Why it favours robots |
|---|---|
| **Repetitive** | The same motion thousands of times a day; a robot does not tire or drift |
| **Structured** | Parts can be presented in known places by fixtures, feeders and conveyors, so the robot needs little understanding of the world |
| **Valuable** | Precision, consistent quality and uptime are worth money on every part |
| **Hazardous** | Heat, fumes, sparks, heavy loads and presses are better faced by machines |

The classic summary is the three Ds: robots take the **dull, dirty and dangerous** jobs. The first Unimate stacked hot die-cast metal parts at a General Motors plant in Trenton, New Jersey.

## The numbers (2025)

From the International Federation of Robotics (IFR), *World Robotics 2026*:

| Measure | 2025 |
|---|---|
| Industrial robots in operation | **5,079,078** (+9%) |
| New installations | **603,307** (+11%), a record |
| Market value of installations | US$18.6 billion (+11%) |
| Asia | 457,315 units (76% of the total); China alone 354,226 (+20%) |
| Europe | 73,054 (−14%) |
| Americas | 57,044 (+14%); United States 38,428 |
| Forecast | 655,000 installations in 2026; 806,000 in 2029 |

**Who buys them.** Three industries took 64.5% of all new robots:

| Customer industry | Units installed in 2025 | Share of 603,307 |
|---|---|---|
| Electrical and electronics | 142,235 (+10%) | 23.6% |
| Automotive | 138,429 (+10%) | 22.9% |
| Metal and machinery | 108,571 (+22%) | 18.0% |

**Robot density**, the number of industrial robots per 10,000 manufacturing employees, compares countries of different sizes (IFR, November 2026):

| Country | Robots per 10,000 employees | Roughly one robot per… |
|---|---|---|
| South Korea | 1,066 | 9 employees |
| Singapore | 873 | 11 |
| Germany | 469 | 21 |
| Japan | 452 | 22 |
| United States | 323 | 31 |
| China | 193 | 52 |
| World average | 152 | 66 |

Dividing the market value by the units gives about US$31,000 per robot ($18.6 billion ÷ 603,307). The robot itself is only part of the cost of a working cell: tooling, safety equipment, programming and integration come on top (see [[Robot_Work_Cell_Design]]).

## What robots do on a production line

| Application | What the robot does | See |
|---|---|---|
| **Handling and machine tending** | Loads and unloads machine tools, presses and moulding machines; moves parts between stations | [[Robot_Work_Cell_Design]] |
| **Welding** | Spot-welds car bodies; arc-welds frames and structures | [[Robotic_Welding]] |
| **Assembly** | Inserts, screws, presses and fastens parts, often with force sensing | [[Compliance]] |
| **Dispensing and painting** | Applies paint, sealant and adhesive along precise paths | [[Industrial_Arms]] |
| **Palletising and packaging** | Stacks boxes and bags; packs products at high speed | [[Parallel_Robots]] |
| **Inspection and measurement** | Carries cameras and probes; checks every part | [[Machine_Vision_in_Manufacturing]] |
| **Processing** | Deburrs, grinds, polishes and cuts | [[Compliance]] |
| **Moving material** | Mobile robots carry parts between cells | [[Mobile_Robots]] |

## Anatomy of a robot cell

A robot is useless alone. It works inside a **cell**:
- the **robot** and its controller (see [[Industrial_Arms]]);
- **end-of-arm tooling**: the gripper, welding torch or spray gun (see [[Robot_Hands]]);
- **part presentation**: fixtures, feeders, conveyors, or a camera that finds parts;
- **sensors**: part-present switches, vision, force and torque sensing;
- a **cell controller**, usually a PLC, that sequences the robot with the machines around it (see [[Programmable_Logic_Controllers]]);
- **safety**: fences, light curtains, scanners and safety-rated stops (see [[Collaborative_Robots]]);
- an operator panel.

The company that designs and builds the cell is a **system integrator**. See [[Robot_Work_Cell_Design]].

## Worked example: will the cell keep up?

A factory must deliver **1,200 parts a day**. It runs two 8-hour shifts, each with 30 minutes of breaks.

**Takt time** is the available production time divided by customer demand: the rhythm at which a part must be finished.

$$\text{available time} = 2 \times (480 - 30)\ \text{min} = 900\ \text{min} = 54{,}000\ \text{s}, \qquad \text{takt} = \frac{54{,}000}{1{,}200} = 45\ \text{s}.$$

A robot machine-tending cell has a **cycle time** of 38 s per part. Since 38 s is less than 45 s, the cell seems to have room to spare: at full speed it would make $54{,}000 / 38 = 1{,}421$ parts a day.

Real equipment loses time. **Overall equipment effectiveness (OEE)** multiplies three rates:

$$\text{OEE} = \underbrace{0.90}_{\text{availability}} \times \underbrace{0.95}_{\text{performance}} \times \underbrace{0.98}_{\text{quality}} = 0.838.$$

- *Availability* 90%: the cell is stopped for breakdowns and changeovers 10% of the time.
- *Performance* 95%: short stops and slow running cost another 5%.
- *Quality* 98%: 2% of parts are scrapped or reworked.

Good parts per day: $1{,}421 \times 0.838 = 1{,}190$. That is **10 short of the 1,200 needed**. To meet demand at this OEE the cycle time must fall to $54{,}000 \times 0.838 / 1{,}200 = 37.7$ s, or the losses must shrink. Checking cycle time against takt time is not enough; check it against takt time *after* realistic losses.

## Worked example: does the robot pay for itself?

Illustrative numbers, not market data:
- a complete machine-tending cell costs $150,000 (robot, gripper, safety, integration);
- it takes over a loading job on two shifts, freeing two operators for other work; the fully loaded cost of each position is $55,000 a year;
- maintenance, energy and programming cost $10,000 a year.

$$\text{payback} = \frac{150{,}000}{2 \times 55{,}000 - 10{,}000} = 1.5\ \text{years}.$$

On one shift the saving halves and the payback stretches to $150{,}000 / 45{,}000 = 3.3$ years. That is why robots spread first in plants that run around the clock, and why lower integration costs matter so much to small manufacturers. Quality, safety and steadier output are real benefits too, though harder to price.

## Fixed, flexible and manual

| Approach | Best for | Weakness |
|---|---|---|
| **Fixed ("hard") automation**: dedicated machines and transfer lines | Very high volume, one product | Costly to change |
| **Robotic automation** | Medium to high volume, several variants | Programming and integration effort for each new part |
| **Manual work** | Low volume, high variety, delicate judgement | Cost, consistency, ergonomics |

The frontier is **high-mix, low-volume** production: many product variants in small batches, where reprogramming costs more than the robot saves. Easier programming, vision and learning aim at exactly this.

## AI in the factory, 2026: what is ready and what is not

The IFR's 2026 report draws the line plainly:

| Industry-grade today | Not yet industry-grade |
|---|---|
| Machine vision for detection and robot guidance | General-purpose "physical AI" systems |
| Predictive maintenance, process optimisation and AI-based inspection (increasingly used) | Learning from simulation and experience as a routine production method |
| | Humanoid robots: most remain prototypes or pilots |

The same report notes that standardised, modular cells, ready-made application packages and simpler programming can cut integration cost, which is the main barrier for smaller manufacturers.

- About 7,000 full-size humanoids (taller than 140 cm) were sold worldwide in 2025, against 603,307 industrial robots.
- The IFR's judgement is that at the current stage industrial robots are economically superior for production tasks, and that humanoids must first improve in reliability, safety, operating time and cost (see [[Humanoid_Robots]]).
- Learned policies are entering factories through narrower doors: bin picking, inspection, and handling of variable or deformable parts (see [[Vision-Language-Action_Models]], [[CNN_in_Robotics]]).

## Robots are manufactured too

A robot is itself a manufactured product: motors, precision gearboxes, bearings, castings, electronics and cables, assembled and calibrated. Manufacturing capacity now shapes the robot market:
- Chinese brands have supplied more than half of the robots installed in China since 2024 (IFR);
- humanoid makers are building dedicated factories. Figure describes its BotQ line as designed for up to 12,000 humanoids a year at first (a company statement, 2025).

How cheaply and reliably robots can be built, and repaired, will decide how far they spread beyond factories.

## Where to go next

[[Manipulator_Arm]] → [[Industrial_Arms]] → [[Robot_Work_Cell_Design]] → [[Programmable_Logic_Controllers]] → [[Robotic_Welding]] · [[Machine_Vision_in_Manufacturing]] → [[Collaborative_Robots]]

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Manufacturing]])
```
