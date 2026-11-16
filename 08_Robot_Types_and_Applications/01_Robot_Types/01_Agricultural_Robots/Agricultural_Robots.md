---
title: Agricultural Robots
description: "Robots on farms: autonomous tractors, precision sprayers that target individual weeds, laser and mechanical weeders, harvesting robots, milking robots and agricultural drones. Covers the enabling technologies (RTK GNSS, machine vision, deep learning), a computed example from 2025 precision-spraying results, why harvesting is still hard, and the economics and open problems in 2026."
aliases:
  - Agribots
  - Agbots
  - Agricultural Robotics
  - Precision Agriculture
tags:
  - agriculture
  - computer-vision
  - autonomous-systems
  - field-robotics
type: robot-type
level: beginner
subfield:
  - agriculture
  - field-robotics
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Mobile_Robots]]"
related:
  - "[[Drones]]"
  - "[[CNN_in_Robotics]]"
  - "[[Autonomous_Robots]]"
  - "[[Soft_Robotics]]"
key_takeaways:
  - "Farm robots already work at scale where tasks are repetitive and positions are known: guidance with RTK GNSS, milking, and vision-targeted spraying."
  - "Plant-level precision is the big win: John Deere reported See & Spray on over five million acres in 2025, cutting non-residual herbicide use by nearly 50% on average (about 31 million gallons of mix)."
  - "Harvesting delicate produce remains hard because of occlusion, variability and gentle handling at speed; labour shortages keep pushing development."
sources:
  - "John Deere (2025). John Deere customers use See & Spray technology across five million acres in 2025. https://www.deere.com/en/news/all-news/see-spray-technology-across-5-million-acres/"
  - "Carbon Robotics (2026). LaserWeeder. https://carbonrobotics.com/laserweeder"
  - "Lely (2026). Astronaut A5 milking robot. https://www.lely.com/us/solutions/milking/astronaut-a5/"
  - "Siciliano, B. and Khatib, O., eds. (2016). Springer Handbook of Robotics, 2nd edition, chapter on robotics in agriculture and forestry. https://link.springer.com/referencework/10.1007/978-3-319-32552-1"
---

# Agricultural Robots

**Agricultural robots** bring automation to farming: machines that steer themselves across fields, spray only the weeds, pull or laser weeds, pick fruit, milk cows and survey crops from the air. They respond to labour shortages, rising input costs and the push to use fewer chemicals.

> **Etymology.** Latin *ager*, "field", + *cultura*, "cultivation". *Precision agriculture*, managing each part of a field (and now each plant) according to its needs, grew out of GPS guidance in the 1990s.

## What they do

| Task | Robots | Maturity (2026) |
|---|---|---|
| **Guidance and autonomy** | Self-steering and driverless tractors using RTK GNSS (centimetre-level positioning) | Mature (guidance); driverless tillage in early commercial use |
| **Precision spraying** | Cameras on a sprayer boom detect weeds and fire individual nozzles | Commercial at scale |
| **Weeding** | Laser weeders, mechanical weeding robots | Commercial in high-value crops |
| **Milking** | Cows walk into robotic milking stalls voluntarily; arms attach cups using vision | Mature since the 1990s |
| **Harvesting** | Strawberry, apple and tomato pickers; robotic lettuce harvest | Pilot to early commercial |
| **Drones** | Mapping, crop health imaging, spraying | Widespread; see [[Drones]] |
| **Monitoring** | Ground robots scouting rows for disease and yield estimation | Research to early commercial |

## Worked example: what precision spraying saves

John Deere reported that farmers ran See & Spray on more than 5 million acres in the 2025 season, saving about 31 million gallons of herbicide mix: a reduction in non-residual herbicide of nearly 50% on average.

Per acre, that is $31\times10^6 / 5\times10^6 = 6.2$ gallons of mix saved. If that is about half of what broadcast spraying would have used, broadcast use was around 12 gallons per acre on those fields. On a 1,000-acre farm, the savings are about 6,200 gallons of mix per season.

These are the vendor's own figures. Independent university trials have reported similar reductions in post-emergence herbicide, but savings depend on weed pressure: a field with few weeds saves more.

## Enabling technologies

- **RTK GNSS**: satellite positioning corrected by a base station to about 2 cm, enough to drive between crop rows.
- **Machine vision and deep learning**: detecting weeds among crops, ripeness and disease from images; trained on large labelled datasets (see [[CNN_in_Robotics]]).
- **Fast actuation**: individually controlled nozzles, lasers or blades acting within milliseconds as the machine moves.
- **Fleet management**: scheduling, remote supervision and data platforms.

## Why harvesting is hard

Fruit hides behind leaves; every plant is different; produce bruises easily; and picking must be fast enough to compete with skilled workers at harvest-season wages. Approaches include soft grippers and suction (see [[Soft_Robotics]]), redesigned orchards (trellised "fruit walls" that are easier for machines), and multi-arm platforms.

## Open problems

Cost per acre for smaller farms; reliability in dust, mud and weather; connectivity in rural areas; safety around people and animals; data ownership; and regulation of driverless machines on public roads between fields.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Agricultural_Robots]])
```
