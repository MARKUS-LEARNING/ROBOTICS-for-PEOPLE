---
title: Underwater and Space Robots
description: "Robots for the two environments people can least reach: the deep ocean and space. Covers ROVs and AUVs, planetary rovers, orbital manipulators and free-flyers, why both domains force autonomy (communication delays computed for Mars and the deep sea), navigation without GPS, Ingenuity's 72 flights on Mars, and current directions in 2026."
aliases:
  - Underwater Robots
  - Space Robots
  - AUV
  - ROV
  - Planetary Rovers
tags:
  - underwater-robotics
  - space-robotics
  - autonomous-systems
  - navigation
type: robot-type
level: beginner
subfield:
  - marine-robotics
  - space-robotics
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Mobile_Robots]]"
related:
  - "[[Autonomous_Robots]]"
  - "[[Drones]]"
  - "[[SLAM]]"
  - "[[Localization]]"
key_takeaways:
  - "Underwater and in space, radio-quality real-time control is unavailable: water blocks radio, and distance delays signals by seconds to tens of minutes, so these robots must act autonomously between human check-ins."
  - "Underwater robots are tethered ROVs (human-piloted, unlimited power) or untethered AUVs (autonomous, battery-limited); navigation fuses inertial sensors, Doppler velocity logs and acoustic positioning."
  - "Space robots range from Mars rovers and the Ingenuity helicopter (72 flights, 2021–2024) to station arms and free-flyers; autonomy and in-space servicing are the growth areas."
sources:
  - "NASA (2026). Ingenuity Mars Helicopter. https://science.nasa.gov/mission/mars-2020-perseverance/ingenuity-mars-helicopter/"
  - "NASA (2026). Astrobee. https://www.nasa.gov/astrobee/"
  - "Siciliano, B. and Khatib, O., eds. (2016). Springer Handbook of Robotics, 2nd edition (chapters on underwater and space robotics). https://link.springer.com/referencework/10.1007/978-3-319-32552-1"
  - "Woods Hole Oceanographic Institution (2026). Underwater vehicles. https://www.whoi.edu/what-we-do/explore/underwater-vehicles/"
---

# Underwater and Space Robots

The deep ocean and space share three features that make them robot territory: they are deadly for people, expensive to reach, and hard to communicate with. Robots there must survive extreme pressure, cold, radiation or vacuum, and must often decide for themselves because humans cannot react in time.

> **Etymology.** *ROV* stands for *remotely operated vehicle*, *AUV* for *autonomous underwater vehicle*. *Rover* is from Middle Dutch *roven*, "to rob", later "to wander": the wandering machines of planetary exploration, from the Soviet Lunokhod 1 (1970) to NASA's Perseverance (2021).

## Underwater robots

| Type | How | Uses |
|---|---|---|
| **ROV** | Tethered to a ship, piloted live; power and video through the cable | Offshore energy, pipeline repair, deep-sea science, salvage |
| **AUV** | Untethered, battery-powered, follows a mission autonomously | Seafloor mapping, surveys, under-ice and mine-countermeasure work |
| **Gliders** | Change buoyancy to glide, for months at a time | Ocean monitoring over thousands of kilometres |
| **Hybrid (HROV)** | Switch between tethered and autonomous modes | Flexible deep-sea work |

**Navigation without GPS.** Radio and GPS do not penetrate water. AUVs combine:
- inertial navigation;
- a **Doppler velocity log** measuring speed over the seafloor;
- depth sensors;
- **acoustic positioning** from ships or seafloor beacons.

Sonar replaces lidar for mapping, and SLAM on sonar or camera data reduces drift near structures (see [[SLAM]]).

## Space robots

| Type | Examples | Notes |
|---|---|---|
| **Planetary rovers** | Sojourner, Spirit, Opportunity, Curiosity, Perseverance (Mars); Chinese Yutu and Zhurong rovers | Drive semi-autonomously between ground commands; sample collection |
| **Aerial explorers** | Ingenuity Mars Helicopter | Planned as 5 test flights in 30 days; completed 72 flights over nearly three years before damage ended its mission in January 2024 |
| **Orbital manipulators** | Canadarm2 on the ISS; Canadarm3 planned for lunar orbit | Capture visiting vehicles, assemble and maintain structures |
| **Free-flyers** | NASA's Astrobee robots on the ISS | Research platforms for in-cabin autonomy |
| **Servicing** | Satellite life-extension and refuelling missions | A growing commercial field |

## Worked example: why they must be autonomous

Signals travel at the speed of light, $c \approx 300{,}000$ km/s.

- **Mars**, at closest approach (about 54.6 million km), is $54.6\times10^6 / 299{,}792 \approx 182$ s ≈ **3 minutes** away one way; at its farthest (about 401 million km) it is **22 minutes** away. A rover driver's command and the image that shows its result are separated by 6 to 45 minutes. Rovers therefore navigate hazards autonomously and are commanded once or twice per Martian day.
- **The Moon** is 1.3 s away, so teleoperation is awkward but possible.
- **The deep sea**: sound travels about 1,500 m/s, so a message to an AUV 3 km down takes **2 s** one way, at data rates of only kilobits per second. No live video is possible without a tether.

Communication shapes the whole design. The robot must handle anything that happens between messages, and must keep itself safe when uncertain.

## Directions in 2026

- **More onboard autonomy**: hazard avoidance, science-target selection and fault recovery for rovers and AUVs, increasingly using learned perception.
- **Ocean data at scale**: fleets of gliders and AUVs for climate and seabed mapping.
- **In-space servicing and assembly**: robotic arms that refuel, repair and build in orbit.
- **Lunar programmes**: robotic landers and rovers preparing for crewed missions.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Underwater_and_Space_Robots]])
```
