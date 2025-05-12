---
title: Underwater and Space Robots
description: Overview of robotic systems designed for operation in extreme underwater and space environments, covering types (ROVs, AUVs, Rovers, Orbital Arms), technologies, applications, and challenges.
tags:
  - robot-types
  - underwater-robot
  - space-robot
  - AUV
  - ROV
  - rover
  - orbital-robot
  - extreme-environments
  - exploration
  - field-robotics
layout: default
category: robotics
author: Jordan_Smith_&_le_Chat
date: 2025-04-28
permalink: /underwater_and_space_robots/
related:
  - "[[Robots]]"
  - "[[Mobile_Robots]]"
  - "[[Industrial_Arms]]"
  - "[[AUV]]"
  - "[[ROV]]"
  - "[[Planetary_Rover]]"
  - "[[Space_Robotics]]"
  - "[[Extreme_Environments]]"
  - "[[Navigation]]"
  - "[[Localization]]"
  - "[[Manipulation]]"
  - "[[Autonomy]]"
  - "[[Robot_Types_and_Applications]]"
---

# Underwater and Space Robots

Underwater and space robots represent specialized classes of robotic systems engineered to operate in extreme environments that are largely inaccessible, inhospitable, or hazardous to humans. These domains – the deep sea and outer space – share common challenges such as remote operation, limited or delayed communication, harsh physical conditions (extreme pressure or vacuum, temperature fluctuations, radiation), the need for high reliability, and constraints on power and autonomy.

---

## Underwater Robots

These robots operate beneath the surface of oceans, lakes, or rivers for various tasks.

* **Types:**
    * **[[ROV|Remotely Operated Vehicles (ROVs)]]:** These are typically unoccupied underwater robots connected to a surface vessel or platform via a tether (umbilical cable). The tether provides power and high-bandwidth communication for real-time control by human operators and sensor data transmission (e.g., video). ROVs are workhorses in the offshore industry and scientific exploration, often equipped with [[Manipulation|manipulator arms]], cameras, lights, sonar, and specialized tools for inspection, maintenance, repair (IMR), construction support, object retrieval, and scientific sampling.
    * **[[AUV|Autonomous Underwater Vehicles (AUVs)]]:** These are untethered, self-powered robots that operate autonomously based on pre-programmed mission plans or adaptive algorithms. They navigate independently and collect data using onboard sensors. Primary applications include large-area seafloor mapping (bathymetry, side-scan sonar), oceanographic surveys (measuring water properties), environmental monitoring, and military reconnaissance (e.g., mine countermeasures). Challenges include underwater [[Navigation]] (no GPS, reliance on [[IMU_Sensors|Inertial Navigation Systems - INS]], Doppler Velocity Logs - DVL, acoustic positioning like LBL/USBL), limited acoustic communication bandwidth, and energy endurance.
    * **Hybrid ROV/AUVs (HROVs):** Combine features of both, potentially operating autonomously for transit/survey but connecting via a lightweight tether or docking for manipulation or high-bandwidth data transfer.
    * **Underwater Gliders:** Use changes in buoyancy and wings to achieve long-duration, low-power gliding motion for persistent ocean monitoring over vast areas.

* **Key Technologies:** Pressure-resistant housings and components, buoyancy control systems, thrusters/propellers, acoustic modems and positioning systems, sonar (imaging, mapping), Doppler Velocity Logs (DVLs), Inertial Navigation Systems (INS), underwater cameras and lighting, specialized chemical/biological sensors, manipulators (mainly ROVs).
* **Applications:** Offshore oil & gas (IMR, pipeline survey), oceanographic science, underwater archaeology, defense and security, aquaculture.

---

## Space Robots

These robots operate beyond Earth's atmosphere, either in orbit or on the surfaces of celestial bodies.

* **Types:**
    * **Orbital Manipulators:** Large [[Industrial_Arms|robotic arms]] based on spacecraft or space stations. Examples include the Shuttle Remote Manipulator System (SRMS or Canadarm), and the systems currently on the International Space Station (ISS): Canadarm2 (SSRMS), Dextre (SPDM), and the Japanese Experiment Module arm (JEMRMS). They are used for station assembly, external maintenance, handling cargo, capturing visiting spacecraft, and assisting astronauts during spacewalks (EVAs).
    * **[[Planetary Rover|Planetary Rovers]]:** [[Mobile_Robots|Mobile robots]] designed to traverse the surfaces of the Moon, Mars, or other bodies. Examples include the Soviet Lunokhod lunar rovers, NASA's Mars rovers (Sojourner, Spirit, Opportunity, Curiosity, Perseverance). They face challenges of extreme temperatures, radiation, dust, varying gravity, rough terrain, communication delays, and long-term reliability. They employ specialized locomotion systems (e.g., rocker-bogie suspension) and carry scientific instruments for in-situ analysis and sample collection.
    * **Planetary Landers:** Stationary platforms that deliver instruments to the surface (e.g., Viking, Phoenix Mars landers). Some may deploy smaller mobile elements (like Sojourner from the Mars Pathfinder lander).
    * **Free-Flying Robots:** Smaller robotic platforms designed to operate autonomously or via teleoperation inside or outside spacecraft, assisting astronauts or performing inspections (e.g., SPHERES and Astrobee on the ISS).

* **Key Technologies:** Radiation-hardened electronics, advanced materials for thermal stability and structural integrity, high-efficiency power systems (solar arrays, Radioisotope Thermoelectric Generators - RTGs), reliable mechanisms (actuators, deployment systems), autonomous [[Navigation]] and [[SLAM]] for rovers, robust long-range communication systems (e.g., Deep Space Network), advanced robotic arms and sampling tools for manipulators and rovers.
* **Applications:** Planetary science and geology, astrobiology (search for life), satellite servicing (refueling, repair, assembly - e.g., Orbital Express demo), space station construction and maintenance, astronaut support, potential future applications like space debris removal and in-situ resource utilization (ISRU).

---

## Common Challenges

Both underwater and space robotics operate at the frontier of technology and face overlapping challenges:

* **Extreme Environments:** High pressure/vacuum, vast temperature ranges, radiation, corrosive fluids, abrasive dust.
* **Remote Operation & [[Autonomy]]:** Limited or delayed communication necessitates varying degrees of autonomy for decision-making, navigation, and fault recovery.
* **Communication Constraints:** Bandwidth limitations (especially acoustic underwater) and significant signal latency (due to vast distances in space) impact real-time control and data return.
* **Reliability:** Systems must operate reliably for long durations (months or years) with little or no possibility for manual repair. Fault detection, isolation, and recovery (FDIR) capabilities are critical.
* **Power:** Generating and managing sufficient power from limited sources (batteries, solar, RTGs, tethers) for extended missions is a constant challenge.
* **Navigation & [[Localization]]:** Determining position and orientation accurately without relying on standard terrestrial infrastructure like GPS.

These demanding fields push the boundaries of robotics, requiring highly robust, intelligent, and often autonomous systems to explore and operate in Earth's final frontiers.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #robotics OR #automation WHERE contains(file.outlinks, [[Robots]])