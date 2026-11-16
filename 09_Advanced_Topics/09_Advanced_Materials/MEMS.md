---
title: Microelectromechanical Systems (MEMS)
description: "MEMS are machines micrometres in size made with chip-fabrication methods: accelerometers, gyroscopes, pressure sensors, microphones, micromirrors and micro-actuators. Covers how they are made and how a MEMS gyroscope works, a computed drift example showing why IMUs need fusion, their role in every 2026 robot, and MEMS actuators for microrobots."
aliases:
  - MEMS
  - Microelectromechanical Systems
  - MEMS Sensors
tags:
  - sensors
  - materials
  - electronics
type: concept
level: intermediate
subfield:
  - electronics-embedded
  - micro-nano
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Sensors]]"
related:
  - "[[IMU_Sensors]]"
  - "[[Advanced_Sensor_Fusion]]"
  - "[[Nanotechnology]]"
  - "[[Smart_Materials]]"
key_takeaways:
  - "MEMS put moving mechanical structures on silicon chips, which made accelerometers, gyroscopes and microphones cheap and tiny enough for every phone, drone and robot."
  - "A MEMS gyroscope measures rotation through the Coriolis force on a vibrating mass; a bias of 10 °/h integrates to 0.17° of heading error per minute, which is why IMUs are fused with other sensors."
  - "MEMS actuators (electrostatic comb drives, micromirrors, piezo films) power optics, lidar scanners and microrobots."
sources:
  - "Bosch Sensortec (2026). Inertial measurement units (MEMS). https://www.bosch-sensortec.com/products/motion-sensors/imus/"
  - "Zhang, L. et al. (2021). A comparative review of artificial muscles for microsystem applications. https://www.nature.com/articles/s41378-021-00323-5"
  - "Feynman, R. P. (1959). There's Plenty of Room at the Bottom. https://web.pa.msu.edu/people/yang/RFeynman_plentySpace.pdf"
---

# Microelectromechanical Systems (MEMS)

**MEMS** are mechanical devices, such as beams, springs, membranes and moving masses, built at the scale of micrometres using the same lithography and etching processes that make computer chips. They turned motion and pressure sensors from precision instruments into components costing cents, and made drones, phones and modern robots possible.

> **Etymology.** *Micro-electro-mechanical systems*, a term from the late 1980s. Silicon accelerometers for car airbags in the early 1990s were the first mass market.

## How they are made

Silicon wafers are patterned by photolithography and etched (deep reactive-ion etching) to release free-standing structures, sometimes bonded to other wafers and sealed in a vacuum. Thousands of devices are made at once on one wafer, which is why they are cheap.

## Common MEMS devices in robots

| Device | Principle | Robot use |
|---|---|---|
| **Accelerometer** | Proof mass on springs; displacement sensed capacitively | Tilt, vibration, impacts; part of every IMU |
| **Gyroscope** | Vibrating mass; rotation causes a Coriolis force sensed as motion in another axis | Orientation, balance, stabilisation |
| **Pressure sensor** | Membrane deflection | Altitude in drones, pneumatic actuators, touch |
| **Microphone** | Membrane | Speech interfaces, acoustic sensing |
| **Micromirror** | Electrostatic or electromagnetic tilting mirror | Scanning lidar, projectors |
| **Actuators** | Electrostatic comb drives, thermal, piezo films | Micro-positioning, microrobots |

## Worked example: why gyros need help

Integrating a gyroscope's rate gives orientation, but its small constant error (**bias**) integrates too. With a residual bias of 10 °/h, typical of good consumer-grade parts after calibration:

57905\text{heading error after 60 s} = 10 \times \frac{60}{3600} = 0.17°.57905

After 10 minutes it is 1.7°, and real biases also wander with temperature. A walking robot or drone therefore fuses the gyro with accelerometers (gravity fixes roll and pitch), magnetometers, cameras or lidar (to fix heading and position) in a filter (see [[Advanced_Sensor_Fusion]]). Tactical- and navigation-grade gyros are far more stable but cost orders of magnitude more.

## MEMS in 2026

Every robot carries MEMS IMUs, usually several. Six-axis IMUs with built-in sensor-fusion processors are standard parts, MEMS mirrors steer beams in solid-state lidar, and MEMS microphones arrays localise sound. At the frontier, MEMS fabrication builds insect-scale robots and the actuators of microrobots (see [[Nanorobots]]).

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[MEMS]])
```
