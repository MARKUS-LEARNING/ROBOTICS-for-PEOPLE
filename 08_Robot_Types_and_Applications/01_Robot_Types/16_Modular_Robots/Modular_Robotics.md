---
title: Modular Robotics
description: "Modular robots are built from standard, interchangeable modules: actuators, links, sensors, end effectors or whole robot units. Covers the two meanings of modularity (engineering modularity, as in actuator modules and tool changers, and self-reconfigurable modular robots), benefits and costs, a computed example of how module mass limits the length of a modular arm, and modular design in 2026 robot platforms."
aliases:
  - Modular Robots
  - Modular Robot
tags:
  - modularity
  - actuators
  - robot-design
type: concept
level: intermediate
subfield:
  - modular-reconfigurable
  - mechanisms-actuation
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Robots]]"
  - "[[Manipulator_Arm]]"
related:
  - "[[Reconfigurable_Robots]]"
  - "[[Swarm_Robots]]"
  - "[[ros2_control]]"
  - "[[Robot_Hands]]"
key_takeaways:
  - "Modularity means building robots from standard parts with standard interfaces (mechanical, electrical, software), so they can be assembled, repaired and changed quickly."
  - "Modules carry their own mass and the arm must lift it: with 1 kg modules spaced 0.15 m apart and 20 N·m at the base, a horizontal arm can hold only four modules."
  - "In 2026 modularity shows up as integrated actuator modules (motor, gearbox, driver, sensors in one unit), quick-change end effectors, and software modularity (ROS 2 packages, ros2_control plugins); self-reconfiguring modular robots remain mostly in research."
sources:
  - "Yim, M. et al. (2007). Modular Self-Reconfigurable Robot Systems [Grand Challenges of Robotics]. https://ieeexplore.ieee.org/document/4141037"
  - "Seo, J., Paik, J. and Yim, M. (2019). Modular Reconfigurable Robotics. https://www.annualreviews.org/doi/10.1146/annurev-control-053018-023834"
  - "HEBI Robotics (2026). Actuators. https://www.hebirobotics.com/actuators"
  - "ros-controls (2026). ros2_control. https://github.com/ros-controls/ros2_control"
---

# Modular Robotics

A **modular robot** is assembled from standard building blocks with standard connections, much like LEGO. Instead of designing every robot from scratch, engineers combine existing modules: joints, links, sensors, grippers, even complete mobile units. Modularity makes robots faster to build, easier to repair and possible to reconfigure.

> **Etymology.** Latin *modulus*, "small measure", the diminutive of *modus*, "measure". In engineering, a *module* is a self-contained unit with defined interfaces.

## Two meanings

| Meaning | What is modular | Examples |
|---|---|---|
| **Engineering modularity** | Components with standard interfaces, combined at design or setup time | Integrated actuator modules; tool changers; mobile bases with swappable top modules; ROS 2 packages |
| **Self-reconfigurable modular robots** | Identical robotic units that connect and rearrange *themselves* into different shapes | M-TRAN, SMORES, M-Blocks; see [[Reconfigurable_Robots]] |

## Benefits and costs

| Benefits | Costs |
|---|---|
| Faster design and prototyping | Interfaces add mass, volume and cost |
| Repair by swapping a module | Connectors are weak points (stiffness, wear, signal integrity) |
| Reuse across products and tasks | Less optimised than a purpose-built design |
| Scales production of one part | Need for standards across vendors |

## Worked example: how long can a modular arm be?

Build a horizontal arm from identical 1 kg joint modules spaced 0.15 m apart. The base joint must hold the weight of every module beyond it, each at its own distance:

$$\tau_{base} = \sum_{i=1}^{n} m g\,(0.15\,i) = 9.81 \times 0.15 \times \frac{n(n+1)}{2}$$

| Modules $n$ | Base torque (N·m) |
|---|---|
| 3 | 8.8 |
| 4 | 14.7 |
| 5 | 22.1 |
| 6 | 30.9 |

If each module can produce 20 N·m, the arm can hold out **four** modules horizontally, with 5.3 N·m left for payload. The fifth exceeds the base's capacity before carrying anything. This is the central trade-off of modular arms: identical modules are convenient, but the joints near the base need far more torque than those at the tip. Real designs mix module sizes, larger near the base, which is why actuator families come in several torque classes.

## Modularity in 2026

- **Integrated actuator modules**: motor, gearbox, motor driver, encoders and sometimes torque sensing in one unit with a single bus connection, used in humanoids, quadrupeds and research arms.
- **Quick-change tooling**: standard flanges and automatic tool changers let one arm switch grippers, screwdrivers and cameras.
- **Mobile manipulation kits**: bases, arms and sensor heads from different vendors combined through common interfaces.
- **Software modularity**: ROS 2 packages, [[ros2_control]] hardware plugins and standard message types let modules from different vendors work together.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Modular_Robotics]])
```
