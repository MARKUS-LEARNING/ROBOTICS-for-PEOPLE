---
title: Parallel Robots
description: "Parallel robots connect a moving platform to a base through several kinematic chains working together: Delta robots for ultra-fast picking, Gough–Stewart hexapods for motion simulators and precision positioning. Covers why parallel structures are stiff and fast, the inverted difficulty of their kinematics (inverse easy, forward hard), a computed leg-length example, singularities, and uses in 2026."
aliases:
  - Parallel Robot
  - Parallel Manipulator
  - Delta Robot
  - Stewart Platform
  - Hexapod
tags:
  - parallel-robots
  - kinematics
  - industrial-robot
type: robot-type
level: intermediate
subfield:
  - kinematics-dynamics
  - industrial-automation
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Manipulator_Arm]]"
  - "[[Inverse_Kinematics]]"
related:
  - "[[Cable-Driven_Robots]]"
  - "[[Industrial_Arms]]"
  - "[[Forward_Kinematics]]"
key_takeaways:
  - "Several chains share the load, so parallel robots are stiff, accurate and fast for their mass, at the cost of a smaller workspace than serial arms."
  - "Their kinematics invert serial arms' difficulty: inverse kinematics is easy (each leg length follows directly from the platform pose), forward kinematics is hard (a Stewart platform can have up to 40 solutions)."
  - "Delta robots dominate high-speed light picking; hexapods drive flight and driving simulators and precision positioning in optics and astronomy."
sources:
  - "Merlet, J.-P. (2006). Parallel Robots, 2nd edition. https://link.springer.com/book/10.1007/1-4020-4133-0"
  - "Clavel, R. (1990). Device for the movement and positioning of an element in space (Delta robot patent US 4,976,582). https://patents.google.com/patent/US4976582A/en"
  - "Lynch, K. M. and Park, F. C. (2017). Modern Robotics, chapter 7: Kinematics of Closed Chains. https://hades.mech.northwestern.edu/index.php/Modern_Robotics"
  - "Siciliano, B. and Khatib, O., eds. (2016). Springer Handbook of Robotics, 2nd edition, chapter on parallel mechanisms. https://link.springer.com/referencework/10.1007/978-3-319-32552-1"
---

# Parallel Robots

A **parallel robot** holds its end effector (a platform) with several chains of links at once, each anchored to the base. A serial arm is a single chain, like a human arm; a parallel robot is more like a table held up by adjustable legs. Sharing the load across chains makes it stiff, precise and fast.

> **Etymology.** *Parallel* from Greek *parallēlos*, "beside one another": the chains work side by side. Eric Gough built a hexapod tyre-testing machine in the 1950s; D. Stewart described a six-legged flight-simulator platform in 1965 (hence "Gough–Stewart platform"). Reymond Clavel invented the **Delta robot** at EPFL in the 1980s.

## Common types

| Type | Structure | Strengths | Uses |
|---|---|---|---|
| **Delta** | Three arms with parallelograms; motors fixed in the base | Very light moving parts; several picks per second | Food, pharmaceutical and electronics packaging |
| **Gough–Stewart (hexapod)** | Six extensible legs between base and platform | Six-DoF motion, high stiffness and accuracy | Flight and driving simulators, telescope mirror and optics positioning, machining |
| **Planar parallel** | Chains moving in a plane | Simple, precise | Positioning stages |
| **Cable-driven parallel** | Cables instead of rigid legs | Huge workspaces | See [[Cable-Driven_Robots]] |

## Serial versus parallel

| | Serial arm | Parallel robot |
|---|---|---|
| Workspace | Large | Smaller |
| Stiffness and accuracy | Errors add up along the chain | Errors are averaged across legs; high stiffness |
| Speed | Each motor carries the ones after it | Motors fixed at the base; light moving parts |
| Inverse kinematics | Hard (multiple solutions, iterative) | **Easy** |
| Forward kinematics | **Easy** | Hard |

## Worked example: inverse kinematics of a hexapod leg

For a Gough–Stewart platform, each leg connects a base point $\mathbf b_i$ to a platform point $\mathbf p_i$ (fixed in the platform frame). Given the platform's position $\mathbf t$ and rotation $R$, each leg length is simply

$$\ell_i = \lVert \mathbf t + R\,\mathbf p_i - \mathbf b_i \rVert.$$

Example: base point $\mathbf b = (1, 0, 0)$ m, platform point $\mathbf p = (0.5, 0, 0)$ m, platform level ($R = I$) and centred 1 m above the base ($\mathbf t = (0, 0, 1)$):

$$\ell = \lVert (0.5, 0, 1) - (1, 0, 0) \rVert = \lVert(-0.5, 0, 1)\rVert = \sqrt{0.25 + 1} = 1.118\ \text{m}.$$

Do this for six legs and you have the inverse kinematics, in closed form and in microseconds. Going the other way, finding the pose from six measured leg lengths, means solving coupled nonlinear equations. The general Stewart platform has up to 40 solutions, so controllers use numerical methods started from the last known pose.

## Singularities

At some poses a parallel robot gains an uncontrollable motion: the platform can move slightly even with all actuators locked, or it loses stiffness in some direction. Designers keep the working range away from these configurations.

## Uses in 2026

Delta robots remain the standard for high-speed picking of light items, now guided by deep-learning vision. Hexapods position optics, telescope secondary mirrors and test equipment with micrometre precision, and move motion simulators for driving and flight training. Parallel designs also appear in humanoid ankles and wrists, where two motors drive a joint through parallel linkages to keep mass close to the body.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Parallel_Robots]])
```
