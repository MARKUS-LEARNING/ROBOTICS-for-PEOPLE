---
title: Cable-Driven Robots
description: "Cable-driven robots move by pulling cables or tendons: cable-driven parallel robots that hang a platform from winches (stadium cameras, the FAST radio telescope's feed cabin), and tendon-driven arms and hands with motors kept away from the joints. Covers why cables help, the rule that cables can only pull, a computed tension example showing why near-horizontal cables are a problem, and uses."
aliases:
  - Cable-Driven Parallel Robots
  - CDPR
  - Tendon-Driven Robots
tags:
  - parallel-robots
  - actuators
  - mechanism-design
type: robot-type
level: intermediate
subfield:
  - mechanisms-actuation
  - kinematics-dynamics
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Parallel_Robots]]"
related:
  - "[[Robot_Hands]]"
  - "[[Soft_Robotics]]"
  - "[[Manipulator_Arm]]"
key_takeaways:
  - "Cables are light, cheap and long, so cable robots can span stadiums and telescopes, and tendon-driven limbs can keep heavy motors in the body."
  - "Cables can only pull, never push: every pose must be reachable with all tensions positive, which shapes the usable workspace."
  - "Tension rises steeply as cables flatten: holding 10 kg with two cables at 60° from vertical needs 98 N each, at 80° it needs 282 N."
sources:
  - "Pott, A. (2018). Cable-Driven Parallel Robots: Theory and Application. https://link.springer.com/book/10.1007/978-3-319-76138-1"
  - "Nan, R. et al. (2011). The Five-Hundred-Meter Aperture Spherical Radio Telescope (FAST) Project. https://arxiv.org/abs/1105.3794"
  - "Merlet, J.-P. (2006). Parallel Robots, 2nd edition. https://link.springer.com/book/10.1007/1-4020-4133-0"
  - "Shaw, K., Agarwal, A. and Pathak, D. (2023). LEAP Hand: Low-Cost, Efficient, and Anthropomorphic Hand for Robot Learning. https://arxiv.org/abs/2309.06440"
---

# Cable-Driven Robots

**Cable-driven robots** move by pulling on cables (also called tendons or wires) wound on motorised winches or pulleys. The idea appears in two very different forms: giant **cable-driven parallel robots** that suspend a platform from several winches, and **tendon-driven** arms and hands whose motors sit far from the joints they move, as our forearm muscles move our fingers.

> **Etymology.** *Cable* is from Late Latin *capulum*, "lasso, halter". *Tendon* is from Latin *tendere*, "to stretch".

## Two families

| Family | How | Examples |
|---|---|---|
| **Cable-driven parallel robots (CDPRs)** | 3–8+ cables from fixed winches to a moving platform; changing cable lengths moves the platform | Skycam and Spidercam over sports fields; the feed cabin of China's 500 m FAST radio telescope; large-scale 3D printing and warehouse handling |
| **Tendon-driven mechanisms** | Cables routed through a limb pull on joints; motors in the base or body | Dexterous robot hands, lightweight arms, continuum and surgical instruments, cable-driven exosuits |

## Why cables?

- **Light and long.** A few kilograms of cable can span hundreds of metres, giving workspaces no rigid robot could match.
- **Fast.** Very low moving mass.
- **Motors stay home.** Heavy actuators sit in the base, making limbs light and safe (low inertia).
- **Cheap to scale.** More winches for more payload or more control.

## The rule: cables only pull

A cable can carry tension but never compression. A CDPR with $m$ cables controlling $n$ degrees of freedom needs **positive tension in every cable** at every pose; this defines the *wrench-feasible workspace*. Fully controlling all $n$ degrees of freedom with positive tensions usually needs at least $n + 1$ cables (7 for full 6-DoF control), or gravity acting as the "extra cable".

## Worked example: why flat cables are trouble

A 10 kg platform hangs from two symmetric cables, each at angle $\theta$ from the vertical. Vertical balance gives $2T\cos\theta = mg$, so

$$T = \frac{mg}{2\cos\theta}$$

| Angle from vertical | Tension per cable |
|---|---|
| 30° | 56.6 N |
| 60° | 98.1 N |
| 80° | 282.5 N |

As the cables approach horizontal, tension shoots up ($\cos\theta \to 0$). Near the edges of a cable robot's workspace, winches and cables overload. That is why CDPR workspaces are kept well inside their anchor points, and why winches are mounted high above the work area.

## Challenges

- **Cable elasticity and sag**, which reduce stiffness and precision on long spans.
- **Collisions** between cables and objects or people.
- **Friction and wear** in tendon routing (hands and arms); tendon tension must be maintained.
- **Calibration** of cable lengths and anchor positions.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Cable-Driven_Robots]])
```
