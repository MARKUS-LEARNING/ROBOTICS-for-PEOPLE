---
title: OMPL
description: "OMPL (Open Motion Planning Library) is the standard open-source library of sampling-based motion planners, used by MoveIt and many research systems. Covers what it provides and what it deliberately leaves out, the main planners, a minimal Python example, OMPL 2.0 (April 2026: rewritten Python bindings installable with pip, VAMP vectorised collision checking, new planners), and how it compares with GPU planners."
aliases:
  - Open Motion Planning Library
tags:
  - motion-planning
  - path-planning
  - robotics-software
type: tool
level: intermediate
subfield:
  - planning
  - software
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Path_Planning_Algorithms]]"
related:
  - "[[MoveIt2]]"
  - "[[C++_Motion_Planning]]"
  - "[[Sampling-Based_Planning]]"
  - "[[Probabilistic_Roadmaps]]"
key_takeaways:
  - "OMPL implements dozens of sampling-based planners (RRT, RRT-Connect, PRM, RRT*, BIT*, AIT*, EIT* and more) behind one interface; you supply the state space and a validity (collision) checker."
  - "OMPL 2.0 (April 2026) rewrote the Python bindings (pip install ompl) and added VAMP, a SIMD-vectorised collision-checking backend reported to plan at up to 25 kHz for common arms."
  - "It is the default planning backend of MoveIt; GPU trajectory optimisers (cuRobo) complement it for fast, smooth arm motions."
sources:
  - "Şucan, I. A., Moll, M. and Kavraki, L. E. (2012). The Open Motion Planning Library. https://ieeexplore.ieee.org/document/6377468"
  - "Kavraki Lab and contributors (2026). OMPL repository and release notes (2.0.0, 2.0.2). https://github.com/ompl/ompl"
  - "Thomason, W., Kingston, Z. and Kavraki, L. E. (2023). Motions in Microseconds via Vectorized Sampling-Based Planning. https://arxiv.org/abs/2309.14545"
---

# OMPL

**OMPL**, the Open Motion Planning Library, is a C++ library (with Python bindings) of sampling-based motion planners. It is the planning engine behind [[MoveIt2]] and countless research projects.

> **Etymology.** Open Motion Planning Library, from Lydia Kavraki's lab at Rice University (2010 onward), described by Şucan, Moll and Kavraki in 2012.

## What it provides, and what it leaves out

OMPL provides **planners** and **state spaces** (real vectors, SO(2), SO(3), SE(2), SE(3), compound spaces, Dubins and Reeds–Shepp curves). It deliberately does **not** include collision checking, robot models or visualisation. You supply a **state validity checker** (for example via FCL or MoveIt's planning scene). This separation is why OMPL fits so many robots.

## Main planners

| Planner | Character |
|---|---|
| RRT, RRT-Connect | Fast single queries; RRT-Connect is the usual default for arms |
| PRM, PRM* | Reusable roadmaps for repeated queries |
| RRT*, Informed RRT*, BIT*, AIT*, EIT* | Asymptotically optimal; improve the path with more time |
| KPIECE, EST, SBL | Exploration-oriented; useful in difficult spaces |
| Kinodynamic planners | Respect dynamics through controls (RRT, KPIECE with controls) |
| New in 2.0: AORRTC, BLIT* and others | Faster optimal planning |

## A minimal Python example

```python
from ompl import base as ob, geometric as og

space = ob.RealVectorStateSpace(2)
bounds = ob.RealVectorBounds(2); bounds.setLow(0); bounds.setHigh(10)
space.setBounds(bounds)

def is_valid(s):                      # a circular obstacle centred at (5, 5), radius 2
    return (s[0] - 5) ** 2 + (s[1] - 5) ** 2 > 4

ss = og.SimpleSetup(space)
ss.setStateValidityChecker(ob.StateValidityCheckerFn(is_valid))
start, goal = ob.State(space), ob.State(space)
start[0], start[1], goal[0], goal[1] = 1, 1, 9, 9
ss.setStartAndGoalStates(start, goal)
if ss.solve(1.0):                      # plan for up to 1 s
    ss.simplifySolution()
    print(ss.getSolutionPath())
```

The exact Python API changed with the 2.0 rewrite of the bindings, so check the current examples in the repository.

## OMPL 2.0 (2026)

From the release notes (2.0.0, April 2026):
- **rewritten Python bindings**, enabled by default and installable with `pip install ompl`;
- **VAMP integration**: a vectorised (SIMD) backend for forward kinematics and collision checking, reported to reach planning speeds of up to 25 kHz for common arms (Panda, UR5, Fetch, Baxter);
- **new planners**, including AORRTC and BLIT*.

Version 1.7 (2025) had added Effort Informed Trees (EIT*) and dropped the old ODE and MORSE integrations.

## OMPL and GPU planners

Sampling-based planning is fast to find *a* path but usually needs smoothing and time-parameterisation. GPU trajectory optimisers such as cuRobo produce smooth, minimum-jerk motions in tens of milliseconds. Vectorised CPU planning (VAMP) has made OMPL competitive for speed again. Many stacks use both: OMPL for hard, cluttered problems, optimisers for smoothness and reactivity (see [[C++_Motion_Planning]]).

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[OMPL]])
```
