---
title: Formal Methods in Control
description: "Formal methods give mathematical guarantees about what a control system will and will not do: temporal-logic specifications (LTL, STL), model checking, correct-by-construction synthesis and reachability analysis. Includes a worked reachable-set example, how formal methods verify the safety layers around learned components, and language models translating instructions into checkable specifications."
aliases:
  - Formal Verification
  - Temporal Logic
  - LTL
  - Reachability Analysis
tags:
  - verification
  - safety
  - control
type: concept
level: research
subfield:
  - control
  - planning
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Hybrid_Systems_and_Control]]"
related:
  - "[[Knowledge_Representation_and_Reasoning]]"
  - "[[Collision_Avoidance]]"
  - "[[Ethical_and_Social_Implications]]"
  - "[[Hamilton-Jacobi-Bellman_Equation]]"
key_takeaways:
  - "Formal methods state requirements precisely (for example in temporal logic) and then prove, rather than test, that a system meets them."
  - "Reachability analysis computes every state a system could reach: a robot with speed at most 1 m/s can be anywhere within 2 m of its start after 2 s, so a safety zone must cover that."
  - "Proving a whole learned robot correct is out of reach; the practical 2026 pattern is to verify the safety layer (reachability sets, barrier-function filters, verified switching logic) that the learned components run inside."
sources:
  - "Belta, C., Yordanov, B. and Aydin Gol, E. (2017). Formal Methods for Discrete-Time Dynamical Systems. https://link.springer.com/book/10.1007/978-3-319-50763-7"
  - "Kress-Gazit, H., Lahijanian, M. and Raman, V. (2018). Synthesis for Robots: Guarantees and Feedback for Robot Behavior. https://www.annualreviews.org/doi/10.1146/annurev-control-060117-104838"
  - "Bansal, S., Chen, M., Herbert, S. and Tomlin, C. J. (2017). Hamilton-Jacobi Reachability: A Brief Overview and Recent Advances. https://arxiv.org/abs/1709.07523"
  - "Ames, A. D. et al. (2019). Control Barrier Functions: Theory and Applications. https://arxiv.org/abs/1903.11199"
---

# Formal Methods in Control

Testing shows that a system works in the cases you tried. **Formal methods** prove properties for *all* cases covered by a model: "the robot never enters the forbidden zone", "every request is eventually served", "the gripper never opens while carrying a load above the table". For safety-critical robots, that difference matters.

> **Etymology.** *Formal*: expressed in a precise mathematical language. Temporal logic was brought into computer science by Amir Pnueli in 1977 for reasoning about programs over time, work that earned him the Turing Award. Its use for robot control grew in the 2000s.

## Specifying what "correct" means

**Linear temporal logic (LTL)** adds time operators to logic: **G** (always), **F** (eventually), **X** (next) and **U** (until). For example,

$$\mathbf G\,\neg \text{collision} \ \wedge\ \mathbf G\,(\text{request} \rightarrow \mathbf F\,\text{delivered})$$

means "never collide, and every request is eventually delivered". **Signal temporal logic (STL)** adds real-valued, timed conditions, for example "always keep distance > 0.5 m, and reach the goal within 30 s", and gives a *robustness* score saying by how much a trajectory satisfies or violates a specification.

## The main techniques

| Technique | Question | Robotics use |
|---|---|---|
| **Model checking** | Does this finite model satisfy the specification? | Verifying mission and switching logic, behaviour trees |
| **Controller synthesis** | Build a controller that is correct by construction | High-level task controllers from LTL specifications |
| **Reachability analysis** | What states can the system possibly reach? | Safety envelopes, collision guarantees |
| **Theorem proving** | Prove properties of a model or code | Certified control software components |
| **Runtime monitoring** | Check the specification while running, and react to violations | Safety monitors on deployed robots |

## Worked example: a reachable set

A mobile robot moves in the plane with speed at most 1 m/s. Starting at position $\mathbf p_0$, after $t$ seconds it can be anywhere within a disc of radius $t$ m around $\mathbf p_0$; after 2 s, a disc of radius 2 m. If a person may be 1.5 m away, the robot cannot rule out contact within 2 s unless it restricts its speed. Reachability analysis does this computation for real dynamics and disturbances, often by solving an [[Hamilton-Jacobi-Bellman_Equation|HJ]] equation, and the result defines the zone a safety controller must keep clear.

## Formal methods and learning (2026)

- **Verify the shield, not the brain.** Neural policies are too large to verify completely. The safety layer around them can be verified: control barrier function filters, reachability-based envelopes, verified switching to a fallback controller (see [[Collision_Avoidance]]).
- **Neural network verification** for small networks: bounding outputs over input sets.
- **Language to specifications**: language models translate natural-language instructions into LTL or STL, which planners then satisfy with guarantees, and which people can inspect.
- **Regulation**: safety standards for autonomous and AI-driven machines increasingly ask for evidence of this kind (see [[Ethical_and_Social_Implications]]).

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Formal_Methods_in_Control]])
```
