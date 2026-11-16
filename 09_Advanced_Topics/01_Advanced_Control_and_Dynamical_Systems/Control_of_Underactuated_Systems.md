---
title: Control of Underactuated Systems
description: "Underactuated robots have fewer actuators than degrees of freedom: legged robots in flight, drones, balancing robots, the acrobot and cart-pole. Covers why they are hard (you cannot command every direction), energy-based swing-up with a computed target energy, partial feedback linearisation, LQR stabilisation, trajectory optimisation, and why reinforcement learning has been so successful on them."
aliases:
  - Underactuated Systems
  - Underactuated Robotics
  - Underactuation
tags:
  - control
  - locomotion
  - reinforcement-learning
type: concept
level: research
subfield:
  - control
  - legged-humanoid
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Nonlinear_Control_Theory]]"
  - "[[Optimal_Control]]"
related:
  - "[[Feedback_Linearization]]"
  - "[[Hybrid_Systems_and_Control]]"
  - "[[Legged_Robots]]"
  - "[[Drones]]"
  - "[[Reinforcement_Learning_for_Robots]]"
key_takeaways:
  - "A system is underactuated when it cannot command an arbitrary acceleration in every degree of freedom: the cart-pole has 2 degrees of freedom and 1 motor; a walking robot cannot push on the ground it is not touching."
  - "Control must use the dynamics rather than cancel them: pump energy to the swing-up target (mgl = 0.98 J for a 0.2 kg, 0.5 m pole), then catch the upright state with LQR."
  - "Agile legged, aerial and dexterous robots are underactuated; trajectory optimisation, MPC and especially RL in simulation are the methods of choice in 2026."
sources:
  - "Tedrake, R. (2026). Underactuated Robotics, chapter: The Acrobot, Cart-Pole, and Quadrotors. https://underactuated.csail.mit.edu/acrobot.html"
  - "Tedrake, R. (2026). Underactuated Robotics, chapter: Simple Models of Walking and Running. https://underactuated.csail.mit.edu/simple_legs.html"
  - "Hwangbo, J. et al. (2019). Learning agile and dynamic motor skills for legged robots. https://arxiv.org/abs/1901.08652"
  - "Kaufmann, E. et al. (2023). Champion-level drone racing using deep reinforcement learning. https://www.nature.com/articles/s41586-023-06419-4"
---

# Control of Underactuated Systems

A robot is **underactuated** when its actuators cannot directly command acceleration in every degree of freedom. A fully actuated arm can push each joint any way it likes; a cart-pole can only push the cart, and must move the pole indirectly through the coupling. Most interesting robots are underactuated:
- legged robots: the body is not attached to the ground, and a foot in the air cannot push;
- drones: four thrusts for six degrees of freedom;
- balancing robots and dexterous hands manipulating free objects.

> **Etymology.** *Under* + *actuated*: fewer actuators than needed for full control. Russ Tedrake's MIT course and online book *Underactuated Robotics* made the term central to modern robot control.

## Why it is hard

For dynamics $M(\mathbf q)\ddot{\mathbf q} + C\dot{\mathbf q} + \mathbf g = B\mathbf u$, the system is underactuated when $B$ has fewer independent columns than there are degrees of freedom. There are directions in which the controller cannot directly push, so it cannot simply cancel the dynamics ([[Feedback_Linearization]] fails). It must instead **exploit** them: inject energy, use momentum, plan ahead.

## Classic examples

| System | DoF | Actuators | Challenge |
|---|---|---|---|
| Cart-pole | 2 | 1 (cart) | Swing the pole up and balance it |
| Acrobot | 2 | 1 (elbow only) | Swing up with no shoulder motor |
| Quadrotor | 6 | 4 thrusts | Must tilt to move sideways |
| Biped in single support | Many | No torque at the foot contact point (or limited) | Balance via stepping and momentum |

## Energy-based swing-up, computed

For a pole of mass $m$ with its centre of mass at distance $l$ from the pivot, define its energy as $E = \tfrac12 ml^2\dot\theta^2 + mgl\cos\theta$ (θ measured from upright, potential measured from the pivot's height). At rest upright, $E^* = mgl$; for $m = 0.2$ kg and $l = 0.5$ m, that is $0.2 \times 9.81 \times 0.5 = 0.98$ J. Hanging at rest, $E = -mgl$, so the controller must add $2mgl = 1.96$ J. Swing-up controllers move the cart so as to pump energy in when $E < E^*$ and remove it when $E > E^*$, for example accelerating the cart in the direction of $(E - E^*)\,\dot\theta\cos\theta$. Once the pole nears upright with nearly the right energy, a linear controller (LQR) designed around the upright equilibrium catches and balances it. Switching between the two controllers is a small hybrid system (see [[Hybrid_Systems_and_Control]]).

## Methods

- **Partial feedback linearisation**: linearise the actuated (or a chosen) coordinate, accepting nonlinear behaviour in the rest.
- **Energy shaping and passivity-based control**.
- **LQR around equilibria and trajectories** (time-varying LQR along a planned motion).
- **Trajectory optimisation and MPC** over contact sequences (see [[Model_Predictive_Control_(MPC)]]).
- **Reinforcement learning**: the policy discovers how to exploit dynamics without hand-derived structure.

## Why learning took over agile underactuated control

The most striking robot results of 2019–2026 are underactuated: quadrupeds running and recovering on rough terrain, humanoids walking and doing flips, drones racing at champion level, hands spinning objects. Each was learned with reinforcement learning in simulation (see [[Reinforcement_Learning_for_Robots]]). Learning handles the combination of nonlinearity, contact switching and underactuation that is hard to treat analytically. Classical underactuated control still provides the models, the simulators, the reward structure and the analysis of why the learned behaviours work.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Control_of_Underactuated_Systems]])
```
