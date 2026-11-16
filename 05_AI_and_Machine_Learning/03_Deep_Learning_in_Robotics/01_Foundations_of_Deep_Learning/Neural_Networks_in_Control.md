---
title: Neural Networks in Control
description: "Using neural networks inside feedback control: as learned dynamics or actuator models, as approximate controllers, inside model-predictive control, and as end-to-end learned policies, with the stability and safety questions each raises."
tags:
  - neural-networks
  - control-theory
  - neurocontrol
  - system-identification
  - reinforcement-learning
  - adaptive-control
type: method
level: advanced
subfield:
  - robot-learning
  - control
status: reviewed
author: Jordan_Smith
date: 2025-04-28
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Neural_Networks]]"
  - "[[Feedback_Control]]"
related:
  - "[[Model_Predictive_Control_(MPC)]]"
  - "[[Reinforcement_Learning_for_Robots]]"
  - "[[Adaptive_Control]]"
  - "[[Computed_Torque_Control]]"
  - "[[World_Models]]"
key_takeaways:
  - "Networks enter control loops in four roles: model of the plant, model of the actuator, controller, or the cost/value inside an optimiser."
  - "The most reliable pattern keeps a classical controller for stability and uses the network for what physics models miss."
  - "Guarantees are the hard part: learned components are tested statistically, bounded by safety filters, and certified only in narrow cases."
sources:
  - "Hwangbo, J. et al. (2019). Learning Agile and Dynamic Motor Skills for Legged Robots. https://arxiv.org/abs/1901.08652"
  - "Nagabandi, A. et al. (2017). Neural Network Dynamics for Model-Based Deep Reinforcement Learning with Model-Free Fine-Tuning. https://arxiv.org/abs/1708.02596"
  - "Salzmann, T. et al. (2022). Real-time Neural-MPC: Deep Learning Model Predictive Control for Quadrotors and Agile Robotic Platforms. https://arxiv.org/abs/2203.07747"
  - "Chang, Y.-C., Roohi, N. and Gao, S. (2019). Neural Lyapunov Control. https://arxiv.org/abs/2005.00611"
---

# Neural Networks in Control

A feedback controller compares what the robot is doing with what it should be doing and corrects the difference. Classical control does this with equations derived from physics. **Neural networks in control** replace or augment some of those equations with functions learned from data, for the parts of the robot that physics models get wrong or that are too complex to write down.

> **Etymology.** *Neural* is from Greek *neuron*, "sinew, nerve". *Control* comes from medieval Latin *contrarotulare*, "to check against a duplicate roll": a clerk kept a counter-roll of accounts to check the original. A controller still does exactly that: it checks the actual state against the desired one. *Neurocontrol* was a named research area by the late 1980s.

## Four roles a network can play

| Role | What is learned | Example | Why |
|---|---|---|---|
| **Plant model** | Robot or object dynamics $\dot x = f_\theta(x, u)$ | Learned dynamics for model-based RL (Nagabandi et al. 2017) | Physics models miss friction, aerodynamics, cables |
| **Actuator model** | Motor response: command → actual torque | ANYmal's "actuator net" (Hwangbo et al. 2019) | Series-elastic and geared actuators are hard to model; key to sim-to-real |
| **Controller** | The control law itself, $u = \pi_\theta(x)$ | Locomotion policies trained by RL | Handles complex contact-rich behaviour |
| **Inside an optimiser** | Dynamics, cost or value used by MPC | Real-time Neural-MPC for quadrotors (Salzmann et al. 2022) | Keeps MPC's constraints and replanning, adds learned accuracy |

## The residual pattern

The most dependable recipe keeps the classical model and learns only its error:

$$\dot x = f_{\text{physics}}(x, u) + f_\theta(x, u)$$

or keeps a classical controller and adds a learned correction, $u = u_{\text{PD}} + u_\theta$. The physics part provides structure and good behaviour far from the training data; the network soaks up what the physics misses. If the network misbehaves, its contribution can be clipped.

## Worked example: learning a drag term

A quadrotor's thrust model ignores aerodynamic drag, so at 10 m/s its position controller lags. Log flights; at each time step compute the residual acceleration $r = a_{\text{measured}} - a_{\text{model}}$. Train a small network $f_\theta(v)$ to predict $r$ from velocity, then feed $a_{\text{model}} + f_\theta(v)$ into the controller. Because $f_\theta$ only has to learn a smooth function of velocity, a few minutes of flight data are enough; this is the idea behind learned-dynamics MPC for agile flight.

## Stability and safety

Classical controllers come with proofs: Lyapunov functions, gain margins. Neural components usually do not. Approaches in use:

- **Safety filters.** A learned command is passed through a filter (for example a control-barrier-function quadratic program) that minimally changes it to keep the system in a safe set.
- **Learned certificates.** Train a network to be a Lyapunov function alongside the controller and check it with a verifier (Chang, Roohi and Gao 2019, *Neural Lyapunov Control*); practical for small systems.
- **Statistical validation.** Test across large randomised simulation suites and long real-world trials; this is what most deployed learned controllers rely on.
- **Bounded authority.** Limit what the learned part can command, and keep a classical fallback.

## State of the art (as of November 2026)

- Learned policies trained in simulation are the standard controllers for legged robots and increasingly for humanoid whole-body motion; see [[Reinforcement_Learning_for_Robots]].
- Large learned policies such as [[Vision-Language-Action_Models]] issue targets at 10–50 Hz to classical low-level controllers, which handle stiffness and safety at 500–1000 Hz.
- Learned dynamics inside MPC remain the preferred approach where constraints must be respected explicitly, as in agile flight.

## Reading list

- Hwangbo et al. 2019: actuator networks and sim-to-real for ANYmal.
- Salzmann et al. 2022, *Real-time Neural-MPC*.
- Chang, Roohi and Gao 2019, *Neural Lyapunov Control*.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Neural_Networks_in_Control]])
```
