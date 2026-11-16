---
title: GPU-Parallel Simulation
description: "Running thousands of robot simulations at once on a GPU so reinforcement learning gets years of experience in hours: how batched physics works, the lineage from Isaac Gym and Brax to Isaac Lab 3.0, MJX, MuJoCo Warp, Newton, ManiSkill3 and Genesis, and the limits of the approach."
aliases:
  - Massively Parallel Simulation
  - GPU Simulation
tags:
  - simulation
  - parallel-computing
  - cuda
  - reinforcement-learning
  - robot-learning
type: method
level: advanced
subfield:
  - software
  - robot-learning
status: reviewed
author: Jordan_Smith
date: 2026-11-15
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Physics_Engines]]"
  - "[[Reinforcement_Learning]]"
related:
  - "[[Isaac_Lab]]"
  - "[[MuJoCo]]"
  - "[[Reinforcement_Learning_for_Robots]]"
  - "[[CUDA]]"
key_takeaways:
  - "One GPU steps thousands of copies of a robot in lockstep, and the learning code reads their states without copying data to the CPU."
  - "This turned robot RL from days of training into minutes to hours, and made locomotion learning routine."
  - "Batched simulation favours simple, uniform scenes; rich rendering, very different worlds per copy and some contact effects remain costly."
sources:
  - "Makoviychuk, V. et al. (2021). Isaac Gym: High Performance GPU-Based Physics Simulation For Robot Learning. https://arxiv.org/abs/2108.10470"
  - "Freeman, C. D. et al. (2021). Brax -- A Differentiable Physics Engine for Large Scale Rigid Body Simulation. https://arxiv.org/abs/2106.13281"
  - "Rudin, N. et al. (2021). Learning to Walk in Minutes Using Massively Parallel Deep Reinforcement Learning. https://arxiv.org/abs/2109.11978"
  - "Zakka, K. et al. (2025). MuJoCo Playground. https://arxiv.org/abs/2502.08844"
  - "Tao, S. et al. (2024). ManiSkill3: GPU Parallelized Robotics Simulation and Rendering for Generalizable Embodied AI. https://arxiv.org/abs/2410.00425"
  - "NVIDIA (2026). Isaac Lab 3.0 release notes. https://github.com/isaac-sim/IsaacLab/releases/tag/v3.0.0"
---

# GPU-Parallel Simulation

**GPU-parallel simulation** runs many copies of a robot simulation at the same time on a graphics processor. Instead of one simulated robot learning for a week, four thousand learn side by side for a few minutes, and a reinforcement learning algorithm learns from all of them at once. It is the main reason robots can now learn to walk, run and recover from falls in an afternoon of computing.

> **Etymology.** *GPU* stands for *graphics processing unit*, a term NVIDIA popularised with the GeForce 256 in 1999. GPUs were built to shade millions of pixels in parallel; since the late 2000s (CUDA, 2007) the same hardware has been used for general computing, from neural networks to physics. *Parallel* is from Greek *parallēlos*, "beside one another".

## Why it works

A GPU has thousands of simple cores that are fast when they all perform the same operation on different data. Thousands of identical robots stepping the same equations with different states are exactly that pattern. Two design rules make it pay off:

1. **Batch everything.** Store every world's state in large arrays and update them all in one GPU call per step.
2. **Never leave the GPU.** The policy network, the simulator and the RL update all read and write the same GPU memory, so no time is lost copying states to the CPU and back.

## A short lineage

| Year | System | What it showed |
|---|---|---|
| 2021 | **Isaac Gym** (NVIDIA) | End-to-end GPU physics plus learning; training speedups of two to three orders of magnitude over CPU pipelines |
| 2021 | **Brax** (Google) | A differentiable rigid-body engine in JAX, on GPUs and TPUs |
| 2021 | Rudin et al. | A quadruped learning to walk in minutes with 4,096 parallel robots |
| 2023–24 | **MJX** (MuJoCo in JAX), **ManiSkill3**, **Orbit → Isaac Lab** | Parallel simulation for mainstream research engines; manipulation with GPU rendering |
| 2025 | **MuJoCo Playground**, **Isaac Lab** paper, **Genesis** | Ready-made environment suites; open GPU multi-physics |
| 2026 | **Isaac Lab 3.0** (October), **MuJoCo 4 / Playground 1.0** and **MuJoCo Warp 4** (November), **Newton 1.x** | One task API over several backends; open Warp-based engines; deformables and fluids on the GPU |

## Worked example: budgeting a training run

A locomotion policy needs about 100 million control steps. With 4,096 parallel robots, each policy update collects $4096 \times 24 \approx 98{,}000$ steps (24 steps per robot per update). Suppose the whole pipeline (physics at 200 Hz with 4 substeps per 50 Hz control step, the policy forward pass and the PPO update) delivers 100,000 control steps per second on one GPU. Then

$$\frac{10^8 \text{ steps}}{10^5 \text{ steps/s}} = 1000 \text{ s} \approx 17 \text{ minutes.}$$

The same 100 million steps on one robot in real time at 50 Hz would take $10^8 / 50 = 2 \times 10^6$ s: about 23 days. These are illustrative numbers, but this is the order of magnitude that made "learning to walk in minutes" literal.

## Limits

- **Uniformity.** Batched simulation is fastest when every world has the same robot and similar scenes. Very different objects or scene layouts per world cost speed.
- **Rendering.** Physics batches well; rendering many camera views for vision-based policies is much more expensive. Tiled and ray-traced batch renderers help.
- **Contact fidelity.** Simplifications that make physics fast can make contact less accurate, which shows up as a [[Sim-to-Real_Transfer|sim-to-real gap]].
- **Memory.** Thousands of worlds with high-resolution sensors can exhaust GPU memory.

## Tools (as of November 2026)

| Tool | Engine | Notes |
|---|---|---|
| [[Isaac_Lab]] 3.0 | PhysX (OVPhysX) or Newton | One task API; Isaac Sim optional; RSL-RL and other RL libraries |
| [[MuJoCo]] Playground 1.0 | MJX / MuJoCo Warp | Locomotion, manipulation and dexterous environments |
| Newton | NVIDIA Warp | Apache-2.0, Linux Foundation; MuJoCo Warp, VBD, MPM and other solvers |
| ManiSkill3 | SAPIEN / PhysX | GPU-parallel manipulation with rendering |
| Genesis | Own multi-solver engine | GPU multi-physics |
| Brax | JAX | Differentiable, lightweight |

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[GPU-Parallel_Simulation]])
```
