---
title: MuJoCo
description: "MuJoCo (Multi-Joint dynamics with Contact) is an open-source physics engine from Google DeepMind for robotics and control research. Covers its contact model, the MJCF format, MJX and MuJoCo Warp for GPUs, Menagerie robot models, MuJoCo Playground, and what changed in MuJoCo 4 (November 2026)."
aliases:
  - MuJoCo 4
  - MJX
  - MuJoCo Warp
tags:
  - mujoco
  - simulation
  - dynamics
  - reinforcement-learning
type: tool
level: intermediate
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
related:
  - "[[GPU-Parallel_Simulation]]"
  - "[[Isaac_Lab]]"
  - "[[Sim-to-Real_Transfer]]"
  - "[[URDF]]"
  - "[[Simulators_and_IDEs]]"
key_takeaways:
  - "MuJoCo models contact as soft convex constraints, which makes it fast, stable and well suited to control and learning."
  - "It runs on CPUs, batched CPU worlds, GPUs and TPUs (MJX), NVIDIA GPUs (MuJoCo Warp) and in the browser."
  - "MuJoCo 4 (6 November 2026) added MuJoCo Studio, deformables with plasticity, speculative contacts and a system-identification toolbox."
sources:
  - "Todorov, E., Erez, T. and Tassa, Y. (2012). MuJoCo: A physics engine for model-based control. https://homes.cs.washington.edu/~todorov/papers/TodorovIROS12.pdf"
  - "Google DeepMind (2026). MuJoCo 4: Studio & Filament, Deformables, Speculative Contacts. https://github.com/google-deepmind/mujoco/discussions/3773"
  - "Google DeepMind (2026). MuJoCo 4.0.0 release notes. https://github.com/google-deepmind/mujoco/releases/tag/4.0.0"
  - "Zakka, K. et al. (2025). MuJoCo Playground. https://arxiv.org/abs/2502.08844"
  - "Google DeepMind (2026). MuJoCo documentation: Computation. https://mujoco.readthedocs.io/en/stable/computation/index.html"
---

# MuJoCo

**MuJoCo** (*Multi-Joint dynamics with Contact*) is a physics engine built for robotics, biomechanics and control research. It is known for fast, stable contact simulation and a clean model format, and it is one of the two most widely used simulators in robot learning (the other being NVIDIA's [[Isaac_Lab|Isaac stack]]). Google DeepMind maintains it as open source under the Apache-2.0 licence.

> **Etymology.** The name is an acronym: **Mu**lti-**Jo**int dynamics with **Co**ntact. Emanuel Todorov created it at the University of Washington, published it with Tom Erez and Yuval Tassa in 2012, and sold it commercially through his company Roboti LLC. DeepMind acquired MuJoCo in October 2021, made it free, and released the source code in 2022.

## How it models contact

MuJoCo treats contacts as **soft constraints** and computes contact forces by solving a convex optimisation problem each step (see [[Physics_Engines]]). Contacts can penetrate very slightly, in a controlled way set by parameters such as `solref` (stiffness and damping as a time constant) and `solimp` (impedance). In exchange the dynamics are smooth, stable at practical time steps, and invertible: given a motion, MuJoCo can compute the forces that produced it, which is useful for control and system identification.

## The model format: MJCF

Models are written in **MJCF**, an XML format; MuJoCo also imports [[URDF]], and the optional `mujoco[usd]` package adds an OpenUSD importer and exporter. A minimal pendulum:

```xml
<mujoco>
  <option timestep="0.002"/>
  <worldbody>
    <body name="pole" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 1 0" damping="0.05"/>
      <geom type="capsule" fromto="0 0 0  0 0 -0.5" size="0.02" mass="0.2"/>
    </body>
  </worldbody>
  <actuator>
    <motor joint="hinge" ctrlrange="-1 1"/>
  </actuator>
</mujoco>
```

```python
import mujoco
model = mujoco.MjModel.from_xml_path("pendulum.xml")
data = mujoco.MjData(model)
data.qpos[0] = 0.5                      # start 0.5 rad from vertical
for _ in range(500):                    # 1 second at 2 ms per step
    mujoco.mj_step(model, data)
print(data.qpos[0], data.qvel[0])
```

## Where it runs

| Backend | Hardware | Use |
|---|---|---|
| C library + Python bindings | CPU | Development, control, single-robot work |
| `mujoco.batch` | Many worlds on CPU cores | Parallel evaluation without a GPU |
| **MJX** | GPUs and TPUs via JAX | GPU-parallel RL; differentiable |
| **MuJoCo Warp** | NVIDIA GPUs via Warp | Fast GPU-parallel RL; developed with NVIDIA; one of Newton's solvers and an Isaac Lab 3.0 backend |
| WebAssembly | Browser | Teaching and demos (`live.mujoco.org`) |

## What changed in MuJoCo 4 (6 November 2026)

From DeepMind's announcement and release notes:

- **MuJoCo Studio**: a modular viewer and tool SDK (Python and C++) with physically based rendering through Filament, hot reloading and browser streaming.
- **Soft bodies**: implicit integration for solids, shells and rods; penetration-free cloth; elasto-plasticity (permanent deformation) for joints, tendons and deformables.
- **Speculative contacts**: fast, thin objects no longer tunnel through each other within a step.
- **Collision**: native convex collision and multi-point contacts by default.
- **Actuators and sensors**: built-in PID and datasheet-based DC-motor actuators, delays, tactile and contact sensors.
- **Sim-to-real**: a system-identification toolbox (`mujoco.sysid`) and **MuJoCo Playground 1.0**.
- **Packaging**: the Python package is split into a minimal `mujoco-core` plus optional components; Python 3.11–3.15 are supported.
- **Models**: about 70 robot models in **MuJoCo Menagerie**, installable with `pip install mujoco-menagerie`.

## Ecosystem

- **MuJoCo Menagerie**: curated, tuned models of commercial robots (arms, humanoids, quadrupeds, hands).
- **MuJoCo Playground**: GPU-parallel environments for locomotion, manipulation and dexterity, with training recipes; version 1.0 released alongside MuJoCo 4.
- **dm_control**: DeepMind's control-suite tasks.
- **Gymnasium**: standard RL environments built on MuJoCo.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[MuJoCo]])
```
