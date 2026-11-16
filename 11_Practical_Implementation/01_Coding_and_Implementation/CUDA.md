---
title: CUDA
description: "CUDA is NVIDIA's platform for programming GPUs, the hardware that trains and runs robot learning models and powers massively parallel simulation. Covers the programming model (kernels, threads, memory), why robotics needs it (simulation, perception, learning, GPU planning), Amdahl's law with a computed limit on speed-up, embedded GPUs (Jetson Thor), and the higher-level tools most people use instead of raw CUDA in 2026."
aliases:
  - GPU Programming
  - CUDA Programming
tags:
  - cuda
  - robotics-software
type: tool
level: advanced
subfield:
  - software
  - electronics-embedded
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[C++]]"
related:
  - "[[GPU-Parallel_Simulation]]"
  - "[[Isaac_Lab]]"
  - "[[Deep_Learning]]"
  - "[[CNN_in_Robotics]]"
key_takeaways:
  - "CUDA runs thousands of lightweight threads in parallel on a GPU; it pays off when the same operation applies to many data items: pixels, points, simulated robots, network weights."
  - "Speed-up is limited by the serial part: if 95% of a workload parallelises perfectly, the whole runs at most 20× faster, however many GPU cores you add (Amdahl's law)."
  - "Most robotics developers use CUDA through libraries (PyTorch, Warp, TensorRT, cuRobo, Isaac Lab) rather than writing kernels; Isaac Lab 3.0 targets CUDA 13.0, and Jetson Thor brings the same stack onto robots."
sources:
  - "NVIDIA (2026). CUDA Toolkit documentation. https://docs.nvidia.com/cuda/"
  - "NVIDIA (2026). Jetson Thor. https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-thor/"
  - "NVIDIA (2026). Isaac Lab 3.0 release notes. https://github.com/isaac-sim/IsaacLab/releases/tag/v3.0.0"
  - "Makoviychuk, V. et al. (2021). Isaac Gym: High Performance GPU-Based Physics Simulation For Robot Learning. https://arxiv.org/abs/2108.10470"
---

# CUDA

**CUDA** is NVIDIA's platform for general-purpose computing on graphics processing units (GPUs). A GPU has thousands of simple cores; CUDA lets programs launch thousands of threads that each process one pixel, one point, one neuron or one simulated robot at the same time. Modern robot learning and simulation depend on it.

> **Etymology.** Originally *Compute Unified Device Architecture*, released by NVIDIA in 2007. The acronym is now used as a name.

## The programming model

- A **kernel** is a function that runs on the GPU, launched over a grid of **blocks**, each containing many **threads**.
- Threads in a block share fast **shared memory**; all threads can access slower **global memory**.
- Data moves between CPU and GPU memory, which costs time, so good GPU code keeps data on the GPU.

```cpp
__global__ void scale(float* x, float a, int n) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) x[i] *= a;          // each thread handles one element
}
// launch: scale<<<(n + 255) / 256, 256>>>(d_x, 2.0f, n);
```

## Why robotics needs GPUs

| Workload | GPU use |
|---|---|
| Learning | Training and running neural networks (PyTorch, JAX) |
| Simulation | Thousands of robots in parallel for RL ([[GPU-Parallel_Simulation]], [[Isaac_Lab]], MuJoCo Warp) |
| Perception | Image processing, detection and depth networks, point-cloud filtering |
| Planning | Batched trajectory optimisation and collision checking (cuRobo) |
| Rendering | Synthetic data generation, photoreal simulation |

## Worked example: the limit on speed-up

**Amdahl's law**: if a fraction $p$ of a program can be parallelised and the rest is serial, the maximum speed-up with unlimited parallel processors is $1/(1 - p)$.

With $p = 0.95$: at most $1/0.05 = 20\times$, even with millions of GPU threads. With 1,000 parallel workers the speed-up is $1/(0.05 + 0.95/1000) \approx 19.6\times$. To gain more, shrink the serial part: move everything onto the GPU, including the simulation step, reward computation and policy, as Isaac Gym did in 2021 by keeping all data on the GPU.

## On the robot

**Jetson** modules put NVIDIA GPUs into robots; **Jetson Thor** (2025) is the module used in many 2025–26 humanoids (for example, 1X lists it for NEO). Deployment typically compiles networks with **TensorRT**, using lower precision (FP16, INT8, FP4) for speed.

## Higher-level tools (2026)

Few robotics developers write raw CUDA kernels. They use PyTorch and JAX; **NVIDIA Warp** (Python that compiles to GPU kernels, used by Newton and MuJoCo Warp); CuPy; TensorRT; and domain libraries (cuRobo, Isaac ROS). Isaac Lab 3.0 targets CUDA 13.0 with PyTorch 2.11.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[CUDA]])
```
