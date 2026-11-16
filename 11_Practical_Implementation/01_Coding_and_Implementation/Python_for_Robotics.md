---
title: Python for Robotics
description: "Why Python is the main language of robot learning, prototyping and tooling, and how to use it well: the 2026 ecosystem (NumPy, SciPy, PyTorch, JAX, MuJoCo, Isaac Lab, LeRobot, rclpy, Rerun), a measured vectorisation example (about 40× faster), performance and real-time limits, packaging with uv, and when to reach for C++ or Rust."
aliases:
  - Python
tags:
  - python
  - robotics-software
  - machine-learning
type: tool
level: beginner
subfield:
  - software
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Robots]]"
related:
  - "[[Python_ROS_Nodes]]"
  - "[[C++]]"
  - "[[Rust]]"
  - "[[MuJoCo]]"
  - "[[Isaac_Lab]]"
key_takeaways:
  - "Python is the default language for robot learning, simulation scripting, data tools and prototyping, because of its scientific and machine-learning libraries."
  - "Write array code, not loops: transforming a million 3D points took about 16 ms with NumPy versus roughly 0.6 s with a per-point loop on one 2026 laptop."
  - "Python is not real-time; keep it out of hard control loops, and use C++ or Rust (or compiled extensions) where timing matters."
sources:
  - "Harris, C. R. et al. (2020). Array programming with NumPy. https://www.nature.com/articles/s41586-020-2649-2"
  - "Paszke, A. et al. (2019). PyTorch: An Imperative Style, High-Performance Deep Learning Library. https://arxiv.org/abs/1912.01703"
  - "Corke, P. (2023). Robotics, Vision and Control: Fundamental Algorithms in Python, 3rd edition. https://link.springer.com/book/10.1007/978-3-031-06469-2"
  - "Open Robotics (2026). rclpy: ROS 2 client library for Python. https://github.com/ros2/rclpy"
---

# Python for Robotics

**Python** is the language most robotics people now write first. It dominates robot learning, simulation scripting, data processing, prototyping and tooling, and it is one of the two official ROS 2 languages. It is not the language of hard real-time control, but it is how most of the "intelligence" of a 2026 robot is developed.

> **Etymology.** Guido van Rossum named Python after the comedy group Monty Python, not the snake; version 1.0 appeared in 1994.

## The 2026 robotics ecosystem

| Need | Libraries |
|---|---|
| Arrays and maths | NumPy, SciPy |
| Robot maths and kinematics | spatialmath, Robotics Toolbox for Python, Pinocchio bindings |
| Deep learning | PyTorch, JAX |
| Simulation | MuJoCo (and MJX, MuJoCo Warp), Isaac Lab, Genesis, PyBullet |
| Robot learning | LeRobot, RL libraries (RSL-RL, Stable-Baselines3) |
| Perception | OpenCV, Open3D, transformers and vision-model libraries |
| ROS 2 | rclpy (see [[Python_ROS_Nodes]]) |
| Visualisation and logging | Rerun, Matplotlib, Foxglove SDK |
| Motion planning | OMPL Python bindings (pip-installable since 2.0), MoveItPy |

## Worked example: vectorise or wait

Transforming a million 3D points by a rotation and translation:

```python
import numpy as np
pts = np.random.default_rng(0).normal(size=(1_000_000, 3))
R, t = np.eye(3), np.array([1.0, 2.0, 3.0])
slow = [R @ p + t for p in pts]   # one small operation per point
fast = pts @ R.T + t              # one array operation
```

Measured on one 2026 laptop: the loop took about 0.6 s (extrapolated from 100,000 points), the array version about 16 ms, roughly **40× faster**. A pure-Python loop without NumPy is slower still. In a 30 Hz perception pipeline, that is the difference between keeping up (33 ms per frame) and falling far behind.

## Limits and remedies

- **Speed**: interpreted code is slow per operation; use array libraries, compiled extensions (pybind11, nanobind, Cython), JIT (Numba, JAX) or move hot code to [[C++]] or [[Rust]].
- **Real time**: garbage collection and the interpreter make timing unpredictable; keep Python out of kilohertz control loops.
- **Concurrency**: the global interpreter lock limits CPU-bound threads; free-threaded Python builds (an option from Python 3.13) and multiprocessing help.
- **Packaging**: use virtual environments; `uv` has become a common fast installer (Isaac Lab 3.0 installs with it).

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Python_for_Robotics]])
```
