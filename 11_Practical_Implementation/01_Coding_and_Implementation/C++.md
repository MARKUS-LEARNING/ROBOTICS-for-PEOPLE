---
title: C++ (Robotics)
description: "C++ is the language of robot middleware, drivers, real-time control and high-performance planning. Covers why robotics still uses it, the modern C++ that matters (C++17/20, RAII, smart pointers), the ROS 2 Lyrical move to C++20, real-time rules with a computed 1 kHz budget, key libraries (Eigen, Pinocchio, FCL, OMPL, PCL), and C++ next to Python and Rust in 2026."
aliases:
  - C++
  - Cpp
  - Modern C++
tags:
  - cpp
  - robotics-software
  - real-time
type: tool
level: intermediate
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
  - "[[Python_for_Robotics]]"
  - "[[Rust]]"
  - "[[C++_Motion_Planning]]"
  - "[[ros2_control]]"
key_takeaways:
  - "C++ gives predictable, fast code with direct hardware access, which is why ROS 2's core, ros2_control, MoveIt, Nav2 and most drivers are written in it."
  - "At 1 kHz a control loop has 1 ms per cycle; real-time C++ avoids memory allocation, locks and blocking I/O inside that loop."
  - "ROS 2 Lyrical (2026) targets C++20; Python leads for learning and tools, Rust is growing for safety-critical new code."
sources:
  - "cppreference.com (2026). C++20. https://en.cppreference.com/w/cpp/20"
  - "Open Robotics (2026). ROS 2 Lyrical Luth supported platforms (C++20). https://github.com/ros2/ros2_documentation/blob/rolling/source/Releases/lyrical/supported-platforms.rst"
  - "ros-controls (2026). ros2_control. https://github.com/ros-controls/ros2_control"
---

# C++ (Robotics)

**C++** is the workhorse language for the parts of a robot where speed and timing matter: middleware, device drivers, real-time control loops, motion planners, perception pipelines on embedded computers, and simulators. ROS 2's core libraries, ros2_control, MoveIt, Nav2, Gazebo, MuJoCo and most hardware drivers are written in it.

> **Etymology.** Bjarne Stroustrup created "C with Classes" at Bell Labs in 1979, renamed C++ (C's increment operator: "one more than C") in 1983.

## Why robotics uses it

- **Performance**: compiled, zero-cost abstractions, direct memory control.
- **Predictable timing**: no garbage collector, so code can be written to meet hard deadlines.
- **Hardware access**: drivers, buses, shared memory, GPU interop.
- **Ecosystem**: Eigen (linear algebra), Pinocchio (rigid-body dynamics), FCL (collision), OMPL (planning), PCL and Open3D (point clouds), OpenCV.

## Modern C++ that matters

- **RAII and smart pointers** (`std::unique_ptr`, `std::shared_ptr`): resources freed automatically, no manual `delete`.
- **C++17/20 features**: `std::optional`, `std::variant`, structured bindings, `constexpr`, concepts, ranges, `std::span`. ROS 2 Lyrical moved its baseline to **C++20**.
- **Modern CMake**: `target_link_libraries` with imported targets (in ROS 2 since Kilted, instead of `ament_target_dependencies`).

## Worked example: the real-time budget

A [[ros2_control]] loop at 1 kHz has 1 ms per cycle for reading sensors, computing all controllers and writing commands. A single `new` or `malloc` inside that loop can occasionally take hundreds of microseconds if the allocator must ask the operating system for memory, which is enough to miss the deadline. Real-time C++ rules:
- allocate everything before the loop starts (preallocated buffers, fixed-size containers);
- no blocking I/O, logging to disk or unbounded loops in the loop;
- use lock-free queues to pass data to non-real-time threads;
- run on a real-time kernel (PREEMPT_RT, mainline since Linux 6.12) with appropriate thread priorities.

## C++, Python and Rust in 2026

| Use | Typical language |
|---|---|
| Real-time control, drivers, middleware | C++ (Rust increasingly) |
| Planning and perception libraries | C++ with Python bindings |
| Learning, data, prototyping, tools | Python |
| New safety-critical or embedded code | Rust gaining ground (see [[Rust]]) |

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[C++]])
```
