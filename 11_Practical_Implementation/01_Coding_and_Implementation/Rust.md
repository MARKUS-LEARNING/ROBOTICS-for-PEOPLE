---
title: Rust (Robotics)
description: "Rust is a systems language that prevents whole classes of memory and concurrency bugs at compile time while matching C++ for speed. Covers ownership and borrowing with a small example, why that matters for robots, Rust in ROS 2 (rclrs, and rosidl_rust among the default interface generators since Kilted), embedded Rust on microcontrollers, and an honest view of the trade-offs in 2026."
aliases:
  - Rust
  - Rust Programming Language
tags:
  - rust
  - robotics-software
  - real-time
type: tool
level: intermediate
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
  - "[[Python_for_Robotics]]"
  - "[[Microcontroller]]"
  - "[[Custom_Packages_and_Nodes]]"
key_takeaways:
  - "Rust's ownership and borrowing rules let the compiler reject use-after-free, data races and many null-pointer errors, without a garbage collector."
  - "ROS 2 Kilted (2025) added rosidl_rust to the default interface generators, and the ros2-rust project's rclrs provides a Rust client library."
  - "Rust suits drivers, embedded firmware and safety-critical components; its robotics ecosystem is younger than C++'s or Python's, so mixed-language systems are normal."
sources:
  - "Klabnik, S. and Nichols, C. (2026). The Rust Programming Language. https://doc.rust-lang.org/book/"
  - "ros2-rust contributors (2026). ros2_rust: Rust bindings for ROS 2. https://github.com/ros2-rust/ros2_rust"
  - "Open Robotics (2025). ROS 2 Kilted Kaiju release notes (rosidl_rust added). https://github.com/ros2/ros2_documentation/blob/rolling/source/Releases/Release-Kilted-Kaiju.rst"
---

# Rust (Robotics)

**Rust** is a systems programming language with performance comparable to C and C++, and with a compiler that rules out many of the bugs that crash robot software: dangling pointers, double frees, buffer overruns and data races between threads.

> **Etymology.** Graydon Hoare began Rust as a personal project in 2006, reportedly naming it after rust fungi, which are remarkably robust. Mozilla sponsored it from 2009; version 1.0 shipped in 2015.

## Ownership in one example

```rust
fn main() {
    let scan = vec![1.2_f32, 0.8, 2.5];       // `scan` owns the vector
    let closest = min_range(&scan);           // lend it (borrow) for reading
    println!("closest {closest}");
    let moved = scan;                         // ownership moves to `moved`
    // println!("{:?}", scan);                // compile error: `scan` was moved
    drop(moved);                              // freed exactly once
}
fn min_range(r: &[f32]) -> f32 { r.iter().cloned().fold(f32::INFINITY, f32::min) }
```

Every value has one owner. Others may *borrow* it, either many readers or one writer at a time, never both. These rules are checked at compile time, so a thread cannot modify a buffer that another is reading. In C++ the same mistake compiles and fails at run time, often only occasionally.

## Why it matters for robots

- **Crashes are expensive**: a segfault in a driver can stop a moving robot.
- **Concurrency is everywhere**: sensor callbacks, control loops and planners run in parallel threads.
- **Long-lived systems**: code that is safe by construction is easier to maintain and certify.
- **Embedded targets**: Rust compiles to bare-metal microcontrollers (no operating system), with async frameworks such as Embassy.

## Rust in ROS 2

- **rclrs** (from the ros2-rust project) is a Rust client library for writing ROS 2 nodes.
- Since **Kilted (2025)**, `rosidl_rust` is among the default interface generators, so message, service and action types are generated for Rust alongside C++ and Python.
- Other Rust robotics projects include Zenoh (whose core, used by `rmw_zenoh`, is written in Rust), Rerun (visualisation), and robotics-oriented maths and control crates.

## Trade-offs (2026)

| Pros | Cons |
|---|---|
| Memory and thread safety at compile time | Steeper learning curve (the borrow checker) |
| C++-level performance, no garbage collector | Smaller robotics ecosystem; fewer drivers and planners |
| Excellent tooling (cargo, testing, documentation) | ROS 2 support less mature than C++/Python |
| Good fit for embedded and safety-critical code | Interop with large C++ code bases needs bindings |

Most teams adopt Rust incrementally: new drivers, firmware, or a safety monitor, alongside existing [[C++]] and [[Python_for_Robotics|Python]].

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Rust]])
```
