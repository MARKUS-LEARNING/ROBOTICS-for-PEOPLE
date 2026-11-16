---
title: Hardware Interface Setup
description: "Connecting motors and sensors to a robot's computer and to ROS 2: the buses (UART, I²C, SPI, USB, CAN and CAN FD, EtherCAT, Ethernet), a computed CAN bandwidth budget for a 12-motor robot, writing a ros2_control hardware interface, drivers for sensors, time synchronisation, and a bring-up checklist."
aliases:
  - Hardware Interfaces
  - Robot Bring-Up
  - Hardware Integration
tags:
  - hardware-interface
  - electronics
  - ros2
  - real-time
type: tutorial
level: intermediate
subfield:
  - electronics-embedded
  - software
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[ros2_control]]"
  - "[[Microcontroller]]"
related:
  - "[[Serial_Communication_Tutorials]]"
  - "[[Arduino]]"
  - "[[TF_and_Topic_Architecture]]"
  - "[[URDF]]"
key_takeaways:
  - "Choose the bus by bandwidth, latency and robustness: I²C and SPI on a board, UART/USB for simple links, CAN or CAN FD for joints in vehicles and small robots, EtherCAT for fast, synchronised multi-axis robots."
  - "Classic CAN at 1 Mbit/s carries about 7,700 eight-byte frames per second; a 12-motor robot sending a command and a state frame per motor per cycle tops out near 320 Hz, which is why fast robots use CAN FD or EtherCAT."
  - "In ROS 2 the hardware interface is a ros2_control plugin with read() and write(); getting units, signs, zero offsets, limits and timestamps right matters more than any controller tuning."
sources:
  - "ros-controls (2026). ros2_control. https://github.com/ros-controls/ros2_control"
  - "EtherCAT Technology Group (2026). EtherCAT technology. https://www.ethercat.org/en/technology.html"
  - "CAN in Automation (2026). CAN knowledge. https://www.can-cia.org/can-knowledge/"
  - "micro-ROS (2026). micro-ROS: ROS 2 for microcontrollers. https://micro.ros.org/"
---

# Hardware Interface Setup

**Hardware interface setup** ("bring-up") is the work of connecting a robot's motors, sensors and electronics to its computer, and exposing them to software in a consistent way. It is where most robot projects lose time, and where careful habits pay off most.

> **Etymology.** *Interface*: the face *between* two systems. *Bring-up* is engineering slang for getting new hardware to work step by step.

## Buses and links

| Link | Typical speed | Range | Use |
|---|---|---|---|
| I²C | 0.1–1 Mbit/s | On a board | IMUs, small sensors |
| SPI | 10–50 Mbit/s | On a board | Fast sensors, encoders, displays |
| UART / USB serial | 0.1–12 Mbit/s | Short cable | Microcontroller links (see [[Serial_Communication_Tutorials]]) |
| CAN | up to 1 Mbit/s | Tens of metres, very robust | Motor drivers, vehicles, small robots |
| CAN FD | Data phase typically up to 5–8 Mbit/s; 64-byte payloads | As CAN | Modern joint modules, humanoids |
| EtherCAT | 100 Mbit/s Ethernet, cycle times ≤ 100 µs, jitter ≤ 1 µs | Long chains | Industrial arms, fast legged and humanoid robots |
| Ethernet / GigE | 1–10 Gbit/s | Long | Cameras, lidar, computers |

## Worked example: a CAN bandwidth budget

A classic CAN frame with 8 data bytes takes about 130 bits on the wire (identifier, control, CRC and bit stuffing). At 1 Mbit/s that is about $10^6 / 130 \approx 7{,}700$ frames per second.

A 12-joint quadruped sends each motor one command frame and receives one state frame per control cycle: 24 frames per cycle. The maximum cycle rate is therefore $7{,}700 / 24 \approx 320$ Hz, before any margin. Splitting the motors over several CAN buses, moving to CAN FD (faster data phase, larger frames carrying several values), or using EtherCAT is how robots reach the 1 kHz control loops learned policies and whole-body controllers expect.

## The ROS 2 side: a ros2_control hardware interface

A hardware interface is a C++ plugin that implements:
- `on_init` (read parameters from the URDF `<ros2_control>` tag);
- `on_configure` / `on_activate` (open the bus, enable drives);
- `export_state_interfaces` / `export_command_interfaces`;
- `read()` and `write()`, called every control cycle.

Controllers then run unchanged in simulation and on hardware (see [[ros2_control]]). Sensors such as cameras and lidars usually have their own ROS 2 driver nodes that publish standard messages with correct frames and timestamps (see [[TF_and_Topic_Architecture]]).

## Bring-up checklist

1. **Power and safety first**: emergency stop wired and tested; current limits set low; robot supported or restrained.
2. **One device at a time**: talk to each motor or sensor alone before combining.
3. **Units and signs**: radians versus degrees, counts versus radians, positive direction matching the URDF joint axis.
4. **Zero offsets and limits**: calibrate joint zeros; set software limits inside the mechanical ones.
5. **Timing**: measure the actual loop rate and jitter; hardware timestamps where possible.
6. **Compare with simulation**: the same commands in sim and on hardware should move joints the same way; mismatches reveal sign or scale errors.
7. **Log everything** with [[rosbag2]] during bring-up.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Hardware_Interface_Setup]])
```
