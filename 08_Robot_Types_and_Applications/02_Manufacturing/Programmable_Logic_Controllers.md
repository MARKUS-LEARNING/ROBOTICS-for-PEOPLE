---
title: Programmable Logic Controllers
description: "The programmable logic controller (PLC) is the rugged computer that runs factory machines and sequences robot cells. Covers the scan cycle and what it means for timing (computed), the IEC 61131-3 languages as of the 2025 edition, a robot handshake in Structured Text, fieldbuses and safety controllers, OPC UA for connecting robots to factory software, and how PLCs and ROS 2 share the work."
aliases:
  - PLC
  - PLCs
  - Programmable Controller
  - IEC 61131-3
tags:
  - plc
  - manufacturing
  - automation
  - real-time
type: component
level: intermediate
subfield:
  - industrial-automation
  - electronics-embedded
status: reviewed
author: Jordan_Smith
date: 2026-11-15
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Manufacturing]]"
related:
  - "[[Robot_Work_Cell_Design]]"
  - "[[Hardware_Interface_Setup]]"
  - "[[Microcontroller]]"
  - "[[Control_Systems]]"
  - "[[ros2_control]]"
key_takeaways:
  - "A PLC repeats one loop for ever: read all inputs, run the program, write all outputs. That scan makes its behaviour predictable and easy to diagnose."
  - "Timing follows from the scan: with a 10 ms scan a part on a 1 m/s conveyor moves 10 mm per scan, and the worst-case reaction to a sensor is roughly two scans plus input and output delays."
  - "In a robot cell the PLC is usually the conductor: it sequences robots, machines and conveyors over a fieldbus and reports upward over OPC UA, while a separate safety controller handles emergency stops and guards."
sources:
  - "IEC (2025). IEC 61131-3:2025 Programmable controllers – Part 3: Programming languages (edition 4.0). https://webstore.iec.ch/en/publication/68533"
  - "PLCopen (2026). PLCopen standards: logic, motion control, safety, communication. https://plcopen.org/iec-61131-3"
  - "AutomationDirect (2015). History of the PLC. https://library.automationdirect.com/history-of-the-plc"
  - "OPC Foundation and VDMA (2025). OPC 40010-1: OPC UA for Robotics – Part 1: Vertical Integration (release 1.02). https://reference.opcfoundation.org/Robotics/v100/docs/"
  - "International Federation of Robotics (2026). Top 5 Global Robotics Trends 2026. https://ifr.org/ifr-press-releases/news/top-5-global-robotics-trends-2026"
  - "EtherCAT Technology Group (2026). EtherCAT technology. https://www.ethercat.org/en/technology.html"
---

# Programmable Logic Controllers

A **programmable logic controller (PLC)** is an industrial computer built to run machines: it reads sensors and switches, decides, and drives motors, valves and lamps, all day, for decades, in heat, dust and electrical noise. Almost every production line is run by PLCs, and a robot on that line takes its start signal from one.

> **Etymology.** *Programmable logic*: the PLC replaced cabinets of hard-wired relay logic with a program. Dick Morley of Bedford Associates sketched the idea on New Year's Day 1968, in response to General Motors' Hydra-Matic division asking for a "standard machine controller". The first unit was the Modicon 084, named for the firm's 84th project.

## How a PLC works: the scan

A PLC runs the same loop for ever:

1. **Read** every input into memory (the input image).
2. **Execute** the program, top to bottom, using that frozen snapshot.
3. **Write** every output.
4. Housekeeping and communication, then start again.

One loop is a **scan**, typically a few milliseconds to a few tens of milliseconds. Because inputs are frozen during the scan, the logic sees a consistent picture of the machine, which makes behaviour repeatable and faults traceable.

### Worked example: what the scan does to timing

A conveyor runs at 1 m/s and the PLC scan takes 10 ms.

- **Position uncertainty.** A part travels $1\ \text{m/s} \times 0.010\ \text{s} = 10$ mm during one scan. Anything triggered "when the sensor turns on" is up to 10 mm late, by a different amount each time.
- **Worst-case reaction.** If the sensor changes just after the inputs were read, the change waits a full scan to be seen and is acted on at the end of the next one. With a 3 ms input filter and a 1 ms output delay, the worst case is $3 + 2 \times 10 + 1 = 24$ ms, or 24 mm of belt travel.
- **Missed pulses.** A 5 ms pulse can fall entirely between two input reads and never be seen.

So fast or precise events are not handled in the ordinary scan. PLCs provide interrupt and high-speed counter inputs, and motion controllers or robot controllers with sub-millisecond loops track conveyors and trigger cameras.

## Programming languages: IEC 61131-3

The international standard for PLC languages is **IEC 61131-3**. Its fourth edition, published in May 2025, defines:

| Language | Looks like | Typical use |
|---|---|---|
| **Ladder Diagram (LD)** | Relay wiring diagrams: contacts and coils on rungs | Interlocks and discrete logic; read by electricians everywhere |
| **Function Block Diagram (FBD)** | Blocks wired together | Process and motion functions |
| **Structured Text (ST)** | A Pascal-like programming language | Calculations, state machines, data handling |
| **Sequential Function Chart (SFC)** elements | Steps and transitions | Structuring a machine's sequence |

The 2025 edition lists Structured Text as its one textual language; the older Instruction List, deprecated in 2013, is no longer listed. It also adds UTF-8 strings. PLCopen builds on the standard with common function blocks for motion control and safety.

## A robot handshake in Structured Text

The PLC and the robot controller coordinate through a few signals. A minimal sequence:

```iecst
CASE state OF
  0:  (* Idle: wait for a part and a ready robot *)
      IF PartPresent AND RobotReady AND NOT Fault THEN
          StartCycle := TRUE;
          state := 1;
      END_IF;
  1:  (* Robot has accepted the start *)
      IF RobotBusy THEN
          StartCycle := FALSE;
          state := 2;
      END_IF;
  2:  (* Wait for the robot to finish *)
      IF CycleDone THEN
          PartCount := PartCount + 1;
          state := 0;
      END_IF;
END_CASE;
```

Each signal is acknowledged before the next step, so neither side can run ahead of the other. Emergency stops and guard doors are *not* handled by logic like this: they belong to safety-rated hardware.

## PLC, robot controller and safety controller

| Device | Job |
|---|---|
| **PLC** | Sequences the cell: conveyors, clamps, machine doors, robots; counts parts; reports status |
| **Robot controller** | Plans and executes the robot's motion at a high rate; runs the robot program (see [[Industrial_Arms]]) |
| **Safety controller** (safety PLC) | Monitors emergency stops, door interlocks, light curtains and scanners with redundant, self-checking hardware; removes power or enforces safe speed |

They talk over an industrial Ethernet **fieldbus** such as PROFINET, EtherNet/IP or EtherCAT. EtherCAT was developed for cycle times of 100 µs or less with jitter of 1 µs or less (see [[Hardware_Interface_Setup]]).

## Connecting the factory floor to software: OPC UA

**OPC UA** is the vendor-neutral standard for exchanging structured data between machines and factory software. For robots, the companion specification *OPC UA for Robotics, Part 1: Vertical Integration* (release 1.02, September 2025) defines a common model of a robot system, its axes, status and condition data. A maintenance or production system can then read any vendor's robot the same way.

This is the practical side of what the IFR lists among its 2026 trends as the convergence of information technology (IT) and operational technology (OT): production data flowing from machines into analytics, scheduling and AI tools.

## PLCs and robot software such as ROS 2

Research and newer robot applications are often written with ROS 2. In a factory the two coexist:
- the **PLC and safety controller** keep the line sequence and the safety functions, which must be deterministic and certifiable;
- **ROS 2 nodes** handle perception, planning and learned skills (see [[MoveIt2]], [[Machine_Vision_in_Manufacturing]]);
- they meet at a fieldbus interface, a [[ros2_control]] hardware interface, or an OPC UA connection, exchanging the same kind of handshake shown above.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Programmable_Logic_Controllers]])
```
