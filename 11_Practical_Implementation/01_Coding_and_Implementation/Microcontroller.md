---
title: Microcontroller
description: "Microcontrollers are the small computers inside robot joints, motor drivers, sensors and battery packs. Covers what distinguishes them from single-board computers, the main families in 2026 (Arm Cortex-M, ESP32, RP2040/RP2350, STM32), peripherals robots rely on (timers, PWM, ADC, CAN), a computed PWM-resolution example, real-time firmware with or without an RTOS, micro-ROS, and embedded Rust."
aliases:
  - MCU
  - Microcontrollers
  - Embedded Controller
tags:
  - embedded-systems
  - electronics
type: component
level: intermediate
subfield:
  - electronics-embedded
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Robots]]"
related:
  - "[[Arduino]]"
  - "[[Hardware_Interface_Setup]]"
  - "[[Rust]]"
  - "[[ros2_control]]"
key_takeaways:
  - "A microcontroller is a whole computer on one chip (CPU, memory, timers, converters, communication interfaces) built for deterministic control at low power and cost."
  - "Peripherals do the hard real-time work: a 72 MHz timer generating 20 kHz PWM counts to 3,600, about 11.8 bits of duty-cycle resolution."
  - "Robots use many: one per motor driver or joint, plus sensors and power management; micro-ROS and embedded Rust are the modern ways to program them alongside classic C/C++ with an RTOS."
sources:
  - "micro-ROS (2026). micro-ROS: ROS 2 for microcontrollers. https://micro.ros.org/"
  - "Arduino (2026). UNO R4 Minima documentation. https://docs.arduino.cc/hardware/uno-r4-minima/"
  - "CAN in Automation (2026). CAN knowledge. https://www.can-cia.org/can-knowledge/"
  - "Klabnik, S. and Nichols, C. (2026). The Rust Programming Language. https://doc.rust-lang.org/book/"
---

# Microcontroller

A **microcontroller (MCU)** is a complete small computer on a single chip: processor, memory and the peripherals needed to sense and control hardware. A modern robot contains many of them: in each motor driver, joint module, sensor, battery pack and safety circuit. They do the work that must happen every microsecond, reliably, while the main computer thinks.

> **Etymology.** *Micro* + *controller*. Intel's 8048 (1976) and 8051 (1980) were early widely used microcontrollers.

## Microcontroller versus single-board computer

| | Microcontroller | Single-board computer (e.g. Raspberry Pi, Jetson) |
|---|---|---|
| Runs | Bare-metal firmware or a small RTOS | Linux |
| Timing | Deterministic, microsecond-level | Variable unless tuned for real time |
| Memory | Kilobytes to a few megabytes | Gigabytes |
| Power | Milliwatts to watts | Watts to tens of watts |
| Robot role | Motor control, sensor sampling, safety I/O | Perception, planning, AI, ROS 2 |

## Common families (2026)

- **Arm Cortex-M** (STM32, NXP, Renesas and others): the industry standard for motor control and sensors; M4/M7/M33/M55 cores with floating point and DSP instructions.
- **ESP32** (Espressif): built-in Wi-Fi and Bluetooth, popular for connected hobby and educational robots.
- **RP2040 / RP2350** (Raspberry Pi): dual-core, inexpensive, with programmable I/O state machines.
- **Arduino boards**: friendly entry points built on these chips (see [[Arduino]]).

## Peripherals robots depend on

Timers and **PWM** for motor drive; **ADCs** for current and voltage sensing; encoder interfaces; **UART, SPI, I²C** for sensors; **CAN / CAN FD** and sometimes EtherCAT for joint communication; watchdogs and safety features.

## Worked example: PWM resolution

A motor driver switches at 20 kHz (above human hearing). A timer clocked at 72 MHz counts $72{,}000{,}000 / 20{,}000 = 3{,}600$ ticks per PWM period, so the duty cycle can be set in steps of $1/3600$: $\log_2 3600 \approx 11.8$ bits of resolution. Raising the PWM frequency to 40 kHz halves the counts (about 10.8 bits). Faster switching makes motors quieter and smoother but costs resolution and switching losses, a classic firmware trade-off.

## Programming approaches

- **Bare metal**: a main loop plus interrupts. Simple, but timing can get tangled as the firmware grows.
- **RTOS** (FreeRTOS, Zephyr, NuttX): tasks with priorities and deterministic scheduling.
- **micro-ROS**: runs on top of an RTOS and makes the MCU a ROS 2 participant through an agent on the main computer.
- **Embedded Rust** (with frameworks such as Embassy): memory-safe firmware (see [[Rust]]).

## In 2026 robots

Humanoids and quadrupeds have one or more MCUs per joint running field-oriented control of brushless motors at 10–40 kHz, communicating over CAN FD or EtherCAT with a central computer that runs the 1 kHz loop in [[ros2_control]] (see [[Hardware_Interface_Setup]]).

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Microcontroller]])
```
