---
title: Arduino
description: "Arduino is the open hardware and software platform most people first use to make things move: microcontroller boards, a simple C++ framework and a huge library ecosystem. Covers the boards that matter in 2026 (UNO R4, Nano RP2040 Connect, Portenta), a computed serial-bandwidth example, typical robot jobs (motor control, sensors, safety I/O), connecting to ROS 2 via micro-ROS, and when to move to a more capable microcontroller."
aliases:
  - Arduino Uno
  - Arduino IDE
tags:
  - electronics
  - embedded-systems
type: tool
level: beginner
subfield:
  - electronics-embedded
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Microcontroller]]"
related:
  - "[[Serial_Communication_Tutorials]]"
  - "[[Hardware_Interface_Setup]]"
  - "[[Python_ROS_Nodes]]"
key_takeaways:
  - "Arduino boards run one program in a loop with direct access to pins, timers and buses: ideal for reading sensors and driving motors close to the hardware."
  - "The UNO R4 (2023) moved the classic UNO to a 32-bit 48 MHz Arm core with 32 kB SRAM; heavier robot firmware uses Teensy, ESP32, RP2040 or Portenta boards."
  - "At 115,200 baud a serial link carries about 11.5 kB/s, roughly 360 messages per second of 32 bytes, enough for sensor data and commands at a few hundred hertz; micro-ROS lets supported boards join ROS 2 directly."
sources:
  - "Arduino (2026). UNO R4 Minima documentation. https://docs.arduino.cc/hardware/uno-r4-minima/"
  - "micro-ROS (2026). micro-ROS: ROS 2 for microcontrollers. https://micro.ros.org/"
  - "micro-ROS contributors (2026). micro_ros_arduino: supported boards and precompiled library. https://github.com/micro-ROS/micro_ros_arduino"
---

# Arduino

**Arduino** is an open-source electronics platform: inexpensive microcontroller boards, a simple C++-based programming framework, and thousands of libraries for sensors, motors and displays. For many people it is the first robot brain they program, and it stays useful in professional robots for low-level jobs.

> **Etymology.** Named after the Bar di Re Arduino in Ivrea, Italy, where the founders met; the bar was named after Arduin of Ivrea, an 11th-century king of Italy. The project started in 2005 at the Interaction Design Institute Ivrea.

## How an Arduino program looks

```cpp
#include <Servo.h>
Servo s;
void setup() { s.attach(9); Serial.begin(115200); }
void loop() {
  int raw = analogRead(A0);                 // read a potentiometer, 14-bit on the UNO R4
  int angle = map(raw, 0, 16383, 0, 180);
  s.write(angle);                           // move a hobby servo
  Serial.println(angle);
  delay(10);                                // about 100 Hz
}
```

`setup()` runs once; `loop()` runs forever. There is no operating system: the program owns the hardware, which makes timing simple and predictable for small tasks.

## Boards in 2026

| Board | Processor | Notes |
|---|---|---|
| **UNO R4 Minima / WiFi** (2023) | Renesas RA4M1, 48 MHz Arm Cortex-M4, 32 kB SRAM, 256 kB flash | The classic UNO form factor, now 32-bit; 14-bit analog inputs, DAC, CAN |
| **Nano RP2040 Connect** | Raspberry Pi RP2040 dual-core | Wi-Fi/Bluetooth, IMU; micro-ROS supported |
| **Portenta H7** | Dual-core STM32H7 | Industrial-grade; micro-ROS supported |
| Compatible boards (Teensy 4.x, ESP32) | Fast Arm or Xtensa cores | Popular for robot firmware; Arduino framework available |

## Worked example: how much fits through the serial port?

A UART at 115,200 baud sends 10 bits per byte (start, 8 data bits, stop), so about 11,520 bytes per second. Messages of 32 bytes (a few sensor readings plus a header) fit about $11{,}520/32 = 360$ times per second. That is fine for 100 Hz sensor streaming and commands, too little for camera images, and tight for kilohertz control of many motors. Faster serial rates, USB, CAN or Ethernet are the next steps (see [[Serial_Communication_Tutorials]], [[Hardware_Interface_Setup]]).

## Arduino in a ROS 2 robot

Common patterns:
1. **Serial bridge**: the Arduino reads sensors and drives motors; a node on the main computer talks to it over USB serial (see [[Python_ROS_Nodes]]).
2. **micro-ROS**: on supported boards (Portenta H7, Nano RP2040 Connect, Teensy, ESP32, OpenCR and others), the board runs a micro-ROS client and appears as ROS 2 nodes through a micro-ROS agent on the main computer.
3. **ros2_control hardware interface**: a C++ plugin on the main computer speaks the board's protocol (see [[Hardware_Interface_Setup]]).

## When to move on

Use a more capable microcontroller (see [[Microcontroller]]) when you need hard real-time control of many motors, field-oriented control of brushless motors, CAN FD or EtherCAT, or safety certification.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Arduino]])
```
