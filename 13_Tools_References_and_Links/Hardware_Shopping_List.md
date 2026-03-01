---
title: Hardware Shopping List
description: A curated reference of robotics hardware platforms, computing boards, sensors, actuators, and tools — organized by category and budget tier — to guide hardware selection for projects from hobbyist to research grade.
tags:
  - hardware
  - shopping-list
  - components
  - parts
  - resources
  - references
  - electronics
  - wishlist
  - sensors
  - actuators
layout: default
category: robotics
author: Jordan_Smith_&_Claude
date: 2025-05-02
permalink: /hardware_shopping_list/
related:
  - "[[Resources_Index]]"
  - "[[Microcontroller]]"
  - "[[LIDAR]]"
  - "[[Camera_Systems]]"
  - "[[IMU_Sensors]]"
  - "[[Actuators]]"
  - "[[Arduino]]"
  - "[[Python_for_Robotics]]"
  - "[[ROS_2_Overview]]"
  - "[[ros2_control]]"
  - "[[Gazebo_Simulator]]"
  - "[[Sim2Real_Experiments]]"
---

# Hardware Shopping List & Reference

A curated reference for robotics hardware selection — from hobby kits to research platforms. Organized by component category with budget tiers and application guidance.

*Supplier examples are illustrative and not endorsements. Prices change frequently — verify current pricing before purchasing.*

See also: [[Resources_Index]], [[ros2_control]], [[Sim2Real_Experiments]]

---

## Complete Robot Platforms

Ready-to-use or near-complete platforms that integrate computing, sensing, actuation, and software support.

### Mobile Robot Platforms

| Platform | Price Tier | ROS 2 Support | Best For |
|---|---|---|---|
| **TurtleBot 4** (Clearpath/Robotis) | $$$ (~$1,500) | ✅ Native | Education, research, Nav2 development |
| **TurtleBot 4 Lite** | $$ (~$400) | ✅ Native | Entry-level ROS 2 mobile robot |
| **Robotis OP3** | $$$$ | ✅ | Humanoid research platform |
| **Husarion ROSbot 2R** | $$$ (~$1,200) | ✅ Native | ROS 2 research platform with LiDAR |
| **AgileX Scout Mini** | $$$$ | ✅ | Outdoor research, rough terrain |
| **Create 3** (iRobot) | $ (~$300) | ✅ Native | Mobile robotics education base |
| **Limo** (AgileX) | $$ (~$800) | ✅ | Compact indoor/outdoor research |
| **Generic 4WD Rover Chassis** | $ (~$30–80) | ⚙️ DIY | Build-your-own mobile base |

### Manipulator Arms

| Platform | DOF | Price Tier | Best For |
|---|---|---|---|
| **Trossen WidowX 250 S** | 6 | $$$ (~$1,500) | ROS 2 / MoveIt 2 manipulation research |
| **Robotis OpenMANIPULATOR-X** | 4 | $$ (~$800) | ROS 2 + Dynamixel; education |
| **xArm 6 / xArm 7** (uFactory) | 6/7 | $$$ (~$3,000–5,000) | Mid-range industrial-grade arm |
| **Franka Panda / FR3** | 7 | $$$$ (~$15,000) | Research manipulation; torque-controlled |
| **Universal Robots UR3e/UR5e** | 6 | $$$$ | Industry-standard cobot |
| **SO-ARM100** | 6 | $ (~$100) | Low-cost learning/LeRobot compatible |
| **Koch v1.1** | 6 | $ (~$250) | Open-source; teleoperation + imitation learning |

### Legged Robots

| Platform | Price Tier | Notes |
|---|---|---|
| **Unitree Go2** | $$$ (~$2,700) | Quadruped; SDK + ROS 2 bridge available |
| **Unitree G1** | $$$$ (~$16,000) | Humanoid; research-grade |
| **Boston Dynamics Spot** | $$$$ (~$75,000) | Industrial-grade quadruped |
| **Anybotics ANYmal** | $$$$ | Research quadruped; force-controlled |
| **Petoi Bittle X** | $ (~$300) | Mini quadruped; beginner/education |
| **Stanford Pupper / MangDang Mini Cheetah** | $$ | Open-source DIY quadruped |

---

## Computing Boards

### Microcontrollers (MCUs)

For real-time low-level control, hardware interfacing, and embedded systems. Typically run bare-metal firmware or RTOS.

| Board | MCU | Best For | Price |
|---|---|---|---|
| **Arduino Uno R4** | RA4M1 (ARM Cortex-M4) | Education, basic I/O | ~$27 |
| **Arduino Mega 2560** | ATmega2560 | Many I/O pins, simple sensors | ~$48 |
| **Teensy 4.1** | iMXRT1062 (ARM Cortex-M7, 600 MHz) | High-speed real-time control, micro-ROS | ~$30 |
| **ESP32-S3** | Xtensa LX7 dual-core + WiFi/BT | Wireless robotics, IoT integration | ~$10 |
| **STM32 Nucleo-F446RE** | STM32F446 (ARM Cortex-M4) | Real-time control, ros2_control hardware interfaces | ~$15 |
| **Raspberry Pi Pico 2** | RP2350 dual-core | Low-cost embedded, micro-ROS | ~$5 |

**micro-ROS**: Enables ROS 2 communication directly on MCUs. Supported on ESP32, Teensy, STM32, and others — allowing MCUs to act as first-class ROS 2 nodes.

### Single-Board Computers (SBCs)

Run full Linux + ROS 2. Used for onboard computation, perception, and high-level decision making.

| Board | CPU | GPU/AI | RAM | Best For | Price |
|---|---|---|---|---|---|
| **Raspberry Pi 5** | Cortex-A76 quad 2.4GHz | — | 4/8 GB | ROS 2, navigation, lightweight vision | ~$80 |
| **NVIDIA Jetson Orin Nano** | Cortex-A78AE 6-core | 1024 CUDA cores | 8 GB | AI inference, perception pipelines | ~$250 |
| **NVIDIA Jetson Orin NX 16** | Cortex-A78AE 8-core | 1024 CUDA cores | 16 GB | Advanced perception, RL deployment | ~$500 |
| **NVIDIA Jetson AGX Orin** | Cortex-A78AE 12-core | 2048 CUDA cores | 32/64 GB | Full research compute on robot | ~$1,000 |
| **BeagleBone AI-64** | TDA4VM dual Cortex-A72 | MMA accelerator | 4 GB | Real-time + AI hybrid tasks | ~$240 |
| **Khadas VIM4** | Cortex-A73 quad + A53 quad | NPUR 5 TOPs | 8 GB | Budget AI compute | ~$130 |

**Jetson Selection Guide:**
- *Teaching / sensor logging*: Raspberry Pi 5
- *Object detection / vision*: Jetson Orin Nano
- *Full perception stack + RL inference*: Jetson Orin NX
- *Research platform / multiple cameras + LiDAR*: Jetson AGX Orin

---

## Sensors

### Cameras

| Sensor | Type | Resolution | Best For | Price |
|---|---|---|---|---|
| **Logitech C920 / C270** | RGB USB | 1080p / 720p | Basic vision, low cost | ~$50–$80 |
| **Intel RealSense D435i** | RGB-D + IMU | 1280×720 depth | SLAM, 3D perception, manipulation | ~$200 |
| **Intel RealSense D455** | RGB-D | Wide FOV | Navigation, large workspace perception | ~$280 |
| **Luxonis OAK-D Pro** | RGB-D + Stereo + IMU | 4K RGB | On-device AI inference (MyriadX VPU) | ~$350 |
| **Luxonis OAK-D Lite** | RGB-D | 1080p | Budget on-device vision | ~$150 |
| **Stereolabs ZED 2i** | Stereo RGB-D + IMU | 2.2K | Outdoor SLAM, long-range depth | ~$450 |
| **Stereolabs ZED X** | Stereo RGB-D + IMU | 4K | Jetson-native, GMSL connector | ~$550 |
| **FLIR Blackfly S** | Industrial RGB | Up to 20MP | High-quality machine vision | ~$500+ |

### LiDAR

| Sensor | Type | Range | Beams | Best For | Price |
|---|---|---|---|---|---|
| **RPLIDAR A1/A2** | 2D | 12m / 18m | 1 | Education, basic SLAM | ~$100–$150 |
| **RPLIDAR A3** | 2D | 25m | 1 | Indoor navigation | ~$250 |
| **Hokuyo URG-04LX** | 2D | 5.6m | 1 | Research-grade 2D SLAM | ~$1,000 |
| **Velodyne VLP-16 (Puck)** | 3D | 100m | 16 | Outdoor SLAM, autonomous vehicles | ~$4,000 |
| **Ouster OS0-32** | 3D | 50m | 32 | Dense 3D mapping, research | ~$3,500 |
| **Livox Mid-360** | 3D solid-state | 40m | Non-repetitive | Indoor 3D SLAM, cost-effective | ~$1,000 |
| **Livox Avia** | 3D solid-state | 450m | Non-repetitive | Outdoor mapping, long range | ~$3,000 |

**2D vs. 3D LiDAR:** 2D LiDAR (single beam rotating) is sufficient for flat indoor environments and standard Nav2. 3D LiDAR enables full 3D mapping, outdoor navigation, and richer perception but costs significantly more.

### IMUs

| Sensor | DOF | Grade | Interface | Price |
|---|---|---|---|---|
| **MPU-6050** | 6 (Accel + Gyro) | MEMS consumer | I2C | ~$3 |
| **BNO055** | 9 (+ Magnetometer, integrated fusion) | MEMS consumer | I2C/UART | ~$10 |
| **ICM-42688-P** | 6 | MEMS high-performance | SPI/I2C | ~$5 |
| **VectorNav VN-100** | 9 + AHRS | Tactical | RS-232/SPI | ~$500 |
| **Xsens MTi-30** | 9 + AHRS | Industrial | USB/RS-422 | ~$1,500 |
| **Microstrain 3DM-GX5** | 9 + AHRS | Research | USB | ~$2,000 |

### Force/Torque and Tactile Sensors

| Sensor | Type | Best For | Price |
|---|---|---|---|
| **ATI Mini45** | 6-axis F/T | Wrist sensing, manipulation | ~$6,000 |
| **Robotiq FT 300-S** | 6-axis F/T | Cobot force sensing, compliant control | ~$4,500 |
| **OnRobot HEX-H** | 6-axis F/T | Assembly, contact-rich manipulation | ~$3,500 |
| **GelSight / DIGIT** | Tactile (optical) | Dexterous manipulation research | ~$500 |
| **Load Cells (generic)** | 1-axis force | Simple force feedback | ~$5–30 |

---

## Actuators and Motors

### Servo and Smart Actuators

| Actuator | Control | Feedback | Best For | Price |
|---|---|---|---|---|
| **Robotis Dynamixel XL-430** | Position/Velocity/Current | Position, velocity, load | Educational arms, low-torque joints | ~$50 |
| **Robotis Dynamixel XM540** | Position/Velocity/Current | Full state | Research manipulators, mid-torque | ~$200 |
| **Robotis Dynamixel PRO** | Position/Velocity/Torque | Full state | High-torque research joints | ~$500+ |
| **Hebi Robotics X5-4** | Position/Velocity/Torque + compliant | Full state | Research arms, safe interaction | ~$2,000 |
| **T-Motor AK10-9** | Torque (quasi-direct drive) | Position, velocity, torque | Legged robots, compliant joints | ~$400 |
| **Hobby Servo (e.g., MG996R)** | Position (PWM) | None | Low-cost grippers, simple joints | ~$5–15 |

### DC Motors

| Type | Driver Needed | Best For |
|---|---|---|
| Standard DC gearmotor (Pololu, etc.) | H-Bridge (L298N, DRV8833) | Differential drive mobile bases |
| BLDC Outrunner (drones, wheels) | ESC or FOC driver (ODrive, VESC) | High-speed wheels, gimbal drives |
| Stepper Motor (NEMA 17/23) | Stepper driver (A4988, TMC2209) | Precise positioning, open-loop axes |

**ODrive / VESC:** Open-source brushless motor controllers that enable high-performance field-oriented control (FOC). Popular for custom mobile bases and legged robots.

---

## Power Systems

| Component | Key Considerations |
|---|---|
| **LiPo Batteries** | High energy density; require BMS, proper charger, fire-safe storage. Specify voltage (S rating: 3S=11.1V, 4S=14.8V) and capacity (mAh). |
| **Li-ion Packs** | Safer than LiPo; common in larger platforms (e.g., 24V e-bike packs for mobile bases). |
| **BMS (Battery Management System)** | Essential for any lithium battery. Prevents overcharge, over-discharge, over-current. |
| **Buck/Boost Converters** | Step voltage levels for different components (e.g., 12V→5V for SBCs, 12V→3.3V for MCUs). Use switching regulators, not linear, for efficiency. |
| **Power Distribution Board (PDB)** | Simplifies wiring; distributes main battery to multiple loads with fusing. |
| **XT60 / XT30 Connectors** | Standard high-current connectors for LiPo/Li-ion main power. |
| **Chargers** | Must match battery chemistry and cell count. iMax B6 or ISDT Q6 are popular balanced chargers. |

---

## Structural and Fabrication

| Category | Options |
|---|---|
| **Aluminum Extrusion** | 2020/4040 T-slot (MakerBeam, Misumi, OpenBuilds); modular, strong, machinable |
| **Sheet Metal / Acrylic** | Laser-cut for custom chassis; acrylic for light-duty, aluminum for robust |
| **3D Printing** | FDM (PLA/PETG/TPU) for prototyping; SLS/resin for production-quality parts |
| **Bearings** | MR series (miniature) for joints; standard 608 for wheels |
| **Shaft Couplers** | Rigid, jaw (Oldham), or flexible; must match shaft diameters and torque |
| **Timing Belts/Pulleys** | GT2 profile standard for robotics; low backlash |
| **Fasteners** | M2, M3, M4 metric (ISO standard); keep an assortment on hand |

---

## Test and Measurement Tools

| Tool | Purpose | Budget Option |
|---|---|---|
| **Digital Multimeter (DMM)** | Voltage, current, resistance, continuity | Fluke 117 (~$120), UT61E (~$40) |
| **Oscilloscope** | Signal debugging, PWM analysis, communication buses | Rigol DS1054Z (~$350), Hantek 2D72 (~$80) |
| **Logic Analyzer** | SPI, I2C, UART, CAN decode | Saleae Logic 8 (~$150), Cypress FX2 clones (~$10) |
| **Digital Calipers** | Mechanical measurements | Mitutoyo 500-196 (~$100), Neiko (~$20) |
| **Bench Power Supply** | Stable variable voltage during development | Korad KA3005P (~$50), Rigol DP832 (~$300) |
| **Soldering Iron** | PCB work, wire termination | Hakko FX-888D (~$120), TS80P (~$80) |
| **3D Printer** | Rapid custom parts | Bambu Lab A1 Mini (~$300), Prusa MK4 (~$800) |

---

## Key Suppliers

| Supplier | Speciality |
|---|---|
| **Adafruit** | Breakout boards, sensors, educational kits, great documentation |
| **SparkFun Electronics** | Similar to Adafruit; strong maker community |
| **Pololu Robotics** | Motor drivers, gearmotors, stepper drivers, chassis kits |
| **Robotis** | Dynamixel servos, OpenManipulator, TurtleBot |
| **Mouser / Digi-Key** | Full component inventory; for production-grade sourcing |
| **McMaster-Carr** | Structural hardware, fasteners, mechanical components (US) |
| **Misumi** | Aluminum extrusion, precision mechanical components |
| **ServoCity** | Servo accessories, structural components, hardware kits |
| **RobotShop** | Broad robotics marketplace |
| **AliExpress / LCSC** | Budget components, Chinese suppliers; longer lead times |
| **Amazon** | Convenience; verify specifications carefully |

---

## Budget Tier Guide

| Tier | Budget | Platform Examples |
|---|---|---|
| **Hobbyist** | <$500 | Arduino + chassis + HC-SR04 + RPLiDAR A1 + Raspberry Pi 5 |
| **Student/Maker** | $500–$2,000 | TurtleBot 4 Lite, custom arm with Dynamixel XL, Jetson Orin Nano |
| **Research Entry** | $2,000–$10,000 | TurtleBot 4, WidowX 250 S, Ouster OS0-32, ZED 2i, Jetson AGX Orin |
| **Full Research** | >$10,000 | Unitree Go2 + AGX Orin + full sensor suite, or Franka FR3 + workstation |

---

## Dataview Plugin Features

```dataview
LIST FROM #hardware OR #components WHERE contains(file.outlinks, [[Hardware_Shopping_List]])
```

```dataview
TABLE title, description FROM #resources WHERE contains(file.tags, "hardware")
```
