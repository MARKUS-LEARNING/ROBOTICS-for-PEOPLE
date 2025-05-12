---
title: Hardware Shopping List
description: A curated list of potential hardware components, platforms, kits, tools, and suppliers for robotics projects.
tags:
  - hardware
  - shopping-list
  - components
  - parts
  - resources
  - references
  - electronics
  - wishlist
layout: default
category: robotics
author: Jordan_Smith_&_le_Chat
date: 2025-04-29
permalink: /hardware_shopping_list/
related:
  - "[[Resources_Index]]"
  - "[[Tools_References_and_Links]]"
  - "[[Sensor]]"
  - "[[Actuator]]"
  - "[[Microcontroller]]"
  - "[[Single-Board Computer]]"
  - "[[Arduino]]"
  - "[[Raspberry_Pi]]"
  - "[[NVIDIA_Jetson]]"
  - "[[TurtleBot]]"
---

# Hardware Shopping List & Inventory

This note serves as a central place to keep track of robotics hardware components, platforms, tools, and suppliers. Use it as a wishlist, an inventory of parts you own, or a reference for future projects.

See also: [[Resources_Index]], [[Tools_References_and_Links]]

---

## Organization

This list is organized by category. Feel free to add specific items, links, notes on pricing or status (e.g., #Have, #Need, #Wishlist), and potential suppliers. *Supplier examples are illustrative and not endorsements.* 

---

## Platforms & Kits

* **[[TurtleBot|TurtleBot 4]]:** Complete [[ROS (Robot Operating System)|ROS]]-based mobile robot platform for education and research. *(Mentioned in [[Resources_Index]])*
    * *Suppliers:* Clearpath Robotics, Robotis
* **Generic Rover Chassis Kits:** Various kits available for building wheeled mobile robots.
    * *Suppliers:* Adafruit, SparkFun, Pololu, RobotShop
* **Robot Arm Kits:** Kits for building small desktop manipulator arms.
    * *Suppliers:* Robotis (Dynamixel-based), uFactory (xArm), Trossen Robotics

## Computing Boards

* **[[Microcontroller|Microcontrollers (MCUs)]]:** For low-level control, real-time tasks, interfacing simple sensors/actuators.
    * [[Arduino]] (Uno, Mega, Nano, Portenta)
    * ESP32 (with WiFi/Bluetooth)
    * Teensy
    * STM32 Nucleo/Discovery Boards
    * *Suppliers:* Arduino Store, Adafruit, SparkFun, PJRC, STMicroelectronics
* **[[Single-Board Computer|Single-Board Computers (SBCs)]]:** For running Linux, ROS, performing heavier computation (AI, vision).
    * [[Raspberry Pi]] (e.g., Pi 4, Pi 5)
    * [[NVIDIA Jetson]] (Nano, Orin Nano, AGX Orin) - *Mentioned in [[Resources_Index]]*, specialized for AI/GPU acceleration.
    * BeagleBone (Black, AI)
    * *Suppliers:* Raspberry Pi Foundation, NVIDIA, BeagleBoard.org, Adafruit, SparkFun

## [[Sensor|Sensors]]

* **[[Camera_Systems|Cameras]]:**
    * USB Webcams (e.g., Logitech C920)
    * Raspberry Pi Cameras
    * [[RGB-D Sensor|RGB-D Cameras]] (e.g., Intel RealSense, [[OAK-D|OpenCV AI Kit OAK-D]] - *Mentioned in [[Resources_Index]]*, Orbbec Astra)
    * [[Stereo Vision|Stereo Cameras]] (e.g., Stereolabs ZED)
    * *Suppliers:* Logitech, Raspberry Pi, Intel, OpenCV, Stereolabs, Arducam
* **[[LIDAR]]:**
    * 2D LIDAR (e.g., RPLIDAR, Hokuyo)
    * 3D LIDAR (e.g., Ouster, Velodyne - often expensive)
    * Solid-State LIDAR (emerging)
    * *Suppliers:* Slamtec, Hokuyo, Ouster, Velodyne
* **[[IMU_Sensors|IMUs]]:** Inertial Measurement Units (Accelerometer + Gyroscope +/- Magnetometer).
    * Basic MEMS IMUs (e.g., MPU-6050, MPU-9250, BNO055, ICM-20948)
    * Higher-end IMUs (e.g., Xsens MTi series)
    * *Suppliers:* Adafruit, SparkFun, TDK InvenSense, Bosch Sensortec, Xsens
* **[[Ultrasonic_Sensors]]:** Simple, low-cost range finding (e.g., HC-SR04).
    * *Suppliers:* Adafruit, SparkFun, various online electronics retailers
* **Infrared (IR) Rangefinders:** Short-range distance sensing (e.g., Sharp IR sensors).
    * *Suppliers:* Adafruit, SparkFun, Pololu
* **GPS Modules:** For outdoor localization (consider modules with RTK capability for higher precision).
    * *Suppliers:* Adafruit, SparkFun, u-blox
* **Force/Torque Sensors:** Measure interaction forces (e.g., wrist sensors for manipulators, load cells). Often specialized and expensive.
    * *Suppliers:* Robotiq, ATI Industrial Automation
* **Encoders:** Measure motor/wheel rotation (incremental, absolute). Often integrated with motors or available separately.
    * *Suppliers:* US Digital, CUI Devices, motor manufacturers

## [[Actuator|Actuators]] & Motors

* **DC Motors:** Standard brushed motors, often used with gearboxes.
    * *Suppliers:* Pololu, Adafruit, SparkFun, ServoCity
* **Servo Motors:**
    * Hobby Servos (RC Servos): Position control, limited range, low cost.
    * Advanced Servos (e.g., Robotis Dynamixel): Networked, position/velocity/torque control, feedback, wide range.
    * *Suppliers:* Adafruit, SparkFun, Pololu (Hobby); Robotis (Dynamixel)
* **Stepper Motors:** Precise positioning via steps, often used in open-loop control. Require stepper drivers.
    * *Suppliers:* Adafruit, SparkFun, Pololu, StepperOnline
* **Motor Drivers:** Electronics to control motors from microcontrollers/SBCs (H-Bridges for DC, dedicated servo controllers, stepper drivers).
    * *Suppliers:* Pololu, Adafruit, SparkFun, Texas Instruments, Allegro MicroSystems

## Power Systems

* **Batteries:** LiPo (Lithium Polymer - high density, require care), Li-ion (Lithium-ion), NiMH (Nickel-Metal Hydride), Lead-Acid (heavy). Consider voltage, capacity (mAh/Ah), discharge rate (C rating). **Safety is paramount with LiPo/Li-ion.**
* **Battery Management Systems (BMS):** Protect Lithium batteries from overcharge, over-discharge, over-current. Essential for safety and longevity.
* **Voltage Regulators:** Step-up (Boost) and Step-down (Buck) converters to provide stable voltages required by different components from the main battery voltage.
* **Power Distribution Boards (PDBs):** Simplify wiring and power distribution.
* **Chargers:** Appropriate chargers for the specific battery chemistry.
* *Suppliers:* HobbyKing, Adafruit, SparkFun, Tenergy, battery-specific suppliers.

## Structural Components & Miscellaneous

* **Chassis Materials:** Aluminum extrusion (e.g., 80/20, MakerBeam), acrylic sheets, plywood, 3D printed parts.
* **Fasteners:** Metric screws (M2, M3, M4, M5), nuts, washers, standoffs.
* **Wires & Connectors:** Various gauges of wire (silicone wire recommended for flexibility), JST connectors, Dupont connectors, XT60/XT30 (for power), terminal blocks.
* **Prototyping:** Breadboards, perfboards/protoboards.
* **Bearings, Couplers, Gears, Belts/Pulleys:** Mechanical transmission components.
* *Suppliers:* McMaster-Carr, Misumi, OpenBuilds Part Store, Adafruit, SparkFun, Amazon, local hardware stores.

## Tools

* **Basic Hand Tools:** Screwdrivers, pliers, wire strippers, wire cutters, Allen keys, wrenches.
* **Soldering:** Soldering iron, solder, flux, helping hands, fume extractor.
* **Measurement:** Digital Multimeter (DMM), digital calipers.
* **Debugging:** Logic Analyzer, Oscilloscope (more advanced).
* **Fabrication:** 3D Printer, potentially CNC mill or laser cutter.

## Potential Suppliers (General Robotics/Electronics)

* Adafruit
* SparkFun Electronics
* Pololu Robotics & Electronics
* RobotShop
* ServoCity
* Mouser Electronics
* Digi-Key Electronics
* Amazon
* AliExpress

*(Note: This list is illustrative and serves as a starting point. Add items specific to your projects and interests!)*

