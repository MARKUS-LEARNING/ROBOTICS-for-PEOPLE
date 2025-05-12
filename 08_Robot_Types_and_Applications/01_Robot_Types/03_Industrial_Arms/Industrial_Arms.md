---
title: Industrial Arms
description: Overview of industrial robot arms (manipulators), including their history, common types, characteristics, applications in automation, programming, safety, and the rise of collaborative robots (cobots).
tags:
  - robot-types
  - industrial-robot
  - manipulator
  - automation
  - manufacturing
  - cobot
  - robotics-history
layout: default
category: robotics
author: Jordan_Smith_&_le_Chat
date: 2025-04-28
permalink: /industrial_arms/
related:
  - "[[Robots]]"
  - "[[Manipulator_Arm_Types]]"
  - "[[Kinematics]]"
  - "[[Dynamics]]"
  - "[[Control_Theory]]"
  - "[[Workspace]]"
  - "[[Degrees_of_Freedom]]"
  - "[[Trajectory_Planning]]"
  - "[[Safety]]"
  - "[[Collaborative_Robots]]"
  - "[[Robot_Types_and_Applications]]"
---

# Industrial Arms (Industrial Robots)

**Industrial Arms**, often referred to simply as **Industrial Robots**, are automated mechanical manipulators used extensively in manufacturing and other industrial settings. According to the ISO 8373 standard, an industrial robot is an "automatically controlled, reprogrammable, multipurpose manipulator programmable in three or more axes, which may be either fixed in place or mobile for use in industrial automation applications." They are a cornerstone technology of modern automation, designed to perform tasks with high speed, precision, and endurance, often in environments unsuitable or dangerous for human workers.

---

## History

The era of industrial robotics began with the **Unimate**, developed by George Devol and Joseph Engelberger in the late 1950s and first installed at a General Motors plant in 1961 to handle hot die-cast metal parts. Early robots like the Unimate were typically hydraulic and programmed by teaching specific joint coordinates. The automotive industry was an early adopter, using robots for tasks like spot welding by the late 1960s. Key developments include:
* **1962:** The cylindrical **Versatran** robot by AMF.
* **1969:** The first commercial painting robot by **Trallfa** (Norway). Kawasaki produces the Unimate under license in Japan.
* **1973:** KUKA develops the **Famulus**, the first robot with six electromechanically driven axes.
* **1974:** ASEA (now ABB) introduces the all-electric, microprocessor-controlled **IRB-6**. Cincinnati Milacron introduces the minicomputer-controlled **T3**.
* **1978:** The **SCARA** (Selective Compliance Assembly Robot Arm) configuration is developed in Japan, revolutionizing electronics assembly.
* **1979:** Unimation introduces the **PUMA** (Programmable Universal Machine for Assembly), which became a standard in research labs.
* **1980s onward:** Development of parallel robots like the **Delta** robot for high-speed tasks. Increasing use of computer control, offline programming, and sensors.
* **2010s onward:** Rise of [[Collaborative_Robots|Collaborative Robots (Cobots)]].

---

## Common Types

Industrial arms are often classified by their [[Manipulator_Arm_Types|kinematic configuration]], primarily determined by the first three joints:

* **Articulated Robots:** The most common type, resembling a human arm (often 6R configuration like PUMA, ABB IRB series, KUKA robots). They offer high dexterity and a large workspace relative to their footprint. Typically 6 DoF.
    ![Articulated Robot Welding](https://blog.perfectwelding.fronius.com/wp-content/uploads/2021/05/PW_WPIC_WA_HTW_Tauro-1536x864.jpg)
    *(Example: Articulated industrial arms welding car bodies)*
* **SCARA Robots:** Typically 4 DoF (RRP or RRP structure with parallel vertical axes). Designed for high speed and stiffness in planar tasks, common in electronics assembly.
    ![SCARA Robot Assembly](https://upload.wikimedia.org/wikipedia/commons/0/09/SCARA_robot_2R.png)
    *(Example: SCARA robot performing assembly)*
* **Cartesian Robots (Gantry Robots):** Use three orthogonal prismatic (linear) axes (PPP). Offer simple kinematics, high stiffness, and potentially very large workspaces, but require significant overhead structure.
* **Delta Robots:** Parallel kinematic structure, characterized by very high speed and acceleration, primarily used for pick-and-place operations in packaging and assembly.
    ![Delta Robot Pick and Place](https://igus.widen.net/content/9pen18wjaw/jpeg/delta_lca_uk.jpg?crop=no&color=ffffff00&w=740&quality=80)
    *(Example: Delta robot in a packaging application)*
* **Cylindrical and Spherical Robots:** Early configurations, less common today.

---

## Key Characteristics

Industrial robots are specified by several key performance metrics:

* **Payload:** The maximum mass the robot can manipulate at its end-effector (including the weight of the end-effector itself).
* **Reach:** The maximum extent of the robot's workspace, usually measured horizontally from the base.
* **[[Degrees_of_Freedom]] (DoF):** The number of independent axes of motion (typically 4 to 7 for industrial arms). 6 DoF are needed for arbitrary spatial positioning and orientation.
* **Repeatability:** The ability of the robot to return precisely to the same taught point multiple times. This is often more critical than absolute accuracy in industrial tasks and can be very high (e.g., fractions of a millimeter).
* **Accuracy:** The ability of the robot to reach a specific commanded pose in its workspace based on its kinematic model. Often lower than repeatability unless calibrated.
* **Speed:** Maximum achievable velocity of the end-effector (TCP - Tool Center Point) and individual joints. Cycle time for specific tasks (including acceleration/deceleration) is often a more practical measure.
* **Work Envelope / [[Workspace]]:** The 3D volume that the robot's end-effector can access.

---

## Typical Applications

Industrial arms automate a wide range of tasks in manufacturing and logistics:

* **Welding:** Spot welding (automotive bodies), arc welding.
* **Painting & Coating:** Applying paint, sealants, or other coatings.
* **Material Handling:** Loading/unloading machines, transferring parts between stations.
* **Assembly:** Joining components, particularly in electronics and automotive industries.
* **Pick and Place:** Transferring items from one location to another, often at high speed (e.g., packaging).
* **Palletizing:** Stacking boxes or products onto pallets.
* **Machine Tending:** Loading/unloading CNC machines, injection molding machines, etc.
* **Processing:** Grinding, deburring, polishing, cutting, dispensing.
* **Inspection:** Using vision or other sensors for quality control.

---

## Programming and Control

* **Teach Pendants:** Handheld devices used for manually jogging the robot to desired positions and recording points for simple path programming (teaching).
* **Offline Programming (OLP):** Using simulation software (often integrated with CAD data) to create and test robot programs without using the physical robot. This minimizes production downtime.
* **Robot Languages:** Textual programming languages specific to robot manufacturers (e.g., KUKA KRL, ABB RAPID, Fanuc TP) allowing for more complex logic, sensor integration, and communication.
* **Controllers:** Typically proprietary, real-time controllers manage [[Motion_Control]], execute [[Trajectory_Planning]], interpret programs, and interface with sensors and external systems. Modern controllers often use PC-based hardware.

---

## Safety and Collaboration

* **Traditional Safety:** Due to their speed and power, traditional industrial robots pose significant hazards. They typically operate within fenced work cells equipped with safety interlocks (light curtains, pressure mats, door switches) to prevent human entry during operation (governed by standards like ISO 10218).
* **[[Collaborative_Robots|Collaborative Robots (Cobots)]]:** A rapidly growing category of industrial arms designed specifically for safe operation alongside or in direct collaboration with human workers, often without traditional safety fencing. Cobots typically feature lower speeds, power/force limiting capabilities, rounded designs, and integrated sensors (like force/torque sensors) to detect contact and react safely. They are enabling automation in tasks previously difficult to automate due to the need for human dexterity or judgment, especially in small and medium-sized enterprises (SMEs).

    ![Collaborative Robot Arm](https://spectrum.ieee.org/media-library/yumi.jpg?id=25579667&width=2400&height=1529)
    *(Example: Collaborative robot (cobot) working near a human)*

Industrial arms are essential tools in modern manufacturing, driven by demands for productivity, quality, consistency, and flexibility. The integration of advanced [[Sensor|sensors]], [[AI_and_Robot_Control|AI]], and collaborative capabilities continues to expand their application range.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #robotics OR #automation WHERE contains(file.outlinks, [[Robots]])