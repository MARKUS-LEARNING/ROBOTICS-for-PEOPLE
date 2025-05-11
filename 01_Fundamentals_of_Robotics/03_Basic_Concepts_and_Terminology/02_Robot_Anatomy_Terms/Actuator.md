---
title: Actuator
description: "Defines Actuator: A fundamental component that converts stored energy or a control signal into physical motion or force, enabling robots (both manipulators and mobile robots) to move and interact with their environment."
tags:
  - glossary-term
  - component
  - motion
  - control
  - hardware
  - electrical
  - hydraulic
  - pneumatic
  - mechanism
  - mobile-robot
  - manipulator-arm
  - mechatronics
layout: default
category: robotics
author: Jordan_Smith_&_le_Chat
date: 2025-04-29
permalink: /actuator/
related:
  - "[[Degrees_of_Freedom]]"
  - "[[Kinematic_Chains]]"
  - "[[PID_Control]]"
  - "[[Torque_and_Force_Calculations]]"
  - "[[Mechanisms_and_Actuation]]"
  - "[[Robot_Hands]]"
  - "[[Exoskeletons]]"
  - "[[Nanorobots]]"
  - "[[Locomotion]]"
  - "[[Wheeled_Mobile_Robots]]"
  - "[[Legged_Robots]]"
  - "[[Control_Systems]]"
  - "[[Sensors]]"
  - "[[Feedback_Control]]"
  - "[[Electric_Motors]]"
  - "[[Transmission_Mechanisms]]"
  - "[[Robot_Design]]"
  - "[[Manipulator_Dynamics]]"
  - "[[Statics]]"
  - "[[Mechatronics]]"
  - "[[Soft_Robotics]]"
  - "[[Bio-inspired_Robotics]]"
  - "[[Gears]]"
  - "[[Backlash]]"
  - "[[Friction]]"
  - "[[Stiffness]]"
  - "[[Compliance]]"
  - "[[Power_Density]]"
  - "[[Human-Robot_Interaction]]"
---

# Actuator

An **actuator** is a fundamental component of a robot responsible for converting stored energy (typically electrical, hydraulic, or pneumatic) or a control signal into physical motion (linear or rotary) or force. Essentially, it's the "muscle" of the robot that allows it to move its joints, interact with its environment, or manipulate objects. Actuators supply the motive power for robots, enabling both manipulation tasks (like industrial arms) and [[Locomotion]] (like mobile robots). They form a critical link between the robot's [[Control_Systems]] and its physical actions in the world.

---

## Function

Actuators take commands from the robot's [[Control_Systems]] (often electrical signals) and translate them into physical actions like:

* Rotating a joint in a [[Manipulator_Arm|manipulator arm]] or [[Legged_Robots|legged robot]].
* Extending or retracting a limb or prismatic joint.
* Opening or closing a gripper or [[Robot_Hands|robot hand]].
* Driving the wheels or legs of a mobile robot.

The motion produced can be linear (moving in a straight line) or rotary (rotating around an axis), depending on the actuator type and [[Mechanisms_and_Actuation|mechanism design]]. This conversion process is fundamental to how robots perform tasks ranging from delicate assembly to heavy lifting. The performance and characteristics of actuators heavily influence the overall capabilities and limitations of a robotic system.

---

## Common Types

Various types of actuators are used in robotics, with selection depending heavily on the application's requirements (force, speed, precision, size, environment, cost). Key types include:

* **[[Hydraulic_Systems|Hydraulic Actuators]]**: Use pressurized fluid (usually oil) to generate very large forces and high [[Power_Density|power-to-weight ratios]]. Historically significant, they powered the earliest industrial robots like the Unimate and early legged robots like the Raibert hopper. Challenges include bulkiness of power supply (pumps), cost, potential fluid leaks, noise, and maintenance. Used in heavy industrial robots, large [[Legged_Robots|legged robots]], and some [[Exoskeletons]].
* **[[Pneumatic_Systems|Pneumatic Actuators]]**: Use compressed air. Typically provide fast, low-cost, on/off motion, common in simpler automation and grippers. Can be servo-controlled but are generally less precise and powerful than hydraulics. Also used in specific applications like milking robots and some [[Soft_Robotics|soft]]/[[Bio-inspired_Robotics|bio-inspired robots]] (e.g., McKibben artificial muscles).
* **[[Electric_Motors|Electromagnetic Actuators]]**: The most common type today. Convert electrical energy into motion. Link to `[[Electromagnetism]]`.
    * **[[DC_Motors|DC Motors]] (Permanent Magnet)**: Widely available, used with brush commutation. Used in early research robots like the Stanford Arm and often in hobby/educational robots for driving wheels or simple joints. Key characteristics include the torque constant ($k_t$) relating current to torque, and back-EMF ($k_t \omega$), which opposes the supply voltage. Ironless rotor types offer low [[Friction|friction]] but have thermal limits.
    * **[[Stepper_Motors|Stepper Motors]]**: Rotate in discrete steps, often used for open-loop position control in simpler, low-cost robots. Microstepping allows high resolution. Generally have lower power-to-weight ratios than servomotors.
    * **[[Servo_Motors]]**: Typically [[DC_Motors|DC motors]] or [[Brushless_DC_Motors|BLDC motors]] combined with a position sensor ([[Encoders|Encoder]] or [[Resolvers|Resolver]]) and controller, enabling precise [[Feedback_Control|closed-loop control]] of position, velocity, or torque. Very common in robotic arms and legs. Significant advances by companies like Honda and Sony have led to high power-to-weight, compliant servos for humanoid robots. Hobby servos are used in smaller robots like Genghis.
    * **[[Brushless_DC_Motors|Brushless DC Motors]] (AC Servomotors / BLDC)**: Widely used in modern industrial robots. Eliminate brush friction/wear, offering good performance, reliability, and heat dissipation. Controllers are more complex. Linear versions also exist.
    * **[[Direct_Drive_Robots|Direct Drive Motors]]**: High-torque, low-speed motors interfaced directly to links, eliminating transmission [[Backlash]] but often having poor inertia matching and increased distal mass/inertia. Requires large motors to generate sufficient torque.
* **[[Piezoelectricity|Piezoelectric Actuators]]**: Use materials that change shape when a voltage is applied, enabling very fine, precise movements (nanometer resolution) at high speeds. Generate high forces but over very small displacements (stroke). Used in STMs (Scanning Tunneling Microscopes) and [[Nanotechnology|nanomanipulation]]. Can also function as [[Sensors]]. Used in some [[Soft_Robotics|soft robots]].
* **[[Shape_Memory_Alloys|Shape Memory Alloys]] (SMAs)**: Materials that "remember" a shape and return to it when heated (often via electrical current), acting like artificial muscles. Generally have low efficiency and medium response speed. Used in some [[Soft_Robotics|soft robots]].
* **[[Electroactive_Polymers|Electroactive Polymers]] (EAPs)**: Polymers that change shape significantly under electrical stimulation, used as artificial muscles, particularly in [[Soft_Robotics|Soft Robotics]] and [[Bio-inspired_Robotics|Bio-inspired Robotics]].
* **Other Types:** Thermal, bimetallic, chemical, magnetostrictive, bladder, and [[MEMS]]-based actuators are also used, often in research or specialized applications. [[Soft_Robotics|Soft Robotics]] actively explores novel actuation principles (e.g., electrostatic). Research also explores [[Bio-inspired_Robotics|bio-inspired designs]] (e.g., skeletal muscle actuators) and [[Variable_Stiffness_Actuators|Variable Stiffness Actuators]] that can change their [[Compliance]].

---

## Actuation for Locomotion

Actuators are crucial for robot [[Locomotion]], enabling wheeled, legged, and other forms of movement.

* **[[Wheeled_Mobile_Robots|Wheeled Mobile Robots]]**: Typically use [[Electric_Motors]] ([[DC_Motors|DC]], [[Servo_Motors|servo]], [[Stepper_Motors|stepper]]) to drive wheels. Specialized wheels like mecanum or omniwheels allow for [[Omnidirectional_Robots|omnidirectional]] motion, whereas conventional wheels often lead to [[Nonholonomic_Systems|nonholonomic]] constraints.
* **[[Legged_Robots]]**: Require actuators at each joint (often 2-3 [[Degrees_of_Freedom|DoF]] per leg) capable of supporting robot weight and enabling dynamic [[Robot_Gaits|gaits]]. This often leads to higher mechanical complexity and power demands compared to wheeled robots. Efficiency varies; legged locomotion can be more efficient on soft ground, while wheels excel on flat, hard surfaces.

---

## Actuation Placement and Transmission

Actuators can be placed directly at the joint ([[Direct_Drive_Robots|direct drive]]) or remotely, transmitting power via [[Transmission_Mechanisms|mechanisms]].

* **[[Direct_Drive_Robots|Direct Drive]]**: Simplifies transmission and eliminates [[Backlash]], but increases distal mass/inertia and may have poor inertia matching.
* **[[Remote_Actuation|Remote Actuation]]**: Reduces moving inertia by placing heavy motors proximally. Requires transmission systems ([[Gears]] including [[Harmonic_Drive|Harmonic Drives]], RV drives; [[Cable-Driven_Robots|tendons/cables]]; belts/pulleys; linkages; [[Ball_Screw|ball screws]]) which introduce complexity, [[Backlash]], [[Friction]], reduced [[Stiffness]], and [[Compliance]].

---

## Key Considerations

Selecting actuators involves balancing numerous factors critical to the [[Robot_Design]] process:

* **Torque/Force:** Output capability required for the task.
* **Speed:** Maximum velocity/rotational speed needed.
* **Precision/Resolution:** Smallest motion increment required; influenced by [[Encoders|encoder]]/[[Resolvers|resolver]] resolution in servos.
* **Power Consumption & Efficiency:** Energy requirements and conversion effectiveness. Biological systems (muscles) often achieve higher efficiencies than artificial actuators. [[Power_Density]] (power per volume/mass) is critical for [[Nanorobots]].
* **Weight and Size:** Physical constraints impacting inertia and design. Power-to-weight ratio is important.
* **[[Stiffness]]**: Resistance to deformation under load. Both actuator and [[Transmission_Mechanisms|transmission stiffness]] are crucial.
* **[[Backlash]] & [[Friction]]**: Affect precision and control. [[Friction|Friction]] can be complex (static, viscous, Coulomb, Stribeck). [[Direct_Drive_Robots|Direct drives]] eliminate [[Backlash]].
* **Back-drivability:** Ability to be moved by external forces when unpowered; important for safety in [[Human-Robot_Interaction]] and for haptics. Worm gears are typically not back-drivable.
* **Control Interface:** How the actuator receives commands and sends feedback (e.g., PWM, CAN bus).
* **Reliability & Maintenance:** Durability and ease of service. Relevant for hydraulics; [[Brushless_DC_Motors|brushless motors]] offer good reliability.
* **Cost:** Varies significantly based on type and performance.
* **Noise:** Audible noise generated during operation.
* **Environmental Suitability:** Tolerance to temperature, dust, moisture, vacuum, etc.

Actuators are critical components defining a robot's physical interaction capabilities. They are essential links in the [[Kinematic_Chains]] and determine [[Degrees_of_Freedom]]. Their performance dictates the effectiveness of [[Control_Systems|control algorithms]] like [[PID_Control]] and directly impacts the [[Manipulator_Dynamics|robot's dynamics]]. The selection and integration of actuators are core aspects of [[Mechatronics]] and overall [[Robot_Design]].

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

- **List all related components and concepts**:
  ```dataview
  LIST FROM #component OR #mechatronics WHERE contains(file.outlinks, [[Actuator]])
  ```


```dataview
TABLE file.link AS "Note" FROM #RobotDesign
```
