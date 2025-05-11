---
title: Sensors
description: Sensors are devices that detect and respond to physical inputs from the environment, converting them into signals that can be measured or recorded.
tags:
  - robotics
  - measurement
  - electronics
  - control
  - automation
type: Device
application: Measurement and detection in robotic systems
layout: default
category: robotics
author: Jordan_Smith_&_Gemini
date: 2025-04-29
permalink: /sensors/
related:
  - "[[Robot_Design]]"
  - "[[Mechatronics]]"
  - "[[Control_Systems]]"
  - "[[Actuator]]"
  - "[[Feedback_Control]]"
  - "[[Manipulator_Arm]]"
  - "[[Legged_Robots]]"
  - "[[Wheeled_Mobile_Robots]]"
  - "[[Piezoelectricity]]"
  - "[[Nanotechnology]]"
---

# Sensors

**Sensors** are devices that detect and respond to physical inputs from the environment, converting them into signals that can be measured or recorded. They play a crucial role in robotics by providing the necessary data for control, navigation, and interaction with the environment. Sensors enable robots to perceive their surroundings, monitor internal states, and make informed decisions based on real-time data.

---
<img src="https://www.frontiersin.org/files/Articles/576846/fnbot-14-576846-HTML/image_m/fnbot-14-576846-g001.jpg"></img>
<font size=1>*Source: https://www.frontiersin.org/journals/neurorobotics/articles/10.3389/fnbot.2020.576846/full*
---

## Types of Sensors

1. **Temperature Sensors**: Measure temperature changes in the environment or within a system.
   - Examples: Thermocouples, RTDs (Resistance Temperature Detectors), and thermistors.

2. **Pressure Sensors**: Detect changes in pressure, often used in fluid and gas systems.
   - Examples: Piezoresistive sensors, capacitive sensors, and strain gauges.

3. **Proximity Sensors**: Detect the presence or absence of objects within a certain range.
   - Examples: Ultrasonic sensors, infrared sensors, and capacitive sensors.

4. **Accelerometers**: Measure acceleration forces, used for motion detection and vibration analysis.
   - Examples: Piezoelectric accelerometers, MEMS (Microelectromechanical Systems) accelerometers.

5. **Gyroscopes**: Measure rotational motion and orientation.
   - Examples: MEMS gyroscopes, fiber optic gyroscopes.

6. **Optical Sensors**: Detect light or other electromagnetic radiation.
   - Examples: Photodiodes, phototransistors, and image sensors.

7. **Force/Torque Sensors**: Measure forces and torques applied to a system.
   - Examples: Strain gauges, load cells, and piezoelectric sensors.

---

## Key Equations

- **Temperature Measurement (Thermocouple)**:
  $$
  V = \alpha \cdot (T_1 - T_2)
  $$
  where $V$ is the voltage output, $\alpha$ is the Seebeck coefficient, $T_1$ is the measured temperature, and $T_2$ is the reference temperature.
  <br></br>

- **Pressure Measurement**:
  $$
  P = \frac{F}{A}
  $$
  where $P$ is the pressure, $F$ is the force applied, and $A$ is the area over which the force is applied.
  <br></br>

- **Acceleration Measurement**:
  $$
  a = \frac{\Delta v}{\Delta t}
  $$
  where $a$ is the acceleration, $\Delta v$ is the change in velocity, and $\Delta t$ is the change in time.
  <br></br>

- **Angular Velocity (Gyroscope)**:
  $$
  \omega = \frac{\Delta \theta}{\Delta t}
  $$
  where $\omega$ is the angular velocity, $\Delta \theta$ is the change in angle, and $\Delta t$ is the change in time.

---

## Impact on Robotics

- **Environmental Perception**: Sensors enable robots to perceive and interact with their environment, crucial for navigation, obstacle avoidance, and object manipulation.
- **Feedback Control**: Sensors provide real-time data for feedback control systems, allowing robots to adjust their actions dynamically.
- **Design and Integration**: The selection and integration of sensors are important aspects of [[Robot_Design]] and [[Mechatronics]], influencing the performance and reliability of robotic systems.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts
```dataview
LIST FROM #robotics OR #robot-design  WHERE contains(file.outlinks, [[Sensors]])
