---
title: Robots
description: "Defines 'Robot' and outlines its core characteristics, etymology, and basic types within the field of robotics."
tags:
  - glossary-term
  - robot
  - definition
  - robotics
  - core-concept
  - automation
  - machine
layout: default
category: robotics
author: Jordan_Smith_and_le_Chat
date: 2025-05-02
permalink: /robot/
related:
  - "[[Robotics]]"
  - "[[History_of_Robotics]]"
  - "[[Sensors]]"
  - "[[Perception]]"
  - "[[AI_and_Robot_Control]]"
  - "[[Actuator]]"
  - "[[Automation]]"
  - "[[Autonomy]]"
  - "[[Robot_Types_and_Applications]]"
  - "[[Glossary]]"
---

# Robot

A **Robot** is generally defined as a machine—especially one programmable by a computer—capable of carrying out a complex series of actions automatically. While the exact definition can be debated and evolves with technology, robots are typically characterized by their ability to interact with the physical world through a cycle of sensing, computation, and action. [[Robotics]] is the science and technology dealing with robots.

---
<img src="https://cdn.prod.website-files.com/679c26bf7f96c23ec67d2854/679c2855a08dd9ef1ec51a43_FH%20with%20Weeders%20(1).jpg"></img>
<font size=1>*source: https://www.redbarnrobotics.com/*</font>
---

---
<img src="https://novoscriptorium.files.wordpress.com/2018/04/list-7-early-automatons-archytas-51243621-e.jpeg?w=686&h=385&crop=1"></img>
<font size=3>The history of robots traces back to 350 B.C.E., as noted by Smithsonian magazine, with the mathematician Archytas of Tarentum creating a mechanical wooden dove. This early invention, powered by compressed air or steam, could fly up to 200 meters and is often regarded as the first robot and possibly the first drone, marking a significant milestone in autonomous flight.</font>
<font size=1>*source: https://novoscriptorium.com/2018/04/20/archytas-of-tarentum-the-father-of-robotics/*</font>

---

## Etymology

The term "robot" was first popularized by the Czech playwright Karel Čapek in his 1920 play *R.U.R. (Rossum's Universal Robots)*. It derives from the Czech word "robota," meaning "forced labor" or "serfdom," reflecting the initial concept of artificial workers. The term [[Robotics]] itself was coined later by science fiction author Isaac Asimov. (See [[History_of_Robotics]]).

---

## Key Characteristics

Modern robots typically exhibit several core characteristics, often conceptualized as a "Sense-Plan-Act" loop:

1. **[[Sensors|Sensing]] & [[Perception]]**: Robots use [[Sensors|sensors]] (like [[Camera_Systems|cameras]], [[LIDAR]], [[IMU_Sensors|IMUs]], force sensors) to gather data about their internal state and external environment. [[Perception]] involves interpreting this raw sensor data into meaningful information.
   <br>

2. **Computation / [[AI_and_Robot_Control|Control]] / Planning**: A processing unit (computer, [[Microcontroller]]) executes programs, performs calculations, and makes decisions. This can range from simple predefined sequences to complex [[Artificial_Intelligence]] algorithms for [[Path_Planning]], [[Learning|learning]], and reasoning.
   <br>

3. **[[Actuator|Action]] / [[Manipulation]] / [[Locomotion]]**: Robots interact physically with the world using [[Actuator|actuators]] (motors, pistons) that drive [[Mechanisms_and_Actuation|mechanisms]] like wheels, legs, joints, or grippers. This enables [[Locomotion]] (movement) and [[Manipulation]] (handling objects).
   <br>

4. **Programmability**: Robots can typically be programmed to perform different tasks or sequences of actions, distinguishing them from fixed-purpose machines.
   <br>

5. **[[Autonomy]]**: Robots operate with varying degrees of [[Autonomy]], from remotely controlled teleoperation to fully autonomous decision-making based on sensor input and internal goals.
   <br>

---

## Distinction from Automata

While related, robots are generally distinguished from simpler [[Automaton|automata]] (like those historically built) by their ability to sense their environment, process information, and adapt their actions accordingly, rather than just executing pre-programmed, repetitive movements.

---

## Types of Robots

The field encompasses a vast range of machines. Major categories include:

* **[[Industrial_Arms]] (Fixed-base manipulators)**: Used in manufacturing for tasks like welding, assembly, and material handling.
  <br>

* **[[Mobile_Robots]]**:
  - **Wheeled Robots**: Use wheels for locomotion, often employed in indoor environments.
  - **[[Legged_Robots|Legged Robots]]**: Mimic animal or human leg motion for navigating uneven terrains.
  - **Tracked Robots**: Utilize continuous tracks for better traction on rough surfaces.
  - **[[Drones|Flying Robots]]**: Operate in aerial environments for surveillance, delivery, or exploration.
  - **[[Underwater_and_Space_Robots|Underwater Robots]]**: Designed for subaquatic exploration and tasks.
  <br>

* **[[Humanoid_Robots]]**: Robots designed to resemble the human body, capable of performing tasks in environments designed for humans.
  <br>

* **[[Collaborative_Robots]] (Cobots)**: Designed to work alongside humans in shared workspaces, often with safety features to prevent harm.
  <br>

* **[[Service_Robots]]**: Assist humans in various service roles, such as healthcare, hospitality, and education.
  <br>

* **[[Swarm_Robots]]**: Groups of robots working together to accomplish tasks collectively, often inspired by biological swarm behavior.
  <br>

* **[[Soft_Robotics|Soft Robots]]**: Utilize flexible materials and structures to interact safely with humans and adapt to various environments.
  <br>

The definition and capabilities of robots continue to evolve rapidly with advances in [[AI_and_Robot_Control|AI]], [[Sensors|sensing]], [[Actuator|actuation]], materials science, and [[Robotics Software|software]].

---

## Mathematical Representations

### Kinematics

The kinematics of a robot describes the motion of its components without considering the forces involved. For a robotic arm with $n$ joints, the position and orientation of the end-effector can be described using transformation matrices:

$$
T = T_1 \cdot T_2 \cdot \ldots \cdot T_n
$$

where $T_i$ is the transformation matrix for the $i$-th joint.

<br>

### Dynamics

The dynamics of a robot involve the relationship between the motion of the robot and the forces causing it. The Euler-Lagrange equation is often used to describe the dynamics:

$$
\frac{d}{dt} \left( \frac{\partial L}{\partial \dot{q}} \right) - \frac{\partial L}{\partial q} = \tau
$$

where $L = T - V$ is the Lagrangian, $T$ is the kinetic energy, $V$ is the potential energy, $q$ is the generalized coordinate, and $\tau$ is the applied torque.

<br>

### Control Systems

Control systems use feedback mechanisms to ensure robots achieve desired states or behaviors. A common control algorithm is the Proportional-Integral-Derivative (PID) controller:

$$
u(t) = K_p e(t) + K_i \int e(t) \, dt + K_d \frac{de(t)}{dt}
$$

where $u(t)$ is the control input, $e(t)$ is the error between the desired and actual states, and $K_p$, $K_i$, and $K_d$ are the proportional, integral, and derivative gains, respectively.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #robotics OR #automation WHERE contains(file.outlinks, [[Robots]])
