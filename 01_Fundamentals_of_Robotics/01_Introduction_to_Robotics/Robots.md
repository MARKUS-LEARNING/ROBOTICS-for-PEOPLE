---
title: Robots
description: "Defines 'Robot' and outlines its core characteristics, etymology, and basic types within the field of robotics."
tags:
  - robot
  - definition
  - robotics
  - core-concept
  - automation
  - machine
layout: default
category: robotics
author: Jordan_Smith
date: 2025-05-02
permalink: /robot/
related:
  - "[[Robotics]]"
  - "[[Actuator]]"
  - "[[Sensors]]"
  - "[[Autonomous_Robots]]"
  - "[[Degrees_of_Freedom]]"
---

# Robot

A **Robot** is generally defined as a machine—especially one programmable by a computer—capable of carrying out a complex series of actions automatically. While the exact definition can be debated and evolves with technology, robots are typically characterized by their ability to interact with the physical world through a cycle of sensing, computation, and action. [[Robotics]] is the science and technology dealing with robots.

---
<img src="https://cdn.prod.website-files.com/679c26bf7f96c23ec67d2854/679c2855a08dd9ef1ec51a43_FH%20with%20Weeders%20(1).jpg"></img>
<font size=1>*source: https://www.redbarnrobotics.com/*</font>
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

## Engineering Fundamentals of a Robot System

Every robot, regardless of type, implements a **sense-plan-act** loop. Understanding the engineering constraints at each stage is essential for practitioners.

### The Sense-Plan-Act Cycle Time Budget

For a robot operating at a control frequency of $f$ Hz, the total cycle time budget is:

$$
t_{\text{cycle}} = \frac{1}{f} = t_{\text{sense}} + t_{\text{plan}} + t_{\text{act}}
$$

| Robot Type | Typical $f$ | $t_{\text{cycle}}$ | Bottleneck |
|------------|------------|---------------------|------------|
| Industrial arm (position control) | 1,000–8,000 Hz | 0.125–1 ms | Servo computation |
| Mobile robot (navigation) | 10–50 Hz | 20–100 ms | Perception pipeline |
| Humanoid (whole-body control) | 200–1,000 Hz | 1–5 ms | Dynamics solver |
| Surgical robot (force feedback) | 1,000–3,000 Hz | 0.3–1 ms | Haptic rendering |

### Degrees of Freedom and Workspace

A robot's [[Degrees_of_Freedom|degrees of freedom]] (DOF) determine what tasks it can perform. A rigid body in 3D space has 6 DOF — 3 translational $(x, y, z)$ and 3 rotational $(\phi, \theta, \psi)$. To position and orient an end-effector arbitrarily in 3D space, a manipulator needs at least 6 joints. Robots with $n > 6$ joints are **kinematically redundant**, offering extra flexibility for obstacle avoidance and singularity management.

The [[Grübler's_Formula|Grübler-Kutzbach formula]] computes the mobility of a mechanism:

$$
M = m(N - 1 - J) + \sum_{i=1}^{J} f_i
$$

where $m = 3$ (planar) or $m = 6$ (spatial), $N$ is the number of links (including ground), $J$ is the number of joints, and $f_i$ is the DOF of joint $i$.

### Payload, Speed, and Precision Trade-offs

Every robot design balances three competing requirements:

$$
\text{Payload} \propto \frac{\tau_{\text{max}}}{L_{\text{arm}}}, \quad \text{Speed} \propto \frac{\dot{\theta}_{\text{max}} \cdot L_{\text{arm}}}{1}, \quad \text{Precision} \propto \frac{1}{\text{compliance} + \text{backlash}}
$$

| Specification | Small Cobot (UR3e) | Industrial (FANUC M-20iA) | Heavy-Duty (FANUC M-900iB) |
|---------------|-------------------|--------------------------|---------------------------|
| Payload | 3 kg | 35 kg | 700 kg |
| Reach | 500 mm | 1,811 mm | 2,832 mm |
| Repeatability | $\pm 0.03$ mm | $\pm 0.04$ mm | $\pm 0.1$ mm |
| Weight | 11 kg | 250 kg | 2,850 kg |
| DOF | 6 | 6 | 6 |

### Power and Energy Considerations

For battery-powered mobile robots, operational endurance is governed by:

$$
t_{\text{run}} = \frac{E_{\text{battery}}}{\bar{P}_{\text{total}}} = \frac{E_{\text{battery}}}{P_{\text{motors}} + P_{\text{compute}} + P_{\text{sensors}} + P_{\text{comms}}}
$$

A typical warehouse AMR carries a 48V, 30Ah LiFePO4 battery ($E \approx 1.44$ kWh) and draws 200–500 W average, yielding 3–7 hours of operation. Regenerative braking during deceleration can recover 10–20% of drive energy.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #robotics OR #automation WHERE contains(file.outlinks, [[Robots]])
```
