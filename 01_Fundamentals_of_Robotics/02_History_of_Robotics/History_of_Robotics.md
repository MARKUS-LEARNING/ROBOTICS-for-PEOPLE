---
title: History of Robotics
description: A historical overview of the field of robotics, from early concepts and industrial beginnings to modern advancements driven by AI and computing, including the Uncanny Valley concept.
tags:
  - history
  - robotics-history
  - industrial-robot
  - AI
  - timeline
  - automation
  - uncanny-valley
  - HRI
layout: default
category: robotics
author: Jordan_Smith
date: 2025-04-28
permalink: /history_of_robotics/
related:
  - "[[Key_Figures_and_Labs]]"
  - "[[Future_Trends_2025-2035]]"
  - "[[Robotics]]"
  - "[[Robot_Ethics_and_Policy_Debates]]"
---

# History of Robotics

The field of robotics, the science and technology of [[Robots|robots]], has roots stretching back to ancient concepts of [[Automaton|automata]] but emerged as a distinct engineering discipline in the mid-20th century. Its development has been driven by advances in mechanics, electronics, control theory, computer science, and [[Artificial Intelligence (AI)|artificial intelligence]].

## Conceptual Origins and Early Terms

* **Automata:** The ambition to create artificial devices mimicking life dates back millennia, seen in myths (e.g., Talus) and early mechanical creations like water clocks, Hero of Alexandria's devices, Al-Jazari's machines, Leonardo da Vinci's designs, and 18th-century European and Japanese automata.
* **"Robot":** The word itself originated from the Czech "robota" (meaning forced labor or serfdom) and was popularized by Czech playwright Karel Čapek in his 1920 play R.U.R. (Rossum's Universal Robots).
* **"Robotics" & Three Laws:** Science fiction author [[Isaac Asimov]] coined the term "Robotics" in 1942 and famously formulated his "Three Laws of Robotics" in his stories, providing an early conceptual framework for robot ethics and behavior.

## Precursors (1940s–1950s)

* **Teleoperators:** Raymond Goertz at Argonne National Laboratory developed master-slave manipulators in the late 1940s for handling radioactive materials behind shielding walls. These systems introduced the engineering concept of **bilateral force feedback** — the operator felt scaled-down forces from the remote environment. The mechanical advantage ratio between master and slave arms was typically 1:1 to 4:1, establishing principles of impedance scaling ($Z_{\text{displayed}} = \alpha \cdot Z_{\text{environment}}$) still used in surgical telerobotic systems today.
* **Numerical Control (NC):** MIT's Servomechanisms Laboratory developed the first NC milling machine (1952) under Air Force contract. The key innovation was representing tool paths as sequences of coordinate points on punched tape — the first instance of programmable motion control. The interpolation between waypoints used linear and circular arc segments, a technique that directly carried into robot trajectory generation.
* **Feedback Control Theory:** During this period, the mathematical foundations for robot control were laid. Bode's gain-phase analysis (1945), Nyquist's stability criterion, and root locus methods (Evans, 1948) provided engineers with tools to design stable servo loops. The standard second-order transfer function for a motor-driven joint:

$$
G(s) = \frac{\omega_n^2}{s^2 + 2\zeta\omega_n s + \omega_n^2}
$$

where $\omega_n$ is the natural frequency and $\zeta$ is the damping ratio, became the workhorse model for sizing motors and tuning position controllers.

* **Devol's Patent:** In 1954, inventor [[George Devol]] patented a "Programmed Article Transfer" device (US Patent 2,988,237), laying the groundwork for the first industrial robot. The patent described a system using a magnetic drum memory to store and replay sequences of joint positions — effectively the first teach-and-playback architecture.

## The Industrial Robot Revolution (1960s–1970s)

This era saw the birth of practical, computer-controlled robots primarily for industrial automation.

* **[[Unimate]]:** [[George Devol]] and [[Joseph Engelberger]] founded Unimation, Inc. in 1956. Their [[Unimate]] robot, first installed at a General Motors plant in 1961, is widely considered the first industrial robot. It was a large, hydraulic arm controlled by instructions stored on a magnetic drum, used for handling hot metal parts in die-casting – a task considered one of the "3 D's" (Dull, Dirty, Dangerous) suitable for automation.
* **Early Industrial Robots:** Other early systems included the cylindrical **Versatran** by AMF (1962) and the first commercial painting robot by **Trallfa** (Norway, 1969). Kawasaki Heavy Industries began producing the Unimate under license for the Asian market in 1969.
* **First Generation Characteristics (ca. 1950-1967):** Point-to-point machines with no external sensing. Used pneumatic or hydraulic actuators with mechanical stops. Control was bang-bang: move to hard stop, clamp, release. Repeatability was $\pm 1\text{–}2$ mm — adequate for die-casting and spot welding but not assembly.
* **Key Research:**
    * **SRI's [[Shakey]]:** Developed between 1966-1972, Shakey was the first integrated [[Mobile_Robots|mobile robot]] combining [[Perception]] (vision, range finding, bump sensors), planning, and action, reasoning about its environment.
    * **[[Stanford Arm]]:** Designed by [[Victor Scheinman]] in 1969, this was an influential 6-DoF electric research arm that became a research standard.
* **Technological Shifts:**
    * **Electric Drives:** The shift from hydraulics to electric servo motors began — a pivotal engineering transition. Electric drives offered superior controllability via current-torque linearity ($\tau = K_t \cdot i$), eliminated hydraulic fluid leaks, and enabled the PID position loops still used today. KUKA Famulus (1973, 6 electric axes); ASEA IRB-6 (1974, all-electric with microprocessor control, repeatability $\pm 0.1$ mm).
    * **Computer Control:** Minicomputers like the DEC PDP-11 enabled computed trajectories rather than point-to-point motion. Cincinnati Milacron T3 (1974) was the first commercially available minicomputer-controlled robot.
    * **Early Sensing:** Basic structured-light vision systems appeared (Hitachi, 1973), enabling robots to adapt to part location rather than requiring perfect fixturing.
* **Community Formation:** Robot associations (JIRA 1971/73, RIA 1974) and conferences (ISIR 1970, RoManSy 1973) began.
* **Second Generation Characteristics (ca. 1968-1977):** Closed-loop servo control replaced bang-bang positioning — the robot could now track continuous trajectories, not just visit discrete points. Teach pendants allowed operators to record waypoints by manually guiding the arm (lead-through teaching). PLC and minicomputer control enabled conditional logic (if-then branching), making tasks like spot welding with seam tracking feasible. Repeatability improved to $\pm 0.1\text{–}0.5$ mm.

## Growth, Specialization, and AI Integration (1980s–Present)

* **Assembly & High Speed (1980s):**
    * **[[PUMA]]:** Programmable Universal Machine for Assembly (Unimation/GM, 1978/79) became the research and industrial standard. The PUMA 560 — a 6-DOF all-electric arm with 2.5 kg payload and $\pm 0.1$ mm repeatability — is one of the most-studied robots in history. Its [[Links_and_Joints_Definitions|DH parameters]] appear in nearly every robotics textbook. Peak joint speeds of ~300°/s enabled sub-second cycle times for electronics assembly.
    * **[[SCARA]]:** Selective Compliance Assembly Robot Arm (Makino, Yamanashi University, 1978) exploited a key insight: assembly tasks are mostly planar. By making joints 1 and 2 revolute about vertical axes, the SCARA is stiff in $z$ (supporting payload against gravity) but compliant in $x$-$y$ (accommodating part misalignment). This selective compliance, characterized by a directional stiffness ratio $K_z / K_{xy} > 100$, eliminated the need for precision fixturing in PCB insertion tasks.
    * **[[Delta Robot]]:** Reymond Clavel (EPFL, 1985) invented this parallel kinematic mechanism. Three lightweight arms connect the base to a moving platform via parallelogram linkages, constraining the platform to pure translation (3 DOF). The key engineering advantage: all motors are mounted on the fixed base, minimizing moving mass. This gives acceleration $> 10g$ and pick rates exceeding 300 picks/min — still unmatched by serial architectures. The inverse kinematics reduces to solving three independent loop-closure equations, each yielding a closed-form solution.
    * **Languages & OLP:** Dedicated robot programming languages (e.g., VAL) and Offline Programming emerged.
    * **Third Generation Characteristics (ca. 1978-1999):** Sensor integration transformed robots from blind actuators into perceptive systems. Force/torque sensors enabled compliant assembly (peg-in-hole with $\pm 0.05$ mm clearance). Vision systems provided part identification and pose estimation. Higher-level languages (VAL, AML, RAPID) abstracted motion commands from servo-level details. Cycle times dropped below 1 second for pick-and-place, and repeatability reached $\pm 0.02$ mm on premium systems.
* **Mobile, Service, and AI Era (1990s–Present):**
    * **Mobile & Field Robotics:** Research intensified in autonomous mobile navigation ([[SLAM]]), and applications expanded in space, underwater, and service domains.
    * **[[Humanoid_Robots|Humanoids]] & Social Robots:** Development advanced significantly (Honda ASIMO, Sony QRIO). As robots began to take on more human-like appearances, the importance of understanding human perception of robots grew. In 1970, roboticist [[Masahiro Mori]] proposed the **[[Uncanny Valley]]** hypothesis. This influential idea suggests that as a robot's appearance becomes more human-like, human affinity towards it increases up to a point, but then drops sharply into revulsion or unease if the robot looks *almost*, but not perfectly, human. Affinity increases again only if the resemblance becomes nearly indistinguishable. This concept remains a key consideration in the design of robots intended for [[Human-Robot Interaction (HRI)]].
    * **Consumer Robots:** iRobot's Roomba (2002) became the first mass-market robot, selling millions of units. Its navigation evolved from random bounce (stochastic coverage) to [[SLAM]]-based systematic coverage using a single camera and IMU — a practical demonstration of consumer-grade localization at a < $300 price point.
    * **[[Medical_Robotics]]:** Intuitive Surgical's da Vinci system (FDA cleared 2000) demonstrated the value of robotic precision in surgery. The system provides 7-DOF wristed instruments through 8 mm trocar ports, with tremor filtering ($> 6$ Hz) and 3:1 to 5:1 motion scaling. Over 12 million procedures performed as of 2024. The Mako system (Stryker) introduced haptic boundaries for orthopedic surgery — the robot constrains the surgeon's tool within a pre-planned cutting volume using real-time force feedback at 1 kHz.
    * **[[ROS (Robot Operating System)]]:** Willow Garage released ROS (~2007) as an open-source middleware framework built on a publish-subscribe architecture. It standardized message passing between nodes (sensor drivers, planners, controllers) and introduced tools like `rviz` (visualization), `tf` (coordinate frame management), and `rosbag` (data logging). [[ROS_2_Overview|ROS 2]] (2017+) addressed production requirements: real-time DDS transport, lifecycle node management, and deterministic execution — essential for safety-critical applications per IEC 61508.
    * **AI/ML Revolution:** [[Deep_Learning]] transformed perception — convolutional networks achieved superhuman object detection ($> 95\%$ mAP on COCO) and enabled vision-based grasping of novel objects. New control paradigms emerged: [[Reinforcement_Learning_for_Robots|RL]] for locomotion (Boston Dynamics Atlas, ANYmal), [[Imitation_Learning|imitation learning]] from human demonstrations, and sim-to-real transfer using domain randomization. Foundation models (RT-2, Octo) now map language instructions directly to robot actions.
    * **[[Collaborative_Robots|Cobots]]:** Universal Robots (founded 2005) pioneered force-limited collaborative robots. The UR5 and successors comply with ISO/TS 15066, limiting quasi-static contact forces to $< 150$ N and transient forces to $< 250$ N at any body contact point. This eliminated safety caging for applications under the power and force limiting (PFL) safety mode, making automation accessible to SMEs with typical deployment times of $< 1$ day.
    * **Humanoid Renaissance:** A surge in humanoid development targets general-purpose labor. Tesla Optimus, Figure AI (Figure 02), Agility Robotics (Digit), and Boston Dynamics (electric Atlas) are pursuing full-size humanoids with 30+ DOF, whole-body control, and AI-driven task execution. The engineering challenge is simultaneous bipedal balance (stabilization at $> 200$ Hz), dexterous manipulation, and real-time perception — requiring compute budgets of 50–200 TOPS on edge hardware.

### Engineering Evolution Summary

| Era | Control | Actuators | Sensing | Repeatability | Cycle Time |
|-----|---------|-----------|---------|---------------|------------|
| 1st Gen (1950s-67) | Bang-bang, drum memory | Hydraulic, pneumatic | None | $\pm 1\text{–}2$ mm | 5–15 s |
| 2nd Gen (1968-77) | Servo, PLC | Electric + hydraulic | Encoders only | $\pm 0.1\text{–}0.5$ mm | 2–5 s |
| 3rd Gen (1978-99) | Microprocessor, OLP | Electric servo | Vision, F/T | $\pm 0.02\text{–}0.1$ mm | 0.5–2 s |
| 4th Gen (2000+) | AI/ML, real-time OS | High-torque direct drive | Multi-modal fusion | $\pm 0.01\text{–}0.05$ mm | $< 0.5$ s |

The history of robotics shows a continuous evolution from simple programmed industrial machines to increasingly sophisticated, sensor-rich, adaptable, and intelligent systems — driven at each stage by concrete engineering breakthroughs in actuators, sensing, computation, and control theory.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #robotics OR #kinematics WHERE contains(file.outlinks, [[History_of_Robotics]])
```
