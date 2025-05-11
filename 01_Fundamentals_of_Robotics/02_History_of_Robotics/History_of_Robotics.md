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
author: Jordan_Smith_&_le_Chat
date: 2025-04-28
permalink: /history_of_robotics/
related:
  - "[[Robot]]"
  - "[[Automaton]]"
  - "[[Isaac Asimov]]"
  - "[[George Devol]]"
  - "[[Joseph Engelberger]]"
  - "[[Victor Scheinman]]"
  - "[[Industrial_Arms]]"
  - "[[Mobile_Robots]]"
  - "[[Shakey]]"
  - "[[Stanford Arm]]"
  - "[[Unimate]]"
  - "[[PUMA]]"
  - "[[SCARA]]"
  - "[[Delta Robot]]"
  - "[[ROS (Robot Operating System)]]"
  - "[[Artificial Intelligence (AI)]]"
  - "[[Machine Learning]]"
  - "[[Collaborative Robots]]"
  - "[[Humanoid_Robots]]"
  - "[[Uncanny Valley]]"
  - "[[Masahiro Mori]]"
  - "[[Robotics_History_and_Future]]"
---

# History of Robotics

The field of robotics, the science and technology of [[Robots|robots]], has roots stretching back to ancient concepts of [[Automaton|automata]] but emerged as a distinct engineering discipline in the mid-20th century. Its development has been driven by advances in mechanics, electronics, control theory, computer science, and [[Artificial Intelligence (AI)|artificial intelligence]].

## Conceptual Origins and Early Terms

* **Automata:** The ambition to create artificial devices mimicking life dates back millennia, seen in myths (e.g., Talus) and early mechanical creations like water clocks, Hero of Alexandria's devices, Al-Jazari's machines, Leonardo da Vinci's designs, and 18th-century European and Japanese automata.
* **"Robot":** The word itself originated from the Czech "robota" (meaning forced labor or serfdom) and was popularized by Czech playwright Karel Čapek in his 1920 play R.U.R. (Rossum's Universal Robots).
* **"Robotics" & Three Laws:** Science fiction author [[Isaac Asimov]] coined the term "Robotics" in 1942 and famously formulated his "Three Laws of Robotics" in his stories, providing an early conceptual framework for robot ethics and behavior.

## Precursors (1940s–1950s)

* **Teleoperators:** Remote manipulators, like those developed by Goertz in the late 1940s/early 1950s for handling radioactive materials, established principles of remote mechanical control.
* **Numerical Control (NC):** The development of NC machines provided foundations for programmed motion control.
* **Devol's Patent:** In 1954, inventor [[George Devol]] patented a "Programmed Article Transfer" device, laying the groundwork for the first industrial robot.

## The Industrial Robot Revolution (1960s–1970s)

This era saw the birth of practical, computer-controlled robots primarily for industrial automation.

* **[[Unimate]]:** [[George Devol]] and [[Joseph Engelberger]] founded Unimation, Inc. in 1956. Their [[Unimate]] robot, first installed at a General Motors plant in 1961, is widely considered the first industrial robot. It was a large, hydraulic arm controlled by instructions stored on a magnetic drum, used for handling hot metal parts in die-casting – a task considered one of the "3 D's" (Dull, Dirty, Dangerous) suitable for automation.
* **Early Industrial Robots:** Other early systems included the cylindrical **Versatran** by AMF (1962) and the first commercial painting robot by **Trallfa** (Norway, 1969). Kawasaki Heavy Industries began producing the Unimate under license for the Asian market in 1969.
* **First Generation Characteristics (ca. 1950-1967):** These early robots were essentially programmable machines with limited control, no external sensing, often using pneumatic or hydraulic actuators and mechanical stops or simple logic control.
* **Key Research:**
    * **SRI's [[Shakey]]:** Developed between 1966-1972, Shakey was the first integrated [[Mobile_Robots|mobile robot]] combining [[Perception]] (vision, range finding, bump sensors), planning, and action, reasoning about its environment.
    * **[[Stanford Arm]]:** Designed by [[Victor Scheinman]] in 1969, this was an influential 6-DoF electric research arm that became a research standard.
* **Technological Shifts:**
    * **Electric Drives:** The move from bulky hydraulics to electric drives began (KUKA Famulus 1973, 6 electric axes; ASEA IRB-6 1974, all-electric, microprocessor).
    * **Computer Control:** Minicomputers enabled more sophisticated control (Cincinnati Milacron T3, 1974).
    * **Early Sensing:** Basic vision systems started appearing (Hitachi, 1973).
* **Community Formation:** Robot associations (JIRA 1971/73, RIA 1974) and conferences (ISIR 1970, RoManSy 1973) began.
* **Second Generation Characteristics (ca. 1968-1977):** Servo control, teach pendants, PLC/minicomputer control enabled more complex tasks like spot welding.

## Growth, Specialization, and AI Integration (1980s–Present)

* **Assembly & High Speed (1980s):**
    * **[[PUMA]]:** Programmable Universal Machine for Assembly (Unimation/GM, 1978/79) became standard.
    * **[[SCARA]]:** Selective Compliance Assembly Robot Arm (Japan, 1978) optimized planar assembly.
    * **[[Delta Robot]]:** High-speed parallel manipulator invented by Clavel.
    * **Languages & OLP:** Dedicated robot programming languages (e.g., VAL) and Offline Programming emerged.
    * **Third Generation Characteristics (ca. 1978-1999):** Focused on sensor integration (vision, force), higher-level languages, and adaptive capabilities. Robotics became defined as the intelligent connection of perception and action.
* **Mobile, Service, and AI Era (1990s–Present):**
    * **Mobile & Field Robotics:** Research intensified in autonomous mobile navigation ([[SLAM]]), and applications expanded in space, underwater, and service domains.
    * **[[Humanoid_Robots|Humanoids]] & Social Robots:** Development advanced significantly (Honda ASIMO, Sony QRIO). As robots began to take on more human-like appearances, the importance of understanding human perception of robots grew. In 1970, roboticist [[Masahiro Mori]] proposed the **[[Uncanny Valley]]** hypothesis. This influential idea suggests that as a robot's appearance becomes more human-like, human affinity towards it increases up to a point, but then drops sharply into revulsion or unease if the robot looks *almost*, but not perfectly, human. Affinity increases again only if the resemblance becomes nearly indistinguishable. This concept remains a key consideration in the design of robots intended for [[Human-Robot Interaction (HRI)]].
    * **Consumer Robots:** First widespread successes (iRobot Roomba, 2002).
    * **[[Medical Robotics]]:** Increased use in surgery and assistance (e.g., da Vinci system).
    * **[[ROS (Robot Operating System)]]:** Emergence (~2007) of this open-source framework standardized software development, accelerating research. [[ROS_2_Overview|ROS 2]] later addressed limitations for production systems.
    * **AI/ML Revolution:** [[Deep Learning]] dramatically improved [[Perception]] (vision, speech) and enabled new [[Control Theory|control]] paradigms like [[Reinforcement Learning for Robots|RL]] and [[Imitation Learning]], allowing robots to handle more complex, unstructured tasks.
    * **[[Collaborative Robots|Cobots]]:** Emerged as a major trend, enabling safe HRI without traditional cages and opening automation to new areas.
    * **Humanoid Renaissance:** A recent surge in humanoid development (e.g., Tesla Optimus, Figure AI Figure 01) targets general-purpose applications, powered by AI advancements.

The history of robotics shows a continuous evolution from simple programmed industrial machines to increasingly sophisticated, sensor-rich, adaptable, and intelligent systems operating across diverse and complex domains.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #robotics OR #kinematics WHERE contains(file.outlinks, [[History_of_Robotics]])