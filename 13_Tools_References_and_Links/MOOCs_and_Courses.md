---
title: MOOCs and Courses
description: A curated reference of Massive Open Online Courses, university open courseware, YouTube channels, textbooks, and learning paths for robotics at all levels — from beginner to graduate research.
tags:
  - courses
  - MOOC
  - learning
  - resources
  - references
  - tutorials
  - online-learning
  - education
  - textbook
  - YouTube
layout: default
category: robotics
author: Jordan_Smith_&_Claude
date: 2025-05-02
permalink: /moocs_and_courses/
related:
  - "[[Resources_Index]]"
  - "[[Robot_Operating_System_(ROS)]]"
  - "[[ROS_2_Overview]]"
  - "[[Machine_Learning]]"
  - "[[Control_Theory]]"
  - "[[SLAM_with_ROS]]"
  - "[[Nav2_Navigation]]"
  - "[[MoveIt2]]"
  - "[[Reinforcement_Learning_(RL)]]"
---

# MOOCs and Courses for Robotics

A curated guide to learning robotics across all skill levels — from first principles through graduate-level research. Resources are organized by format and topic to help you build a structured learning path.

See also: [[Resources_Index]]

---

## Learning Paths by Level

| Level | Start Here |
|---|---|
| **Complete Beginner** | Arduino/Python basics → ETH Zürich AMR → ROS 2 Tutorials |
| **CS Background** | Probabilistic Robotics (Thrun) → Modern Robotics → ROS 2 + Nav2 |
| **ME/EE Background** | Modern Robotics (Lynch) → Control Theory (Åström) → ROS 2 + MoveIt 2 |
| **ML Researcher** | Spinning Up → Gymnasium → Isaac Lab → robot learning literature |
| **Research Level** | Pieter Abbeel's CS287 → RSS/ICRA paper reading → personal project |

---

## MOOC Platforms

### Coursera

* **Robotics Specialization** — University of Pennsylvania (Vijay Kumar)
  Covers aerial robotics, kinematics, perception, estimation, motion planning, and Capstone. One of the most comprehensive online robotics programs available.
  *6 courses · Beginner–Intermediate*

* **Modern Robotics** — Northwestern University (Kevin Lynch & Frank Park)
  Based on the landmark textbook. Covers screw theory, kinematics, dynamics, trajectory generation, motion planning, and robot control with Python/MATLAB exercises.
  *6 courses · Intermediate–Advanced*

* **Self-Driving Cars Specialization** — University of Toronto
  State estimation, visual perception, and motion planning for autonomous vehicles. Strong perception and sensor fusion content.
  *4 courses · Intermediate*

* **Deep Learning Specialization** — Andrew Ng (deeplearning.ai)
  Foundational deep learning — neural networks, CNNs, RNNs, structuring ML projects. Essential prerequisite for robot learning work.
  *5 courses · Beginner–Intermediate*

* **Machine Learning Specialization** — Andrew Ng (deeplearning.ai / Stanford)
  Updated version of the classic ML course. Covers supervised/unsupervised learning, recommender systems, RL intro.
  *3 courses · Beginner*

### edX

* **Robotics MicroMasters** — Columbia University
  Animation and CGI motion, kinematics, dynamics, and robot mechanics. Graduate-level depth.

* **Autonomous Mobile Robots** — ETH Zürich (Roland Siegwart)
  Based on the classic AMR textbook. Covers locomotion, sensors, perception, state estimation, localization, mapping, SLAM, and navigation.
  *Self-paced · Intermediate–Advanced*

* **Underactuated Robotics** — MIT (Russ Tedrake)
  Covers nonlinear dynamics, trajectory optimization, LQR, policy search, locomotion. One of the most rigorous robotics courses available online. Companion to the Drake simulator.
  *Graduate level*

### Udacity

* **Robotics Software Engineer Nanodegree**
  Practical program covering ROS, C++, localization (EKF/AMCL), SLAM, path planning, and a capstone project in Gazebo.

* **Self-Driving Car Engineer Nanodegree**
  Sensor fusion (Kalman filters), computer vision, localization, path planning, control. Uses real datasets.

### Udemy

Quality varies. Look for courses by verified instructors with recent reviews:

* **ROS 2 for Beginners** (Edouard Renard) — Highly rated introduction to ROS 2 concepts and tools.
* **ROS2 Nav2** — Navigation stack practical implementation.
* **Robotics with Python Raspberry Pi and GoPiGo** — Hardware-focused introduction.

---

## University Open Courseware

### MIT OpenCourseWare (ocw.mit.edu)

* **6.832 Underactuated Robotics** (Russ Tedrake) — Trajectory optimization, locomotion, manipulation. [Freely available with lecture notes and problem sets.](https://underactuated.mit.edu/)
* **6.832 / 6.4212 Robotic Manipulation** (Russ Tedrake) — Modern manipulation with Drake. [roboticmanipulation.org](https://manipulation.csail.mit.edu/)
* **6.S094 Deep Learning for Autonomous Vehicles** — Deep learning for self-driving systems.
* **16.410 Principles of Autonomy and Decision Making** — Covers planning, search, MDPs, constraint satisfaction.

### ETH Zürich

* **Autonomous Mobile Robots** (Roland Siegwart, Juan Nieto) — Comprehensive mobile robotics. Lecture materials publicly available.
* **Programming for Robotics — ROS** — Intensive week-long ROS 2 course with full lecture videos on YouTube. One of the best free ROS resources available.
  [YouTube playlist: Programming for Robotics (ROS)](https://www.youtube.com/playlist?list=PLE-BQwvVGf8HOvwXPgtDfWoxd4Cc6ghiP)

### Stanford

* **CS223A Introduction to Robotics** (Oussama Khatib) — Classic robotics course covering kinematics, dynamics, control. Lecture videos freely available on YouTube.
* **CS231n Convolutional Neural Networks for Visual Recognition** — The gold standard computer vision course. Essential for perception in robotics.

### Carnegie Mellon University (CMU)

* **16-385 Computer Vision** — Comprehensive CV course with lecture notes.
* **16-831 Statistical Techniques in Robotics** — State estimation, probabilistic methods, Gaussian processes.
* **10-703 Deep Reinforcement Learning** (Katerina Fragkiadaki, Ruslan Salakhutdinov) — Graduate RL course with robotics applications.

### UC Berkeley

* **CS287 Advanced Robotics** (Pieter Abbeel) — Covers MDPs, POMDPs, LQR, trajectory optimization, imitation learning, RL for manipulation. Lecture videos on YouTube.
* **CS285 Deep Reinforcement Learning** (Sergey Levine) — The definitive graduate deep RL course. Homework involves robotics environments. [rail.eecs.berkeley.edu/deeprlcourse](https://rail.eecs.berkeley.edu/deeprlcourse/)

---

## YouTube Channels

| Channel | Focus | Level |
|---|---|---|
| **Articulated Robotics** | ROS 2 — practical, beginner-friendly tutorials on Nav2, URDF, sensors | Beginner–Intermediate |
| **Robotics Back-End** | ROS 2 programming, Python nodes, services, actions | Beginner–Intermediate |
| **The Construct** | ROS 1/2 tutorials, Gazebo, navigation | Beginner–Intermediate |
| **Russ Tedrake (MIT)** | Underactuated robotics, trajectory optimization, manipulation | Advanced |
| **Pieter Abbeel** | Robot learning, RL, imitation learning (UC Berkeley CS287) | Advanced |
| **Sergey Levine** | Deep RL, robot learning (CS285 lectures) | Advanced |
| **Cyrill Stachniss** | SLAM, sensor fusion, photogrammetry (University of Bonn) | Intermediate–Advanced |
| **3Blue1Brown** | Linear algebra, calculus, neural networks — math visualization | Foundational |
| **Two Minute Papers** | Recent AI/robotics research summaries | All levels |

---

## Textbooks

### Foundational Robotics

* **Modern Robotics: Mechanics, Planning, and Control** — Lynch & Park (2017)
  The modern standard textbook. Screw theory approach to kinematics, dynamics, motion planning, and control. Free PDF available at modernrobotics.org. Companion Coursera course.

* **Probabilistic Robotics** — Thrun, Burgard & Fox (2005)
  The definitive text for state estimation, SLAM, Kalman filters, particle filters, and Bayesian robot perception. Essential reading.

* **Introduction to Autonomous Mobile Robots** — Siegwart, Nourbakhsh & Scaramuzza (2011)
  Comprehensive treatment of mobile robotics: locomotion, sensors, perception, localization, mapping, and navigation. Companion to ETH Zürich AMR course.

* **Robotics: Modelling, Planning and Control** — Siciliano, Sciavicco, Villani & Oriolo (2009)
  Rigorous treatment of kinematics, dynamics, trajectory planning, and motion control for robot manipulators.

### Control Theory

* **Feedback Systems: An Introduction for Scientists and Engineers** — Åström & Murray (2021)
  Free PDF at authors' website. Modern control theory accessible to non-engineers. Covers PID, frequency domain, state-space, nonlinear systems.

* **A Mathematical Introduction to Robotic Manipulation** — Murray, Li & Sastry (1994)
  Graduate-level; Lie group treatment of robot kinematics and dynamics. Free PDF available.

### Robot Learning

* **Reinforcement Learning: An Introduction** — Sutton & Barto (2018)
  The definitive RL textbook. Free PDF at incompleteideas.net. Covers MDPs, dynamic programming, TD learning, policy gradient.

* **Deep Learning** — Goodfellow, Bengio & Courville (2016)
  Comprehensive deep learning reference. Free HTML version at deeplearningbook.org.

### Perception

* **Multiple View Geometry in Computer Vision** — Hartley & Zisserman (2004)
  The definitive text for 3D computer vision, camera models, epipolar geometry, structure from motion.

* **Computer Vision: Algorithms and Applications** — Szeliski (2022)
  Broad coverage of CV. 2nd edition freely available at szeliski.org.

---

## Specific ROS 2 Resources

* **ROS 2 Documentation (docs.ros.org)** — Official tutorials, concept explanations, API references. Start with the "Beginner: CLI Tools" and "Beginner: Client Libraries" tutorial series.
* **Nav2 Documentation (navigation.ros.org)** — Complete Nav2 reference including configuration guides, plugin API, and tutorials.
* **MoveIt 2 Documentation (moveit.picknik.ai)** — Tutorials from beginner to advanced manipulation.
* **SLAM Toolbox Documentation** — Configuration and integration guides for mapping and localization.
* **Foxglove Docs (docs.foxglove.dev)** — Visualization and bag analysis workflows.

---

## Interactive Learning Platforms

* **The Construct (theconstructsim.com)** — Web-based ROS development environment with integrated Gazebo simulation. No local install required. Offers structured courses from beginner to advanced.
* **Gymnasium (gymnasium.farama.org)** — Standard RL environment library (successor to OpenAI Gym). Includes robotics environments (MuJoCo-based) for learning RL algorithms.
* **Hugging Face LeRobot** — Open-source robot learning library with pretrained models, datasets, and simulation environments. Growing ecosystem for imitation and reinforcement learning on real hardware.

---

## Conferences to Follow

Staying current with research requires following the major venues:

| Conference | Focus | Frequency |
|---|---|---|
| **ICRA** (IEEE Int'l Conf on Robotics and Automation) | Broad robotics | Annual (May) |
| **IROS** (Intelligent Robots and Systems) | Broad robotics | Annual (Oct) |
| **RSS** (Robotics: Science and Systems) | High-impact research | Annual (Jul) |
| **CoRL** (Conference on Robot Learning) | Learning-based robotics | Annual (Nov) |
| **CVPR / ICCV / ECCV** | Computer vision (robotics perception) | Annual |
| **NeurIPS / ICML / ICLR** | Machine learning (robot learning methods) | Annual |

All papers from major robotics conferences are available free on arXiv or the conference websites.

---

## Dataview Plugin Features

```dataview
LIST FROM #courses OR #MOOC WHERE contains(file.outlinks, [[MOOCs_and_Courses]])
```

```dataview
TABLE title, description FROM #resources WHERE contains(file.tags, "learning")
```
