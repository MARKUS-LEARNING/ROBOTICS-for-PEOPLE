---
title: Course Notes Collection
description: "A curated, checked list of free and open robotics courses as of 2026 (MIT Robotic Manipulation and Underactuated Robotics, Modern Robotics, ETH's ROS course, the Hugging Face robotics course, QUT Robot Academy, Stanford CS231n and more), how each maps to this textbook, and a template for taking course notes in this vault so they link into the knowledge graph."
aliases:
  - Course Notes
  - Robotics Courses
tags:
  - education
  - courses
type: reference
level: beginner
subfield:
  - creative-education
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Labs_Projects_and_Tutorials]]"
related:
  - "[[MOOCs_and_Courses]]"
  - "[[Robotics_Lab_Template]]"
  - "[[Research_Papers_Index]]"
key_takeaways:
  - "Excellent robotics courses are free online in 2026, from classical mechanics and control to robot learning."
  - "Pair each course with this textbook's notes: take notes in the vault and link concepts so they show up in the graph and on learning paths."
  - "Courses age; check that software versions in exercises (ROS distribution, simulator) match what you install."
sources:
  - "Tedrake, R. (2026). Robotic Manipulation (MIT 6.4210). https://manipulation.csail.mit.edu/"
  - "Tedrake, R. (2026). Underactuated Robotics. https://underactuated.csail.mit.edu/"
  - "Lynch, K. M. and Park, F. C. (2026). Modern Robotics specialization (Coursera). https://www.coursera.org/specializations/modernrobotics"
  - "Hugging Face (2026). Robotics course. https://huggingface.co/learn/robotics-course"
  - "ETH Zurich Robotic Systems Lab (2026). Programming for Robotics: ROS. https://rsl.ethz.ch/education-students/lectures/ros.html"
---

# Course Notes Collection

This page collects open courses worth taking alongside this textbook, and a way to keep notes on them inside the vault so they connect to everything else.

> **Etymology.** *Course* from Latin *cursus*, "a running": a course of study is a path run through a subject.

## Courses (checked November 2026)

| Course | Provider | Covers | Pairs with |
|---|---|---|---|
| **Modern Robotics** (specialization) | Northwestern, Coursera; free textbook | Screw theory, kinematics, dynamics, planning, control | [[Forward_Kinematics]], [[Robot_Control]] |
| **Robotic Manipulation** (6.4210) | MIT, online notes and code | Perception, grasping, planning, learning for manipulation | [[Manipulation]], [[MoveIt2]] |
| **Underactuated Robotics** | MIT, online notes and code | Nonlinear dynamics, optimal control, walking, RL | [[Control_of_Underactuated_Systems]], [[Optimal_Control]] |
| **Programming for Robotics: ROS** | ETH Zurich | Practical ROS skills | [[ROS_2_Overview]] |
| **Robotics course** | Hugging Face | Robot learning with LeRobot: datasets, imitation, policies | [[Imitation_Learning]], [[Vision-Language-Action_Models]] |
| **Robot Academy** | QUT (Peter Corke) | Short video lessons on robotics and vision fundamentals | [[Homogeneous_Transformation]], [[Computer_Vision_in_Robotics]] |
| **Self-Driving Cars** (specialization) | University of Toronto, Coursera | State estimation, perception, planning for vehicles | [[Autonomous_Navigation]] |
| **CS231n** | Stanford | Deep learning for computer vision | [[Convolutional_Neural_Networks]], [[Transformers_and_Attention]] |
| **Introduction to Robotics** (2.12, 2005) | MIT OpenCourseWare | Classic mechanics-focused introduction | [[Robots]] |

See also [[MOOCs_and_Courses]] for paid and platform courses.

## Taking course notes in this vault

Create one note per course, and link generously:

```markdown
---
title: "Notes: Robotic Manipulation (MIT 6.4210)"
type: reference
level: intermediate
subfield: [manipulation]
status: draft
tags: [courses, manipulation]
prerequisites: ["[[Forward_Kinematics]]"]
---

# Notes: Robotic Manipulation

## Lecture 3: Basic pick and place
- Spatial algebra → [[Homogeneous_Transformation]]
- Differential IK → [[Inverse_Kinematics]]
- Exercise result: ...

## Questions to follow up
- ...
```

Run `python3 tools/vault.py lint` to check the frontmatter, then look at the note in the graph explorer to see what it connects to.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Course_Notes_Collection]])
```
