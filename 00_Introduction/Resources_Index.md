---
title: Resources Index
description: A master index of key external resources, tools, and references for the Robotics Vault.
tags:
  - index
  - resources
  - tools
  - links
  - references
  - vault-meta
layout: default
category: robotics
author: Jordan_Smith_&_le_Chat
date: 2025-04-26
permalink: /resources_index/
related:
  - "[[MOOCs_and_Courses]]"
  - "[[Research_Papers_Index]]"
  - "[[Best_YouTube_Channels]]"
  - "[[Simulators_and_IDEs]]"
  - "[[Hardware_Shopping_List]]"
---

# Resources Index

This is the master index for key external resources, tools, and references curated within the Robotics Vault. Use the internal links below (like `[[MOOCs_and_Courses]]`) to navigate to more detailed lists within this vault.

---

## Core Textbooks & Reading

Essential texts covering fundamental robotics concepts.
### Robotics
- *Introduction to Autonomous Robots* – Correll, Bekris, et al.
- *Probabilistic Robotics* – Sebastian Thrun, Wolfram Burgard, Dieter Fox
- *Modern Robotics: Mechanics, Planning, and Control* – Kevin M. Lynch & Frank C. Park
- *Robot Modeling and Control* – Mark W. Spong, Seth Hutchinson
### AI
- *Deep Learning* - Goodfellow, Bengio, Courville
- *Reinforcement Learning: An Introduction* - Sutton, Barto
- *Artificial Intelligence: A Modern Approach* - Russell, Norvig

---

## 📄 Academic Papers & Whitepapers [[Research_Papers_Index]]

Links to seminal papers, surveys, and influential whitepapers. See [[Research_Papers_Index]] for a more comprehensive list.

- [SLAM: A Survey of Mapping and Localization Techniques (2022)](https://arxiv.org/abs/2202.02385)
- [Reinforcement Learning for Robotics: A Review (2020)](https://arxiv.org/abs/2007.14491)
- [RT-1 and Robotic Foundation Models (Google Robotics)](https://robotics-transformer.github.io/)

---

## Online Courses & MOOCs [[MOOCs_and_Courses]]

High-quality online learning materials. Find more at [[MOOCs_and_Courses]].

- [Robotics Specialization – Coursera (Penn)](https://www.coursera.org/specializations/robotics)
- [MIT OpenCourseWare – Introduction to Robotics](https://ocw.mit.edu/courses/electrical-engineering-and-computer-science/6-141-robotic-science-and-systems-i-fall-2020/)
- [ROS For Beginners – The Construct](https://www.theconstructsim.com/robotigniteacademy_learnros/)

---

## YouTube Channels [[Best_YouTube_Channels]]

Valuable video resources for learning and staying updated. See [[Best_YouTube_Channels]] for more channels.

- [Robotics with ROS](https://www.youtube.com/c/TheConstruct)
- [NVIDIA AI and Robotics](https://www.youtube.com/@NVIDIADeveloper)

---

## ⚙️ Simulators & Tools [[Simulators_and_IDEs]]

Software for simulation, development, and visualization. Explore more options in [[Simulators_and_IDEs]].

- [Gazebo Simulation](https://gazebosim.org/)
- [Webots Open Source Robot Simulator](https://cyberbotics.com/)
- [RViz (ROS Visualizer)](http://wiki.ros.org/rviz)

---

## Hardware & Kits [[Hardware_Shopping_List]]

Platforms and components for building and experimenting. Check [[Hardware_Shopping_List]] for curated items.

- [TurtleBot 4 – Official ROS Educational Robot](https://www.clearpathrobotics.com/turtlebot/)
- [NVIDIA Jetson Platform](https://developer.nvidia.com/embedded-computing)
- [OpenCV AI Kit (OAK-D)](https://opencvaikit.com/)

---

## Obsidian Plugins (Recommended)

Plugins enhancing the functionality of this vault.

- [Dataview](https://blacksmithgu.github.io/obsidian-dataview/) - *Essential for dynamic tables and lists.*
- [Canvas](https://help.obsidian.md/Canvas) - *For visual thinking and mind-mapping.*
- [Excalidraw](https://github.com/zsviczian/obsidian-excalidraw-plugin) - *For sketching diagrams.*
- [Advanced Tables](https://github.com/tgrosinger/advanced-tables-obsidian) - *For easier markdown table editing.*

---

## AI & Robotics Labs [[Key_Figures_and_Labs]]

Prominent research institutions driving innovation. See [[Key_Figures_and_Labs]] for more detail.

- [Open Robotics](https://www.openrobotics.org/)
- [CMU Robotics Institute](https://www.ri.cmu.edu/)
- [ETH Zurich Autonomous Systems Lab](https://asl.ethz.ch/)

---

## Notes Referencing This Index

This Dataview query shows which notes in your vault link back to this `Resources_Index` page, helping you trace how resources are used. Ensure notes reference this page in their `related:` frontmatter field (e.g., `related: ["[[Resources_Index]]"]`).

```
dataview
TABLE related AS "Related Fields Containing Link"
FROM "" 
WHERE contains(file.outlinks, this.file.link) OR contains(related, this.file.name)
SORT file.name ASC
```



