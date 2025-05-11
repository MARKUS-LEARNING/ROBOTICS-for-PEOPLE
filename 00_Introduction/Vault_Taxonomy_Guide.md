---
title: Vault Taxonomy Guide
description: Explains the organizational structure (folders and tags) of the Robotics Vault.
tags:
  - guide
  - taxonomy
  - organization
  - vault-meta
layout: default
category: robotics
author: Jordan_Smith_&_le_Chat
date: 2025-04-26
permalink: /vault_taxonomy_guide/
related:
  - "[[Vault_Purpose_and_Scope]]"
  - "[[How_to_Use_This_Vault]]"
---

# Vault Taxonomy Guide

This guide explains the organizational structure used within this Robotics Obsidian Vault to ensure consistency and discoverability of information.

## Folder Structure

The vault uses a numbered folder system to group related topics logically:

- **`00_Introduction`**: An overview of the field of robotics, its significance, and the scope of the course.
- **`01_Fundamentals_of_Robotics`**: Definitions of fundamental robotics terms.
- **`02_Mathematics_for_Robotics`**: The mathematics of robot motion and forces.
- **`03_Kinematics_and_Dynamics`**: How robots sense and interpret their environment.
- **`04_Sensors_and_Perception`**: Autonomy, machine learning, and control strategies.
- **`05_AI_and_Machine_Learning`**: Overview of [[ROS_2_Overview]] and related tools like [[Gazebo_Simulator]].
- **`06_Robot_Control`**: Profiles of [[Industrial_Arms]], [[Mobile_Robots]], [[Humanoid_Robots]], etc.
- **`07_Robot_Operating_System_(ROS)`**: Timelines [[History_of_Robotics]], [[Future_Trends_2025-2035]], and [[Robot_Ethics_and_Policy_Debates]].
- **`08_Robot_Types_and_Applications`**: Examples and applications of [[Python_ROS_Nodes]], [[C++_Motion_Planning]], and practical examples.
- **`09_Advanced_Topics`**: Hands-on work like [[Turtlesim_Projects]] and [[Sim2Real_Experiments]].
- **`10_Research_and_Development`**: External resources including [[MOOCs_and_Courses]], [[Research_Papers_Index]], [[Simulators_and_IDEs]].
- **`11_Practical_Implementation`**: External resources including [[MOOCs_and_Courses]], [[Research_Papers_Index]], [[Simulators_and_IDEs]].
- **`12_Labs_and_Tutorials`**: External resources including [[MOOCs_and_Courses]], [[Research_Papers_Index]], [[Simulators_and_IDEs]].
- **`13_Tools_References_and_Links`**: External resources including [[MOOCs_and_Courses]], [[Research_Papers_Index]], [[Simulators_and_IDEs]].


## Tagging Strategy

Tags supplement the folder structure for more granular topic identification and linking. Use tags (#tag-name) to denote:

* **Specific Concepts**: e.g., `#kinematics`, `#SLAM`, `#computer-vision`, `#reinforcement-learning`, `#sensor-fusion`, `#control-theory`
* **Robot Types/Components**: e.g., `#mobile-robot`, `#manipulator-arm`, `#humanoid`, `#lidar`, `#imu`, `#actuator`, `#camera`
* **Software/Tools**: e.g., `#ROS`, `#ROS2`, `#python`, `#c++`, `#gazebo`, `#rviz`, `#dataview`, `#opencv`
* **Document Type**: e.g., `#glossary-term`, `#tutorial`, `#project-log`, `#research-paper`, `#course-notes`, `#concept`, `#example-code`
* **Status/Meta**: e.g., `#todo`, `#in-progress`, `#review-needed`, `#stub`, `#vault-meta`, `#guide`

**Consistency:** Try to reuse existing tags where appropriate. Check the Tags pane in Obsidian to see existing tags.

## Linking Strategy

Use internal wiki-links `[[Note Title]]` generously to connect related concepts across different folders. Effective linking transforms your vault from a collection of notes into a true knowledge *graph*.

* Link key terms back to their definitions in the `01_Glossary`.
* Connect theoretical concepts in `02_Kinematics_and_Dynamics` or `04_AI_and_Robot_Control` to practical implementations in `08_Coding_and_Implementation` or `09_Labs_Projects_and_Tutorials`.
* Link specific tools or software mentioned in notes to their entries in `05_ROS_and_Software_Stacks` or `10_Tools_References_and_Links`.

---

*This guide helps maintain the organization and interconnectedness of the vault. Refer back here when creating new notes or tags to ensure they align with the established structure.*
