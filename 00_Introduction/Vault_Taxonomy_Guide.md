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

* **`00_Start_Here`**: Contains introductory and meta-notes about the vault itself, including purpose (`[[Vault_Purpose_and_Scope]]`), usage guides (`[[How_to_Use_This_Vault]]`), this taxonomy guide, and dashboards (`[[Robotics_Vault_Dashboard]]`).
* **`01_Glossary`**: Definitions of key terms used throughout robotics (e.g., `[[SLAM]]`, `[[PID_Control]]`).
* **`02_Kinematics_and_Dynamics`**: Notes related to the mathematics of robot motion, forces, and control (e.g., `[[Forward_Kinematics]]`, `[[Inverse_Kinematics]]`).
* **`03_Sensors_and_Perception`**: Information on various sensor types (`[[LIDAR]]`, `[[Camera_Systems]]`), calibration, and how robots perceive their environment.
* **`04_AI_and_Robot_Control`**: Focuses on artificial intelligence techniques like machine learning (`[[Reinforcement_Learning_for_Robots]]`), path planning (`[[Path_Planning_Algorithms]]`), and computer vision (`[[Computer_Vision_in_Robotics]]`) applied to robot control.
* **`05_ROS_and_Software_Stacks`**: Details about the Robot Operating System (`[[ROS_2_Overview]]`), simulators (`[[Gazebo_Simulator]]`), and related software concepts (`[[TF_and_Topic_Architecture]]`).
* **`06_Robot_Types_and_Applications`**: Descriptions of different robot morphologies (`[[Humanoid_Robots]]`, `[[Mobile_Robots]]`, `[[Industrial_Arms]]`) and their application domains.
* **`07_Robotics_History_and_Future`**: Covers the evolution of robotics (`[[History_of_Robotics]]`), key figures (`[[Key_Figures_and_Labs]]`), ethical considerations (`[[Robot_Ethics_and_Policy_Debates]]`), and future trends (`[[Future_Trends_2025-2035]]`).
* **`08_Coding_and_Implementation`**: Practical code examples (`[[Python_ROS_Nodes]]`, `[[C++_Motion_Planning]]`), tutorials for hardware interfacing, and version control (`[[Git_Integration_and_Version_Control]]`).
* **`09_Labs_Projects_and_Tutorials`**: Hands-on experiments (`[[Sim2Real_Experiments]]`), project documentation, and learning exercises (`[[Turtlesim_Projects]]`).
* **`10_Tools_References_and_Links`**: External resources like courses (`[[MOOCs_and_Courses]]`), research papers (`[[Research_Papers_Index]]`), simulators (`[[Simulators_and_IDEs]]`), and hardware lists (`[[Hardware_Shopping_List]]`).

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