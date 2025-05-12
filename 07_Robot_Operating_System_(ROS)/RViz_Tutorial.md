---
title: RViz Tutorial
description: An introduction to RViz, the primary 3D visualization tool for ROS, covering its purpose, key features, and basic usage.
tags:
  - RViz
  - ROS
  - visualization
  - debugging
  - robotics-software
  - TF
  - sensor-data
  - tutorial
  - software-stack
layout: default
category: robotics
author: Jordan_Smith_&_le_Chat
date: 2025-04-28
permalink: /rviz_tutorial/
related:
  - "[[ROS (Robot Operating System)]]"
  - "[[Visualization]]"
  - "[[TF_and_Topic_Architecture]]"
  - "[[URDF]]"
  - "[[LIDAR]]"
  - "[[Camera_Systems]]"
  - "[[IMU_Sensors]]"
  - "[[PointCloud2]]"
  - "[[Mapping]]"
  - "[[Localization]]"
  - "[[Topics]]"
  - "[[Nodes]]"
  - "[[Gazebo_Simulator]]"
  - "[[ROS_and_Software_Stacks]]"
---

# RViz Tutorial: Introduction to ROS Visualization

**RViz** (ROS Visualization tool) is a powerful and highly configurable 3D visualization environment specifically designed for the [[ROS (Robot Operating System)]]. It allows developers and users to view the state of a robot, its sensors, and the surrounding environment by subscribing to various ROS [[Topics]] and graphically displaying the data in a 3D scene.

**Important Distinction:** RViz is purely a *visualization* tool. It does **not** simulate physics or robot behavior; that is the role of simulators like [[Gazebo_Simulator]]. RViz simply displays data that is being *published* by other ROS [[Nodes]] (which could be running on a real robot or in a simulation).

## Purpose

RViz is an indispensable tool for ROS development, primarily used for:

* **Debugging:** Visually inspecting sensor data streams ([[LIDAR]] scans, [[Camera_Systems|camera images]], [[IMU_Sensors|IMU]] orientations, [[PointCloud2|point clouds]]) to quickly identify problems.
* **Monitoring:** Observing the real-time state of algorithms like [[Localization]], [[Mapping]], and [[SLAM]].
* **Understanding:** Grasping complex spatial relationships, such as the robot's kinematic structure ([[URDF]]), coordinate frame relationships ([[TF_and_Topic_Architecture|TF Tree]]), and planned paths.
* **Interaction:** Providing simple graphical interaction, such as setting navigation goals or initial pose estimates.
* **Custom Displays:** Allowing developers to visualize custom data structures using built-in or custom display types.

## Key Concepts and Features

RViz achieves its flexibility through a system of configurable **Displays**.

* **Displays:** These are plugins within RViz, each designed to subscribe to a specific ROS topic (or TF data) and render the received data in the 3D view. Common built-in displays include:
    * `RobotModel`: Displays a robot's kinematic model defined in a [[URDF]] file, colored and positioned according to `/joint_states` data.
    * `TF`: Visualizes coordinate frames and their relationships as defined by the ROS TF transform tree. Essential for debugging [[Kinematics]] and sensor poses.
    * `LaserScan`: Shows 2D [[LIDAR]] data as points in a plane.
    * `PointCloud2`: Renders 3D [[Point Cloud]] data from sensors like 3D LIDAR or depth cameras.
    * `Image`: Displays images from [[Camera_Systems|cameras]] within the RViz interface.
    * `Map`: Displays 2D occupancy grid maps (from `nav_msgs/OccupancyGrid`).
    * `Path`: Visualizes planned or executed paths (from `nav_msgs/Path`).
    * `Odometry`: Shows robot pose estimates and velocity vectors (from `nav_msgs/Odometry`).
    * `Pose`/`PoseArray`: Displays single or multiple poses as arrows.
    * `Marker`/`MarkerArray`: Highly versatile display for showing custom geometric shapes (arrows, spheres, cubes, lines, text, meshes) published by nodes for debugging or status visualization (e.g., planned waypoints, detected objects).
* **Configuration:** The entire setup of displays, their parameters (topics subscribed to, colors, sizes, history length, etc.), global options, and the camera viewpoint can be saved to and loaded from `.rviz` configuration files. This makes it easy to recreate complex visualization setups.
* **Global Options:** Settings that affect the entire view, most importantly the **Fixed Frame**.
    * **Fixed Frame:** Defines the stationary coordinate frame relative to which all other data is transformed before being displayed. Common choices include `/map`, `/odom`, or the robot's `/base_link`. Choosing the correct fixed frame is crucial for coherent visualization.
* **Time:** RViz typically uses ROS Time, allowing it to visualize data synchronized with the rest of the ROS system, including data played back from `rosbag` files.
* **Interaction Tools:** Tools available in the toolbar allow users to interact with the 3D scene, such as:
    * Setting a `2D Pose Estimate` for localization systems (like AMCL).
    * Setting a `2D Nav Goal` for navigation stacks.
    * Selecting points or objects in the scene.
    * Measuring distances.
* **[[Plugins]]:** RViz is extensible through plugins, allowing developers to create new display types for custom data or new tools for interacting with the visualization.

## Basic Usage (Tutorial Steps)

1.  **Launch RViz:**
    * ROS 1: `rosrun rviz rviz`
    * ROS 2: `ros2 run rviz2 rviz2`
    * Alternatively, launch RViz via a [[Launch Files|launch file]], often with a pre-loaded configuration: `roslaunch my_package my_rviz.launch` or `ros2 launch my_package my_rviz.launch.py`.
2.  **Set Fixed Frame:** In the "Global Options" section of the "Displays" panel (usually on the left), select an appropriate fixed frame from the dropdown list (e.g., `/map`, `/odom`, `/base_link`). The frame must exist in the TF tree being published. The background grid represents the XY plane of this fixed frame.
3.  **Add Displays:** Click the "Add" button at the bottom of the Displays panel. Select the desired display type (e.g., `TF`, `RobotModel`, `LaserScan`) from the list and click "OK".
4.  **Configure Displays:** Expand the newly added display in the list. Configure its specific parameters:
    * **Topic:** Select the correct ROS topic the display should subscribe to from the dropdown list (topics must be currently published with the correct message type).
    * **Style/Color/Size:** Adjust visual parameters as needed.
    * **Other Parameters:** Configure display-specific options (e.g., history length for odometry trails, decay time for scans). Check the status (Ok, Warning, Error) for diagnostics.
5.  **Navigate the 3D View:** Use mouse controls (typically left-click-drag to rotate, middle-click-drag or Shift+left-click-drag to pan, right-click-drag or scroll wheel to zoom) to change the viewpoint.
6.  **Save/Load Configuration:** Use the `File` -> `Save Config As...` or `File` -> `Open Config` menu options to save or load `.rviz` files.

RViz is an essential day-to-day tool for anyone working with ROS, providing invaluable insight into the spatial and temporal state of a robotic system.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #robotics OR #control-systems WHERE contains(file.outlinks, [[Robot_Operating_System_(ROS)]])