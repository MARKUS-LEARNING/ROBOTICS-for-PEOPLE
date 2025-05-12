---
title: Turtlesim Projects
description: An overview of the Turtlesim simulator in ROS and a collection point for projects and experiments using it to learn ROS concepts.
tags:
  - Turtlesim
  - ROS
  - tutorial
  - project
  - lab
  - simulation
  - learning
  - beginner
layout: default
category: robotics
author: Jordan_Smith_&_le_Chat
date: 2025-04-29
permalink: /turtlesim_projects/
related:
  - "[[ROS_2_Overview]]"
  - "[[Nodes]]"
  - "[[Topics]]"
  - "[[Services]]"
  - "[[Parameters]]"
  - "[[Python_ROS_Nodes]]"
  - "[[C++_Motion_Planning]]"
  - "[[Control_Theory]]"
  - "[[Simulation]]"
  - "[[Labs_Projects_and_Tutorials]]"
---

# Turtlesim Projects

**Turtlesim** is a lightweight, 2D simulator that comes standard with many [[ROS (Robot Operating System)|ROS]] distributions. It features a simple graphical window where one or more "turtle" robots move within a 2D plane, optionally drawing trails behind them. While visually basic, Turtlesim is an invaluable tool for beginners learning core ROS concepts without the complexity of real hardware or advanced simulators like [[Gazebo_Simulator]].

Its primary purpose is educational: providing a hands-on way to understand and experiment with fundamental ROS mechanisms.

---

## Core Turtlesim Components and Interfaces

Typically, working with Turtlesim involves interacting with these key elements:

* **Nodes:**
    * `turtlesim_node`: The main simulator node that displays the window, manages turtle states, and provides services. Launched via `rosrun turtlesim turtlesim_node`.
    * `turtle_teleop_key`: A node that listens to keyboard arrow key presses and publishes velocity commands to control a turtle. Launched via `rosrun turtlesim turtle_teleop_key`.
* **Topics:**
    * `/turtleX/cmd_vel` (`geometry_msgs/Twist`): **Input** topic where nodes publish linear velocity (along the turtle's x-axis) and angular velocity (around the turtle's z-axis) commands to control turtle X (e.g., `/turtle1/cmd_vel`).
    * `/turtleX/pose` (`turtlesim/Pose`): **Output** topic where `turtlesim_node` publishes the current 2D pose (x, y, theta), linear velocity, and angular velocity of turtle X.
    * `/turtleX/color_sensor` (`turtlesim/Color`): **Output** topic publishing the RGB color of the background directly beneath turtle X.
* **Services:**
    * `/clear`: Clears the drawing trails in the simulator window.
    * `/reset`: Resets the simulation to its default state (usually one turtle at the center).
    * `/spawn`: Creates a new turtle at a specified (x, y, theta) pose, optionally with a name.
    * `/kill`: Removes a turtle specified by name.
    * `/turtleX/set_pen`: Allows changing the color (RGB), width, and on/off state of the drawing pen for turtle X.
    * `/turtleX/teleport_absolute`: Instantly moves turtle X to an absolute (x, y, theta) pose.
    * `/turtleX/teleport_relative`: Instantly moves turtle X by a relative linear distance and angular turn.
* **Parameters:**
    * `/background_r`, `/background_g`, `/background_b`: Control the background color of the simulation window (integer values 0-255). Can be set using `rosparam set`.

---

## Example Project Ideas

This note can serve as a collection point for documenting your own Turtlesim-based projects. Create separate notes for each project and link them here, or document them directly using headings. Consider using the structure outlined in [[Manipulator_Control_Experiments]] for documenting each project.

Here are some classic Turtlesim projects to practice core ROS skills:

1.  **Drawing Shapes (Square, Circle, Triangle):**
    * **Goal:** Write a node ([[Python_ROS_Nodes|Python]] or [[C++_Motion_Planning|C++]]) that publishes timed velocity commands to `/turtle1/cmd_vel` to make the turtle trace out a specific geometric shape.
    * **Concepts Learned:** Creating a ROS node, creating a Publisher, using ROS message types (`geometry_msgs/Twist`), controlling timing (`rospy.Rate` / `rclpy.Rate`), basic open-loop control.

2.  **Go-to-Goal Controller:**
    * **Goal:** Write a node that makes the turtle drive towards a target (x, y) coordinate. The node should subscribe to `/turtle1/pose` to get the current position and orientation, calculate the distance and angle error to the goal, and publish appropriate linear and angular velocities to `/turtle1/cmd_vel` using a simple proportional [[Control Theory|controller]]. Stop when close enough to the goal.
    * **Concepts Learned:** Creating a Subscriber, defining callback functions, basic feedback control logic, using TF math (for angles) or `atan2`.

3.  **Pen Control and Services:**
    * **Goal:** Enhance the shape-drawing or go-to-goal projects by using ROS [[Services]]. For example, create a service that allows lifting/lowering the pen (`/turtle1/set_pen`) or teleporting the turtle (`/turtle1/teleport_absolute`) before starting a drawing.
    * **Concepts Learned:** Creating Service Clients, calling services, understanding request/response structures.

4.  **Multi-Turtle Follower:**
    * **Goal:** Use the `/spawn` service to create a second turtle. Write a node that makes the second turtle follow the first turtle by subscribing to `/turtle1/pose` and publishing velocity commands to `/turtle2/cmd_vel`.
    * **Concepts Learned:** Using services (`/spawn`), handling multiple namespaces (`/turtle1`, `/turtle2`), implementing a follower behavior based on relative pose.

5.  **Wall Follower (More Advanced):**
    * **Goal:** (Requires modification or a different simulator setup, as basic Turtlesim has no walls). If using a Turtlesim variant with walls or simulated range sensors, implement a simple wall-following behavior.
    * **Concepts Learned:** Sensor processing, reactive control logic.

---

Documenting your Turtlesim projects, including objectives, code snippets, results (screenshots of paths drawn), and challenges faced, is a great way to solidify your understanding of fundamental ROS principles before moving on to more complex robots and simulators.

--- 

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #labs OR #robotics WHERE contains(file.outlinks, [[Turtlesim_Projects]])