---
title: Python ROS Nodes
description: Guide to creating ROS nodes using Python (rospy for ROS 1, rclpy for ROS 2), covering core concepts, setup, use cases, and comparison with C++.
tags:
  - Python
  - ROS
  - rospy
  - rclpy
  - node
  - robotics-software
  - development
  - coding
  - implementation
layout: default
category: robotics
author: Jordan_Smith_&_le_Chat
date: 2025-04-29
permalink: /python_ros_nodes/
related:
  - "[[Python_for_Robotics]]"
  - "[[Robot_Operating_System_(ROS)]]"
  - "[[Nodes]]"
  - "[[Topics]]"
  - "[[Services]]"
  - "[[Actions]]"
  - "[[Messages]]"
  - "[[Parameters]]"
  - "[[C++_Motion_Planning]]"
  - "[[Custom_Packages_and_Nodes]]"
  - "[[Coding_and_Implementation]]"
---

# Python ROS Nodes

[[Python_for_Robotics]] is a popular and powerful language choice for developing [[Nodes|ROS nodes]], alongside [[C++_Motion_Planning|C++]]. ROS provides dedicated Python client libraries – `rospy` for ROS 1 and `rclpy` for ROS 2 – enabling developers to leverage Python's ease of use, extensive libraries (especially in [[AI_and_Robot_Control|AI]]/[[Machine_Learning|Machine Learning]]), and rapid development cycle for various robotics tasks.

---

## Why Use Python for ROS Nodes?

* **Ease of Use & Rapid Prototyping:** Python's simpler syntax and dynamic typing allow for faster development and iteration compared to C++. This is ideal for prototyping algorithms, scripting tasks, and developing non-performance-critical nodes.
* **Rich Ecosystem:** Access to a vast collection of Python libraries for data science, machine learning (TensorFlow, PyTorch, scikit-learn), web development, and more, facilitating integration with other technologies.
* **AI/ML Integration:** Python is the dominant language in the AI/ML community, making it straightforward to integrate trained models and AI algorithms directly into ROS nodes.
* **Readability:** Python code is often considered more readable and maintainable, especially for developers less familiar with C++.

---

## Setting Up a Python ROS Node

1.  **Package Structure:** Python ROS nodes are typically placed as scripts within the `scripts/` directory of a [[Custom_Packages_and_Nodes|ROS package]].
2.  **Executable Permissions:** Python scripts intended to be run as nodes need executable permissions. Use `chmod +x your_node_script.py` in the terminal.
3.  **Shebang Line:** Start your Python script with `#!/usr/bin/env python3` (or `python` for ROS 1) to indicate it should be executed with the Python interpreter found in the environment.
4.  **Dependencies:**
    * **ROS Dependencies:** Declare dependencies on `rospy` or `rclpy` and any necessary message packages (e.g., `std_msgs`, `sensor_msgs`) in your `package.xml` file using `<exec_depend>` (or `<depend>` in ROS 1).
    * **Python Libraries:** Manage external Python library dependencies using standard Python practices (e.g., `requirements.txt` and `pip`, or listing them in `setup.py` for ROS 2 `ament_python` packages).
5.  **Installation (Making Nodes Executable):**
    * **ROS 1 (Catkin):** Typically involves adding an `install` rule in `CMakeLists.txt` to install the script into the package's `lib/<package_name>/` directory.
    * **ROS 2 (Colcon/Ament):** In `setup.py` for an `ament_python` package, define entry points under `console_scripts` to make the Python script runnable via `ros2 run`.

---

## Core ROS Concepts in Python

The core ROS communication mechanisms ([[Topics]], [[Services]], [[Actions]], [[Parameters]]) are accessed via the `rospy` (ROS 1) or `rclpy` (ROS 2) libraries.

| Feature         | ROS 1 (`rospy`)                     | ROS 2 (`rclpy`)                                       |
| :-------------- | :---------------------------------- | :---------------------------------------------------- |
| **Initialization** | `rospy.init_node('node_name')`      | `rclpy.init()` <br> `node = rclpy.create_node('node_name')` |
| **Publisher** | `pub = rospy.Publisher(...)`        | `pub = node.create_publisher(...)`                    |
|                 | `pub.publish(msg)`                  | `pub.publish(msg)`                                    |
| **Subscriber** | `sub = rospy.Subscriber(...)`       | `sub = node.create_subscription(...)`                 |
|                 | *(Callback function passed)* | *(Callback function passed)* |
| **Service Server**| `srv = rospy.Service(...)`          | `srv = node.create_service(...)`                      |
|                 | *(Callback function passed)* | *(Callback function passed)* |
| **Service Client**| `proxy = rospy.ServiceProxy(...)`   | `client = node.create_client(...)`                    |
|                 | `resp = proxy(req)`                 | `future = client.call_async(req)`                     |
| **Action Server** | `import actionlib` <br> `server = actionlib.SimpleActionServer(...)` | `from rclpy.action import ActionServer` <br> `server = ActionServer(...)` |
| **Action Client** | `import actionlib` <br> `client = actionlib.SimpleActionClient(...)` | `from rclpy.action import ActionClient` <br> `client = ActionClient(...)` |
| **Get Parameter** | `rospy.get_param('param_name', default)` | `node.get_parameter('param_name').value` (or declare first) |
| **Set Parameter** | `rospy.set_param('param_name', value)` | `node.set_parameters([Parameter(...)])`               |
| **Rate Limiting** | `rate = rospy.Rate(hz)` <br> `rate.sleep()` | `rate = node.create_rate(hz)` <br> `rate.sleep()`           |
| **Spinning** | `rospy.spin()` (blocking) <br> `while not rospy.is_shutdown(): rate.sleep()` (manual loop) | `rclpy.spin(node)` (blocking) <br> `rclpy.spin_once(node, timeout_sec=...)` (non-blocking) |
| **Logging** | `rospy.loginfo("msg")`              | `node.get_logger().info("msg")`                       |

*Note:* ROS 2's `rclpy` is generally more object-oriented, with most functionalities accessed via the `node` object, whereas `rospy` often uses global functions. ROS 2 also makes heavy use of asynchronous programming (`async`/`await`) for clients and actions.

---

## Common Use Cases

Python ROS nodes excel in areas where development speed and library availability outweigh raw computational performance:

* **High-Level Task Coordination:** Implementing state machines, behavior trees, or task planners that orchestrate other nodes.
* **AI/ML Integration:** Running inference with TensorFlow/PyTorch models for [[Perception|perception]], creating [[Reinforcement Learning for Robots|RL]] agents, or using Natural Language Processing libraries.
* **Rapid Prototyping:** Quickly developing and testing new algorithms or system logic before potentially optimizing critical parts in C++.
* **Simple [[Hardware_Interface_Setup|Hardware Interfaces]]:** Interfacing with devices via [[Serial_Communication_Tutorials|Serial]] or simple USB protocols where Python libraries exist and real-time performance is not paramount (e.g., using `pyserial`). `rosserial_python` is a common package for Arduino/microcontroller communication in ROS 1.
* **Utility Scripts:** Creating simple nodes for data conversion, logging, system monitoring, or parameter management.
* **GUI Development:** Building graphical user interfaces using ROS tools like `rqt` which heavily utilizes Python.

---

## Python vs. C++ in ROS

* **Choose Python when:**
    * Development speed is critical.
    * Integrating with existing Python libraries (especially AI/ML) is needed.
    * Computational performance is not the primary bottleneck.
    * Readability and ease of maintenance are high priorities.
* **Choose C++ when:**
    * High computational performance or real-time execution is required (e.g., low-level control loops, intensive [[C++_Motion_Planning|motion planning]], heavy image/point cloud processing).
    * Fine-grained memory management is necessary.
    * Interfacing with hardware requires specific C/C++ SDKs or low-level access.

Often, complex ROS systems use a mix of C++ and Python nodes, leveraging the strengths of each language for different components of the system.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #implementation  OR #coding WHERE contains(file.outlinks, [[Python_ROS_Nodes]])