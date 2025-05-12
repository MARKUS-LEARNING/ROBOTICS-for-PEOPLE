---
title: Custom Packages and Nodes
description: Explains how to create custom software components (packages and nodes) within the ROS framework to implement specific robotic functionalities.
tags:
  - ROS
  - package
  - node
  - robotics-software
  - development
  - catkin
  - colcon
  - middleware
  - software-stack
layout: default
category: robotics
author: Jordan_Smith_&_le_Chat
date: 2025-04-28
permalink: /custom_packages_and_nodes/
related:
  - "[[ROS (Robot Operating System)]]"
  - "[[Nodes]]"
  - "[[Topics]]"
  - "[[Services]]"
  - "[[Actions]]"
  - "[[Messages]]"
  - "[[Launch Files]]"
  - "[[ROS_2_Overview]]"
  - "[[Python_ROS_Nodes]]"
  - "[[Git_Integration_and_Version_Control]]"
---

# Custom Packages and Nodes in ROS

While the [[ROS (Robot Operating System)]] ecosystem provides a vast collection of publicly available software packages for common robotics tasks (e.g., navigation, perception, simulation), most real-world robotics applications require the development of custom software to implement specific algorithms, control unique hardware, or orchestrate system behavior. In ROS, custom software development primarily involves creating **custom packages** and **custom nodes**.

---

## ROS Packages

A **package** is the fundamental unit of software organization and distribution in ROS. It groups together related software components needed to perform a specific function.

* **Purpose:**
    * **Organization:** Keeps related code (source files, launch files, configuration, message definitions) together.
    * **Build Unit:** Packages are the smallest unit that can be built by ROS build systems like Catkin (ROS 1) or Colcon (ROS 2).
    * **Dependency Management:** A package explicitly declares its dependencies on other ROS packages and system libraries.
    * **Reusability & Sharing:** Packages are designed to be easily shared and reused across different projects and robots.

* **Standard Structure:** While flexible, ROS packages typically follow a standard directory structure:
    * `src/`: Contains source code (C++, Python, etc.).
    * `include/package_name/`: Contains C++ header files intended for use by other packages.
    * `msg/`: Contains custom [[Messages|message (.msg) definitions]].
    * `srv/`: Contains custom [[Services|service (.srv) definitions]].
    * `action/`: Contains custom [[Actions|action (.action) definitions]].
    * `launch/`: Contains [[Launch Files|launch files (.launch, .launch.py, .launch.xml)]] used to start nodes.
    * `config/`: Contains configuration files (e.g., YAML parameter files).
    * `CMakeLists.txt`: (Required for C++ packages) Build instructions for CMake, used by Catkin/Colcon. Specifies executables, libraries, dependencies, and code generation steps for messages/services/actions.
    * `package.xml`: (Required) Manifest file containing package metadata (name, version, author, license), build system dependencies (e.g., `ament_cmake`, `catkin`), build tool dependencies (e.g., `rosidl_default_generators`), and runtime dependencies (other ROS packages needed).
    * `setup.py` & `setup.cfg`: (Required for Python packages) Build and installation instructions for Python setuptools, used by Colcon.
    * `resource/`: Contains non-code resources marked for installation.

* **Creating Packages:** Standard tools are used to create the basic package structure:
    * **ROS 1:** `catkin_create_pkg <package_name> [dependencies...]`
    * **ROS 2:** `ros2 pkg create --build-type <build_type> <package_name> --dependencies [deps...]` (where `build_type` is typically `ament_cmake` for C++ or `ament_python` for Python).

---

## ROS Nodes

A **node** is an executable program within a ROS system. Nodes use ROS client libraries (`rclcpp`/`roscpp` for C++, `rclpy`/`rospy` for Python) to communicate with other nodes over the ROS network using [[Topics]], [[Services]], and [[Actions]].

* **Purpose:** Each node typically encapsulates a specific, well-defined functionality (e.g., controlling a specific sensor, running a perception algorithm, executing a control loop, planning a path). The modularity of nodes facilitates fault tolerance, code reuse, and easier debugging.

* **Writing Custom Nodes:** Developing a node involves:
    1.  **Initialization:** Initializing the ROS client library (e.g., `rclcpp::init`, `rospy.init_node`). Creating a Node object (in ROS 2) or handle (in ROS 1).
    2.  **Communication Setup:** Creating instances of publishers, subscribers, service servers, service clients, action servers, or action clients, associating them with specific topic/service/action names and message/service/action types.
    3.  **Callback Functions:** Implementing functions that are executed asynchronously when new data arrives on a subscribed topic, a service request is received, or an action goal is accepted.
    4.  **Main Loop / Spinning:** Keeping the node running and processing events (like incoming messages or service requests). This is typically done using functions like `rclcpp::spin()`, `rclcpp::spin_some()`, or `rospy.spin()`. Rate limiters (`ros::Rate`) are often used to control loop execution frequency.
    5.  **Parameters:** Declaring and accessing ROS parameters for runtime configuration.
    6.  **Logging:** Using ROS logging utilities (e.g., `RCLCPP_INFO`, `rospy.loginfo`) for informative output and debugging.

* **Example Files:** See [[Python_ROS_Nodes]] for Python examples.

---

## Custom Interfaces (Messages, Services, Actions)

If standard ROS message/service/action types are insufficient for communication between custom nodes, developers can define their own:

* **Messages (.msg):** Define data structures for [[Topics]]. Files placed in the `msg/` directory.
* **Services (.srv):** Define request and response data structures for [[Services]]. Files placed in the `srv/` directory, separating request and response fields with `---`.
* **Actions (.action):** Define goal, result, and feedback data structures for [[Actions]]. Files placed in the `action/` directory, separating goal, result, and feedback fields with `---`.

These definition files use standard ROS field types (e.g., `int32`, `float64`, `string`, `Header`) or other custom types. The ROS build system automatically generates corresponding source code for C++ and Python during the build process.

---

## Build Systems

ROS uses build systems to compile source code, generate code from interface definitions, link against dependencies, and create executable nodes and libraries.

* **ROS 1 (Catkin):** An extension of CMake. Workspaces are typically managed using `catkin_make` or `catkin build`. Building involves processing `CMakeLists.txt` and `package.xml` files.
* **ROS 2 (Colcon):** A meta-build tool that invokes underlying build systems like CMake (for C++) and setuptools (for Python). It uses `package.xml` for metadata/dependencies and `CMakeLists.txt` or `setup.py` for build instructions. Building is typically done using `colcon build`.

---

## Best Practices

Developing custom ROS packages and nodes benefits from standard software engineering practices:

* **Modularity:** Design nodes with single, clear responsibilities.
* **Clear Interfaces:** Define messages, services, and actions carefully.
* **Documentation:** Document code, package functionality, and usage in README files and `package.xml`.
* **[[Git_Integration_and_Version_Control|Version Control]]:** Use tools like Git to manage source code development.
* **Testing:** Implement unit and integration tests.

Creating custom packages and nodes is the core workflow for extending ROS capabilities and building tailored robotic applications.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #robotics OR #kinematics WHERE contains(file.outlinks, [[Custom_Packages_and_Nodes]])