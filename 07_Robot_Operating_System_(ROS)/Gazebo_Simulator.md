---
title: Gazebo Simulator
description: Describes the Gazebo 3D robotics simulator, its features, robot/environment modeling, and its integration with ROS.
tags:
  - Gazebo
  - simulation
  - robotics-software
  - ROS
  - physics-engine
  - 3D
  - testing
  - development
  - URDF
  - SDF
layout: default
category: robotics
author: Jordan_Smith_&_le_Chat
date: 2025-04-28
permalink: /gazebo_simulator/
related:
  - "[[Robot_Operating_System_(ROS)]]"
  - "[[Simulation]]"
  - "[[URDF]]"
  - "[[SDF]]"
  - "[[Robotics Software]]"
  - "[[Reinforcement Learning (RL)]]"
  - "[[Topics]]"
  - "[[Services]]"
  - "[[Plugins]]"
  - "[[RViz_Tutorial]]"
---

# Gazebo Simulator

**Gazebo** is a powerful, open-source 3D robotics simulator that enables the simulation of complex robots, sensors, and objects within realistic indoor and outdoor environments. It is capable of simulating rigid body dynamics with high fidelity, generating realistic sensor feedback, and provides strong integration with the [[ROS (Robot Operating System)]] framework, making it a cornerstone tool for robotics development, testing, and research.

---

## Key Features

* **Physics Simulation:** Accurately simulates the physical interactions between objects, including collisions, friction, and gravity. It supports multiple high-performance physics engines (such as ODE - Open Dynamics Engine, Bullet, Simbody, DART) allowing users to choose based on their needs.
* **Sensor Simulation:** Provides models for a wide variety of common robotic sensors, including:
    * [[LIDAR]] (2D and 3D)
    * [[Camera_Systems]] (monocular, stereo, RGB-D/depth cameras)
    * [[IMU_Sensors]] (Inertial Measurement Units)
    * Contact Sensors (Bumpers, Force/Torque)
    * GPS
    * Sonar
    These simulated sensors generate data mimicking their real-world counterparts, often including configurable noise models.
* **Robot Models:** Robots are typically defined using XML-based description formats:
    * **[[SDF]] (Simulation Description Format):** Gazebo's native format, allows detailed specification of visuals, collision geometry, inertial properties, joints, sensors, plugins, and world properties.
    * **[[URDF]] (Unified Robot Description Format):** A common ROS format primarily defining kinematic and visual properties. Often used with extensions (like `.xacro` macros) and Gazebo-specific tags (`<gazebo>`) to add simulation details like collision properties, materials, sensor plugins, and actuator interfaces.
* **Environment Modeling:** Allows the creation of complex 3D worlds, including terrains, buildings, lighting conditions, and libraries of common objects (tables, chairs, etc.). Worlds are typically defined using SDF.
* **Interfaces:**
    * **GUI:** Provides a graphical interface for visualizing the simulation, inspecting models and sensor data, and interacting with the simulated world.
    * **APIs:** Offers programming interfaces (primarily C++ API and ROS interfaces) for controlling robots, retrieving sensor data, and interacting with the simulation programmatically.
* **Extensibility ([[Plugins]]):** Supports custom C++ plugins to extend its capabilities, allowing users to model custom sensors, actuators, physics effects, control interfaces, or world behaviors.

---

## ROS Integration

Gazebo's tight integration with ROS is one of its key strengths, facilitated primarily by the `gazebo_ros` package collection.

* **Mechanism:** ROS integration works through Gazebo **plugins** that act as bridges between the Gazebo simulation environment and the ROS communication network.
* **Functionality:**
    * **Sensors:** Simulated sensors (cameras, LiDAR, IMU, etc.) use plugins to publish their data onto ROS [[Topics]] using standard ROS message types (e.g., `sensor_msgs/Image`, `sensor_msgs/LaserScan`, `sensor_msgs/Imu`).
    * **Actuators:** Plugins (like the `diffdrive_plugin` for differential drive robots or `gazebo_ros_control` for manipulators) subscribe to ROS command [[Topics]] (e.g., `geometry_msgs/Twist` for `/cmd_vel`, `trajectory_msgs/JointTrajectory` for arm control) and translate these commands into forces or velocities applied to simulated joints or links.
    * **Robot State:** Simulated joint states are often published on the `/joint_states` topic, which can be used by `robot_state_publisher` to compute and broadcast [[TF_and_Topic_Architecture|TF (transform)]] frames for the simulated robot. Simulated ground truth odometry might be published on `/odom`.
    * **Simulation Control:** ROS [[Services]] are typically provided to pause, unpause, reset the simulation, and spawn or delete models.
    * **Simulation Time:** Gazebo can publish simulation time on the `/clock` topic. Setting the ROS parameter `/use_sim_time` to `true` causes ROS nodes to use Gazebo's clock instead of the system clock, ensuring synchronization between ROS and the simulation.
* **Launching:** Simulations are typically launched using ROS [[Launch Files]], which start the Gazebo server (`gzserver`), the Gazebo client GUI (`gzclient`), load a world file, spawn robot models (using `spawn_model` node/service), and start necessary ROS bridge nodes (like `robot_state_publisher`).

---

## Use Cases

Gazebo is widely used in the robotics community for:

* **Algorithm Development:** Testing and debugging control algorithms, navigation stacks ([[Navigation]]), [[SLAM]] systems, [[Perception]] algorithms, and manipulation strategies in a controlled and repeatable environment.
* **[[Reinforcement Learning (RL)|RL Training]]:** Providing a fast, safe, and parallelizable environment to train robot control policies using RL, mitigating the high sample complexity and safety concerns of real-world training.
* **Visualization and Debugging:** Visualizing robot models, sensor data, and algorithm outputs within a 3D environment. Complements tools like [[RViz_Tutorial|RViz]].
* **Education and Research:** A standard platform for teaching robotics concepts and conducting research experiments without requiring expensive hardware.
* **System Integration Testing:** Verifying the interaction between different ROS nodes and software components before deploying on a physical robot.

*(Note: Gazebo Sim, formerly Ignition Gazebo, is the next-generation version of the simulator, offering enhanced modularity and performance, and is often the default choice for newer ROS 2 distributions.)*

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #robotics OR #kinematics WHERE contains(file.outlinks, [[Custom_Packages_and_Nodes]])