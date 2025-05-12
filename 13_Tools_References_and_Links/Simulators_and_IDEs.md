---
title: Simulators and IDEs
description: A list of common simulation tools and Integrated Development Environments (IDEs) used in robotics research and development.
tags:
  - simulator
  - IDE
  - software
  - tools
  - development
  - Gazebo
  - ROS
  - VSCode
  - simulation
  - coding
  - references
layout: default
category: robotics
author: Jordan_Smith_&_le_Chat
date: 2025-04-29
permalink: /simulators_and_ides/
related:
  - "[[Simulation]]"
  - "[[Gazebo_Simulator]]"
  - "[[RViz_Tutorial]]"
  - "[[ROS (Robot Operating System)]]"
  - "[[C++]]"
  - "[[Python_for_Robotics]]"
  - "[[Software_Engineering]]"
  - "[[Tools_References_and_Links]]"
  - "[[Resources_Index]]"
  - "[[Sim2Real_Experiments]]"
---

# Simulators and IDEs in Robotics

This note provides a list of commonly used [[Simulation|Simulators]] and Integrated Development Environments (IDEs) relevant to robotics development. These tools are essential for testing algorithms, developing control strategies, visualizing robot behavior, and efficiently writing and debugging code.

See also: [[Resources_Index]], [[Tools_References_and_Links]]

---

## Robotics Simulators

Simulators provide virtual environments to model robots, sensors, and interactions, crucial for development without requiring constant access to physical hardware. They are invaluable for algorithm testing, [[Reinforcement Learning (RL)|RL]] training, and [[Sim2Real_Experiments|Sim2Real]] workflows.

* **[[Gazebo_Simulator]]:** *(Mentioned in [[Resources_Index]] & has dedicated note)*
    * **Description:** A powerful, open-source 3D multi-robot simulator with strong [[ROS (Robot Operating System)|ROS]] and [[ROS_2_Overview|ROS 2]] integration. Widely used in the robotics community.
    * **Features:** Multiple physics engines (ODE, Bullet, DART, Simbody), realistic sensor simulation ([[LIDAR]], [[Camera_Systems]], [[IMU_Sensors]], etc.), extensive robot model library, large user community. Includes Gazebo Classic (older versions) and the newer generation Gazebo Sim (formerly Ignition Gazebo).
    * **Website:** [gazebosim.org](https://gazebosim.org/)
* **Webots:** *(Mentioned in [[Resources_Index]] & Handbook)*
    * **Description:** An open-source, cross-platform desktop application used to simulate robots. Popular in education and research.
    * **Features:** User-friendly interface, accurate physics, large library of robot models and sensors, supports programming in [[C++]], [[Python_for_Robotics]], Java, [[MATLAB]]. Interfaces with ROS/ROS 2.
    * **Website:** [cyberbotics.com](https://cyberbotics.com/)
* **NVIDIA Isaac Sim:** *(Mentioned in edit2.pdf)*
    * **Description:** A scalable robotics simulation application and synthetic data generation tool built on the NVIDIA Omniverse platform. Focuses on photorealistic, physically accurate simulation.
    * **Features:** High-fidelity rendering (ray tracing, path tracing), PhysX 5 physics engine, ROS/ROS 2 integration, GPU acceleration, features for [[Sim2Real_Experiments|Sim2Real]] (domain randomization, synthetic data generation), integrated RL training workflows (Isaac Lab). Requires NVIDIA GPU.
    * **Website:** [developer.nvidia.com/isaac-sim](https://developer.nvidia.com/isaac-sim)
* **CoppeliaSim (formerly V-REP):** *(Mentioned in Handbook)*
    * **Description:** A versatile and scalable general-purpose robot simulator, popular in academia and industry.
    * **Features:** Wide range of built-in models, sensors, and actuators; distributed control architecture; multiple programming options (embedded scripts, APIs); ROS interface.
    * **Website:** [coppeliarobotics.com](https://coppeliarobotics.com/)
* **[[MATLAB]]/Simulink:**
    * **Description:** While primarily tools for numerical computation and model-based design, MATLAB and Simulink offer Robotics System Toolbox and other toolboxes for simulating robot kinematics, dynamics, control systems, and integrating with external simulators like Gazebo.
    * **Features:** Strong in control system design and analysis, algorithm development, data analysis and visualization.
    * **Website:** [mathworks.com/products/robotics](https://mathworks.com/products/robotics)
* **Other Simulators:**
    * **CARLA:** Open-source simulator specifically focused on autonomous driving research.
    * **AirSim:** Open-source simulator from Microsoft for autonomous vehicles, focusing on drones and cars, often used with Unreal Engine or Unity for photorealism.
    * **PyBullet:** Python bindings for the Bullet physics engine, allowing for quick prototyping of physics simulations directly in Python.

*Note: [[RViz_Tutorial|RViz]] is primarily a 3D **visualization** tool for ROS data (sensor streams, robot models, markers), not a dynamic simulator.*

---

## Integrated Development Environments (IDEs)

IDEs provide essential tools for writing, compiling, debugging, and managing robotics software projects, typically involving [[C++]] and/or [[Python_for_Robotics]].

* **Visual Studio Code (VS Code):**
    * **Description:** A free, lightweight, yet powerful and highly extensible source code editor from Microsoft. Extremely popular in the general programming and robotics communities.
    * **Features:** Excellent IntelliSense (code completion, analysis), debugging support for C++ and Python, integrated Git [[Git_Integration_and_Version_Control]], vast marketplace of extensions. Crucially, has well-maintained ROS/ROS 2 extensions for building, running, debugging nodes, visualizing TF trees, interacting with topics/services/actions directly within the IDE.
    * **Website:** [code.visualstudio.com](https://code.visualstudio.com/)
* **CLion:**
    * **Description:** A powerful, commercial cross-platform C++ IDE from JetBrains. Favored by many C++ developers for its deep code understanding and refactoring capabilities.
    * **Features:** Excellent CMake integration (essential for ROS), smart C++ code analysis, powerful debugger (GDB/LLDB integration), integrated version control. Has plugins available for ROS development support. Free licenses often available for students/academics.
    * **Website:** [jetbrains.com/clion/](https://www.jetbrains.com/clion/)
* **Qt Creator:**
    * **Description:** A cross-platform C++ IDE focused on the Qt framework, often used for developing graphical user interfaces (GUIs) in robotics (e.g., `rqt` plugins). Also functions as a capable general C++/CMake IDE.
    * **Features:** Strong GUI design tools, good CMake support, integrated C++ editor and debugger.
    * **Website:** [qt.io/product/development-tools](https://www.qt.io/product/development-tools)
* **Eclipse:**
    * **Description:** A long-standing, open-source IDE primarily known for Java, but with strong C/C++ Development Tooling (CDT) and Python (PyDev) plugins.
    * **Features:** Mature platform, highly extensible, supports CMake. Can be configured for ROS development, though setup might be less streamlined than VS Code or CLion with dedicated ROS plugins.
    * **Website:** [eclipse.org](https://www.eclipse.org/)
* **PyCharm:**
    * **Description:** A commercial Python IDE from JetBrains, considered one of the best for pure Python development.
    * **Features:** Excellent Python code analysis, debugging, testing, virtual environment management, scientific tools integration (Jupyter, Numpy, Matplotlib). Useful for Python-heavy ROS projects or ML components. Free Community Edition available.
    * **Website:** [jetbrains.com/pycharm/](https://www.jetbrains.com/pycharm/)

---

Choosing the right simulator and IDE depends on the specific project requirements, target hardware, programming language preferences, budget, and integration needs, particularly with [[ROS (Robot Operating System)|ROS]].

