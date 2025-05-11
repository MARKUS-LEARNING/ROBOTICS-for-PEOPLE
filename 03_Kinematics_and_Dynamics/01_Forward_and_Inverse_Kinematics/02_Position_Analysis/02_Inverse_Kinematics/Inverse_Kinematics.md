---
title: Inverse Kinematics (IK)
description: "Describes Inverse Kinematics (IK): Computing the joint variables required for a robot's end-effector to reach a desired Cartesian pose."
tags:
  - kinematics
  - manipulator
  - control
  - mapping
  - joint-space
  - cartesian-space
layout: default
category: robotics
author: Jordan_Smith_&_le_Chat
date: 2025-04-28
permalink: /inverse_kinematics/
related:
  - "[[Kinematics]]"
  - "[[Forward_Kinematics]]"
  - "[[Workspace]]"
  - "[[Singularities]]"
  - "[[Degrees_of_Freedom]]"
  - "[[Joint Space]]"
  - "[[Cartesian Space]]"
  - "[[Manipulator_Arm_Types]]"
  - "[[Jacobian_Matrix]]"
  - "[[ROBOTICS-for-EVERYONE/03_Kinematics_and_Dynamics/03_Trajectory_Planning/Configuration_Space]]"
---

# Inverse Kinematics (IK)

**Inverse Kinematics (IK)** is the process of calculating the vector of joint parameters $\mathbf{q}$ (joint angles for revolute joints, joint displacements for prismatic joints) required for the end-effector of a robot (or any other point of interest) to achieve a desired pose (position and orientation) in the task space (usually [[Cartesian_Space|Cartesian Space]]). The desired pose is typically specified as a [[Homogeneous Transformation Matrix]] representing the target frame {T} relative to the base frame {0}, denoted as $^0T_T$.

Essentially, inverse kinematics solves the problem: **Given a desired end-effector pose, what are the joint configurations that achieve it?** This represents a mapping from the Cartesian space back to the robot's [[Joints|Joint Space]]. IK is the inverse problem to [[Forward_Kinematics|Forward Kinematics]] (which calculates the end-effector pose from given joint values).

---

## Challenges and Characteristics

Solving the inverse kinematics problem is generally much more complex than solving the forward kinematics problem due to several factors:

* **Nonlinearity:** The forward kinematic equations relating joint variables to end-effector pose are inherently nonlinear and often involve transcendental trigonometric functions. Solving the inverse involves finding roots of these complex equation systems.
* **Solvability (Existence):** A solution to the IK problem may not exist if the desired end-effector pose lies outside the robot's [[Workspace]]. Determining reachability is part of the problem.
* **Multiple Solutions:** Unlike FK (which typically yields a unique pose for a given joint configuration in serial arms), IK often yields multiple valid joint configurations ($\mathbf{q}$) that result in the exact same end-effector pose. For example, a common robot arm might have "elbow-up" and "elbow-down" solutions. A general 6R manipulator can have up to 16 distinct solutions. The number of solutions depends on the robot's [[Kinematic_Chains|kinematic structure]] and [[Degrees_of_Freedom|degrees of freedom (DoF)]].
* **Singularities:** Near or at [[Singularities|singular configurations]], solutions may become degenerate, infinite, or non-existent, requiring special handling.

---

## Solution Methods

Because IK involves solving nonlinear equations, various methods have been developed:

### Closed-Form (Analytical) Solutions

These solutions provide explicit mathematical equations (algebraic or trigonometric) that directly calculate *all* possible joint configurations $\mathbf{q}$ for a given desired pose $^0T_T$.

* **Advantages:** Computationally very fast (essential for real-time control), guaranteed to find all existing solutions.
* **Limitations:** Only exist for specific, kinematically simple robot structures. Most industrial manipulators are designed to allow closed-form solutions.
* **Enabling Structures:** Sufficient conditions often involve kinematic decoupling, such as:
    * **Spherical Wrist:** Three consecutive revolute joint axes intersecting at a common point (the wrist center). This allows separating the IK problem into finding the first three joints to position the wrist center (inverse position kinematics) and then finding the last three joints to orient the end-effector (inverse orientation kinematics).
    * Three consecutive parallel revolute axes.
* **Techniques:**
    * **Algebraic:** Manipulate the FK matrix equation ($^0T_N(q) = {^0T_T}_{desired} (^N T_T)^{-1}$) to isolate joint variables, often reducing the problem to solving polynomial equations (using substitutions like $u_i = \tan(\theta_i/2)$).
    * **Geometric:** Decompose the spatial geometry problem into simpler planar trigonometry problems (e.g., finding wrist position using triangles, then solving wrist orientation using Euler angles).

### Numerical (Iterative) Solutions

These methods use iterative algorithms to converge to *a* solution for $\mathbf{q}$, starting from an initial guess $\mathbf{q}_{guess}$.

* **Advantages:** Applicable to robots with general kinematic structures where closed-form solutions are not available.
* **Disadvantages:** Computationally more expensive, convergence depends on the initial guess, may only find one solution (often the one "closest" to the guess), can get stuck in local minima, may fail to converge, requires careful handling near [[Singularities]].
* **Techniques:**
    * **Jacobian-Based:** Use the [[Jacobian_Matrix]] $J(\mathbf{q})$ to relate small changes in end-effector pose $\Delta \mathbf{x}$ to small changes in joint variables $\Delta \mathbf{q}$ ($\Delta \mathbf{x} \approx J(\mathbf{q}) \Delta \mathbf{q}$). The error between the current pose (found via FK from the current $\mathbf{q}$) and the desired pose is used to compute a corrective $\Delta \mathbf{q}$ using the Jacobian inverse ($J^{-1}$) or pseudoinverse ($J^{\dagger}$). Examples include Newton-Raphson methods and Resolved Motion Rate Control. Singularity robustness techniques (e.g., Damped Least Squares) are often needed.
    * **Optimization-Based:** Formulate IK as minimizing an error function, e.g., the distance between the current end-effector pose and the desired pose, using numerical optimization algorithms.

---

## Applications

Inverse kinematics is crucial for almost any task where the goal is specified in terms of the end-effector's position and orientation in the world, including:

* **Task Planning:** Defining goal configurations for tasks like grasping, insertion, welding, painting, etc.
* **Trajectory Tracking:** Calculating the required joint positions/velocities/accelerations at each time step to make the end-effector follow a desired Cartesian path.
* **Sensor-Based Control:** Moving the robot to Cartesian coordinates provided by vision systems or other sensors.
* **Teleoperation:** Mapping operator commands (often in Cartesian space) to robot joint commands.

---

## Mobile Robots

For mobile robots, "inverse kinematics" typically refers to the calculation of the necessary control inputs (e.g., wheel velocities) to achieve a desired instantaneous chassis velocity (linear and angular velocity). For common configurations like [[Differential_Drive|Differential Drive]], this IK problem is usually much simpler and has a direct analytical solution.

---
```dataview
LIST FROM #robotics OR #kinematics WHERE contains(file.outlinks, [[Inverse_Kinematics]])