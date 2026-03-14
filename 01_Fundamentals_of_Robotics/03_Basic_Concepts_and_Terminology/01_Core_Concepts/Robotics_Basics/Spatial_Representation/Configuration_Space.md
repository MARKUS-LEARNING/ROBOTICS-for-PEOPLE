---
title: Configuration Space
description: Configuration Space is a mathematical representation of all possible configurations a robotic system can assume, essential for motion planning and control in robotics.
tags:
  - robotics
  - kinematics
  - motion-planning
  - configuration-space
  - robot-design
  - engineering
  - glossary-term
  - manipulator-arm
  - mobile-robot
layout: default
category: robotics
author: Jordan_Smith_and_le_Chat
date: 2025-05-02
permalink: /configuration_space/
related:
  - "[[Kinematics]]"
  - "[[Motion_Planning]]"
  - "[[Workspace_Analysis]]"
  - "[[Obstacle_Avoidance]]"
  - "[[Path_Planning]]"
  - "[[Robot_Design]]"
---

# Configuration Space

**Configuration Space** is a mathematical representation of all possible configurations a robotic system can assume. It is essential for motion planning and control in robotics, as it provides a framework for analyzing the feasible configurations of a robot and planning its movements within an environment. The configuration space includes all the possible positions and orientations of the robot's components, such as joint angles and end-effector positions.

---
![image](https://github.com/user-attachments/assets/4b89a986-1b0a-47e5-83b1-0128672ad117)

<font size=1>*source: https://www.researchgate.net/figure/Graphical-presentation-of-the-robot-configuration-and-operational-space-as-well-as-the_fig3_276557012*</font>

![image](https://github.com/user-attachments/assets/6e41f78a-468c-4619-a038-380aeb29e01b)

<font size=1>*source: https://www2.math.upenn.edu/~subhrabh/html_cache/c1de2a5f631b57bb62bc5d4335c962ef.html*</font>
---

## Key Concepts

### Configuration

A configuration of a robotic system refers to a specific arrangement of its components, typically described by a set of parameters such as joint angles or positions. The configuration space encompasses all possible configurations the system can achieve.

### Degrees of Freedom

Degrees of freedom (DoF) represent the number of independent parameters required to define the configuration of a robotic system. The configuration space is defined by these degrees of freedom, which determine the system's ability to move and interact with its environment.

### Workspace

The workspace is the physical space within which a robotic system can operate. It is a projection of the configuration space into the Cartesian space, representing the reachable positions and orientations of the robot's end-effector or other critical points.

### Obstacle Space

The obstacle space represents the regions within the configuration space that are occupied by obstacles. Identifying and avoiding these regions is crucial for safe and efficient motion planning.

---

## Mathematical Formulation

### Configuration Representation

The configuration of a robotic system with $n$ degrees of freedom can be represented as a vector in the configuration space:

$$
\mathbf{q} = \begin{bmatrix} q_1 \\ q_2 \\ \vdots \\ q_n \end{bmatrix}
$$

where $q_i$ represents the $i$-th degree of freedom, such as a joint angle or position.

### Configuration Space

The configuration space $\mathcal{C}$ is the set of all possible configurations $\mathbf{q}$:

$$
\mathcal{C} = \{ \mathbf{q} \in \mathbb{R}^n \mid \text{subject to constraints} \}
$$

### Workspace Mapping

The mapping from the configuration space to the workspace is given by the forward kinematics function $f(\mathbf{q})$:

$$
\mathbf{x} = f(\mathbf{q})
$$

where $\mathbf{x}$ represents the position and orientation of the end-effector or another point of interest in the workspace.

### Example: Robotic Arm

Consider a robotic arm with three revolute joints. The configuration space is defined by the joint angles $\mathbf{q} = [q_1, q_2, q_3]^T$. The workspace is the set of all reachable positions and orientations of the end-effector, calculated using the forward kinematics:

$$
\mathbf{x} = f(\mathbf{q})
$$

For motion planning, the configuration space is analyzed to find a collision-free path from an initial configuration $\mathbf{q}_{\text{init}}$ to a goal configuration $\mathbf{q}_{\text{goal}}$.

---

## C-Space Obstacle Formulation

The configuration space is partitioned into two fundamental regions:

**C-space obstacle** ($\mathcal{C}_{\text{obs}}$): The set of all configurations where the robot body intersects with any workspace obstacle:

$$
\mathcal{C}_{\text{obs}} = \{ \mathbf{q} \in \mathcal{C} \mid \mathcal{A}(\mathbf{q}) \cap \mathcal{O} \neq \emptyset \}
$$

where $\mathcal{A}(\mathbf{q})$ is the robot body at configuration $\mathbf{q}$, and $\mathcal{O}$ is the set of workspace obstacles.

**Free configuration space** ($\mathcal{C}_{\text{free}}$): The complement of the obstacle space within the configuration space:

$$
\mathcal{C}_{\text{free}} = \mathcal{C} \setminus \mathcal{C}_{\text{obs}}
$$

The motion planning problem then reduces to finding a continuous path $\tau: [0, 1] \rightarrow \mathcal{C}_{\text{free}}$ such that $\tau(0) = \mathbf{q}_{\text{init}}$ and $\tau(1) = \mathbf{q}_{\text{goal}}$.

**Practical note:** Computing $\mathcal{C}_{\text{obs}}$ explicitly is computationally expensive (exponential in DoF). For robots with more than 3 DoF, it is almost never computed directly. Instead, motion planners use **collision checking** to test individual configurations or path segments against workspace obstacles.

---

## Dimensionality of Common Robots

The dimension of the configuration space equals the number of independent degrees of freedom:

| Robot Type | DoF | C-Space Dimension | C-Space Topology |
|---|---|---|---|
| Planar mobile robot (x, y, heading) | 3 | 3 | $\mathbb{R}^2 \times S^1$ |
| Quadrotor UAV (6-DoF rigid body) | 6 | 6 | $\mathbb{R}^3 \times SO(3)$ |
| SCARA robot (4-axis) | 4 | 4 | $T^3 \times \mathbb{R}$ |
| Industrial 6R arm (e.g., UR5, KUKA iiwa) | 6 | 6 | $T^6$ (6-torus) |
| Redundant 7-DoF arm (e.g., Franka Emika Panda) | 7 | 7 | $T^7$ |
| Humanoid upper body (dual 7-DoF arms + torso) | 15+ | 15+ | High-dimensional torus |
| Mobile manipulator (base + 6R arm) | 9 | 9 | $\mathbb{R}^2 \times S^1 \times T^6$ |

Here $T^n$ denotes the $n$-dimensional torus (product of $n$ circles $S^1$), reflecting revolute joints with $[0, 2\pi)$ range, and $SO(3)$ is the rotation group.

**Key insight:** As dimensionality grows, the volume of C-space grows exponentially. A 6-DoF robot with 360 discrete angles per joint has $360^6 \approx 2.2 \times 10^{15}$ configurations -- grid-based planning becomes infeasible beyond about 4 DoF.

---

## Connection to Motion Planning Algorithms

The structure of the configuration space directly determines which planning algorithms are applicable:

### Sampling-Based Planners

For robots with $\geq 4$ DoF, sampling-based methods are the standard approach because they avoid explicit construction of $\mathcal{C}_{\text{obs}}$:

- **Probabilistic Roadmap (PRM)**: Builds a graph by randomly sampling configurations in $\mathcal{C}_{\text{free}}$, connecting nearby collision-free samples with edges. Suited for multi-query scenarios (e.g., many pick-place targets in a fixed environment). Requires a collision checker and a local planner.

- **Rapidly-exploring Random Tree (RRT)**: Incrementally grows a tree from $\mathbf{q}_{\text{init}}$ toward random samples, biasing exploration toward unexplored regions. Suited for single-query planning. **RRT\*** adds asymptotic optimality via rewiring.

### Combinatorial Planners

For low-dimensional C-spaces (2--3D), exact methods are feasible:

- **Visibility graphs**: Optimal for 2D point robots among polygonal obstacles.
- **Cell decomposition**: Partitions $\mathcal{C}_{\text{free}}$ into convex cells; plans by connecting adjacent cells.

### Optimization-Based Planners

- **CHOMP / TrajOpt**: Treat the path as a trajectory in C-space and minimize a cost functional that penalizes collisions and path length using gradient descent. Fast for refinement but can get stuck in local minima.

**Practical recommendation:** For industrial 6--7 DoF arms, MoveIt 2 (ROS 2) uses OMPL's implementation of RRT-Connect as the default planner. It typically finds feasible paths in under 1 second for moderately cluttered environments.

---

## Applications in Robotics

- **Motion Planning**: Configuration space is used to plan collision-free paths for robotic systems, ensuring they can navigate their environment safely and efficiently.
- **Obstacle Avoidance**: By identifying and avoiding obstacle regions in the configuration space, robots can operate in cluttered environments without collisions.
- **Manipulation**: Understanding the configuration space allows for precise control of robotic manipulators, enabling them to reach and manipulate objects within their workspace.
- **Path Planning**: Configuration space analysis is crucial for planning optimal paths for mobile robots and other systems, considering constraints and obstacles.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #robotics OR #kinematics WHERE contains(file.outlinks, [[Configuration_Space]])
