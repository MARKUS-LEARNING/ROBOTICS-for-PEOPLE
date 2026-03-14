---
title: Workspace Analysis
description: Workspace Analysis in robotics is the study of the reachable and dexterous regions of a robot manipulator, including workspace volume calculation, manipulability analysis, and singularity identification for workcell design.
tags:
  - robotics
  - kinematics
  - workspace
  - manipulability
  - singularity
  - workcell-design
  - robot-design
  - engineering
  - glossary-term
  - manipulator-arm
layout: default
category: robotics
author: Jordan_Smith_and_le_Chat
date: 2025-05-09
permalink: /workspace_analysis/
related:
  - "[[Kinematics]]"
  - "[[Task_Space_and_Workspace]]"
  - "[[Configuration_Space]]"
  - "[[Jacobian]]"
  - "[[Singularity]]"
  - "[[Robot_Design]]"
  - "[[Inverse_Kinematics]]"
  - "[[Manipulator_Arm]]"
  - "[[Forward_Kinematics]]"
---

# Workspace Analysis

**Workspace Analysis** in robotics is the systematic study of the spatial regions a robot manipulator can reach and operate within. It encompasses the computation of reachable and dexterous workspaces, the evaluation of manipulability throughout the workspace, the identification of singularities, and the practical application of this analysis to workcell layout and robot selection.

---
![image](https://github.com/user-attachments/assets/6544c206-7141-4bad-a5cc-588b5f93eeff)

<font size=1>*source: https://www.roboticsunveiled.com/robotics-task-space-and-workspace/*</font>
---

## Reachable Workspace vs. Dexterous Workspace

### Reachable Workspace

The **reachable workspace** $\mathcal{W}_R$ is the set of all positions the end-effector can reach with at least one orientation:

$$
\mathcal{W}_R = \{ \mathbf{p} \in \mathbb{R}^3 \mid \exists \, \mathbf{q} \in \mathcal{C} \text{ such that } \mathbf{p} = f_p(\mathbf{q}) \}
$$

where $f_p(\mathbf{q})$ extracts the position from the forward kinematics.

### Dexterous Workspace

The **dexterous workspace** $\mathcal{W}_D$ is the subset of the reachable workspace where the end-effector can attain all orientations (or a specified set of orientations):

$$
\mathcal{W}_D = \{ \mathbf{p} \in \mathcal{W}_R \mid \forall \, R \in SO(3), \, \exists \, \mathbf{q} \text{ such that } f(\mathbf{q}) = (\mathbf{p}, R) \}
$$

The dexterous workspace is always a subset of the reachable workspace: $\mathcal{W}_D \subseteq \mathcal{W}_R$. For most 6-DoF robots, the dexterous workspace is significantly smaller than the reachable workspace. A 7-DoF redundant arm (e.g., KUKA iiwa, Franka Emika Panda) has a larger dexterous workspace because the extra DoF provides the flexibility to reconfigure the elbow while maintaining the end-effector pose.

**Practical examples:**
- A UR5e (850 mm reach) has a roughly spherical reachable workspace of radius 850 mm, but the dexterous workspace (where any wrist orientation is achievable) is a much smaller shell.
- For workcell design, the target work surface should lie within the dexterous workspace, not just the reachable workspace, to ensure the robot can approach parts from multiple angles.

---

## Workspace Volume Calculation Methods

### Analytical Methods

For simple serial chains, closed-form workspace boundaries can be derived:

- **Planar 2R arm** (link lengths $l_1$, $l_2$): Workspace is an annulus with inner radius $|l_1 - l_2|$ and outer radius $l_1 + l_2$. Area $= \pi(l_1 + l_2)^2 - \pi(l_1 - l_2)^2 = 4\pi l_1 l_2$.

- **Spatial 3R arm** with concurrent axes: Workspace is a thick spherical shell. The outer boundary is a sphere of radius $l_1 + l_2 + l_3$; the inner boundary depends on joint limits.

### Monte Carlo Sampling

For general robots with arbitrary joint limits, Monte Carlo sampling is the standard numerical approach:

1. Sample $N$ uniformly random configurations: $\mathbf{q}_i \sim \text{Uniform}(\mathbf{q}_{\min}, \mathbf{q}_{\max})$
2. Compute forward kinematics for each: $\mathbf{p}_i = f_p(\mathbf{q}_i)$
3. Discretize the bounding Cartesian volume into voxels of side length $\Delta$
4. Count occupied voxels $n_{\text{occ}}$
5. Estimate workspace volume:

$$
V_{\text{ws}} \approx n_{\text{occ}} \cdot \Delta^3
$$

**Practical guidance:** Use $N \geq 10^6$ samples and $\Delta \leq 10$ mm for reliable estimates. Convergence can be verified by doubling $N$ and checking that the volume estimate changes by less than 1%.

### Workspace Cross-Sections

A useful visualization technique is to plot 2D cross-sections of the workspace at fixed values of one coordinate (e.g., the $z = 0$ plane). This reveals the workspace shape, voids (unreachable regions near the base), and boundary irregularities caused by joint limits.

---

## Manipulability Ellipsoid and Index

### Yoshikawa's Manipulability Index

The **manipulability index** (Yoshikawa, 1985) quantifies how well-conditioned the kinematic mapping is at a given configuration:

$$
w(\mathbf{q}) = \sqrt{\det(J(\mathbf{q}) \, J(\mathbf{q})^T)}
$$

where $J(\mathbf{q})$ is the $m \times n$ Jacobian matrix mapping joint velocities to end-effector velocities.

- $w = 0$ at singular configurations (the robot loses one or more DoF of end-effector motion)
- Higher $w$ indicates the robot can generate end-effector velocities more uniformly in all directions
- $w$ is proportional to the volume of the **manipulability ellipsoid**

### Manipulability Ellipsoid

The set of end-effector velocities $\dot{\mathbf{x}}$ achievable with unit joint velocity ($\| \dot{\mathbf{q}} \| \leq 1$) forms an ellipsoid in task space:

$$
\dot{\mathbf{x}}^T (J J^T)^{-1} \dot{\mathbf{x}} \leq 1
$$

The principal axes of this ellipsoid are the singular vectors of $J$, and their lengths are the singular values $\sigma_1 \geq \sigma_2 \geq \ldots \geq \sigma_m$. The manipulability index equals the product of singular values: $w = \sigma_1 \sigma_2 \cdots \sigma_m$.

### Condition Number

The **condition number** $\kappa = \sigma_{\max} / \sigma_{\min}$ measures the isotropy of the manipulability ellipsoid:

$$
\kappa(J) = \frac{\sigma_1}{\sigma_m}
$$

- $\kappa = 1$: isotropic (equally capable in all directions) -- ideal
- $\kappa \to \infty$: approaching singularity
- **Practical threshold:** Many industrial controllers warn when $\kappa > 50$ and halt motion when $\kappa > 100$

---

## Singularity Analysis Within the Workspace

Singularities occur at configurations where the Jacobian loses rank, i.e., $\det(J J^T) = 0$ (equivalently, $w = 0$).

### Types of Singularities

1. **Boundary singularities**: Occur at the edge of the reachable workspace when the arm is fully extended or fully folded. Example: a 6R arm reaching its maximum extension.

2. **Interior singularities**: Occur within the workspace volume. Example: a PUMA-type arm when the wrist center passes through the axis of joint 1 (shoulder singularity), or when joints 4 and 6 align (wrist singularity).

3. **Wrist singularities**: When the axes of the three wrist joints become coplanar. For a spherical wrist, this occurs when $\theta_5 = 0$ (or $\pi$), causing joints 4 and 6 to become collinear.

### Singularity Mapping

For workcell design, it is valuable to map singularity surfaces within the workspace:

1. Sample configurations densely and compute $w(\mathbf{q})$ for each
2. Identify the iso-surface $w = \epsilon$ (for small $\epsilon$) in task space
3. Plot these surfaces to visualize singularity-prone regions

**Practical consequence:** Trajectories that pass through or near singularities cause joint velocities to spike to infinity. Industrial controllers implement **singularity avoidance** by either:
- Damped least-squares (DLS) inverse kinematics: $\dot{\mathbf{q}} = J^T(J J^T + \lambda^2 I)^{-1} \dot{\mathbf{x}}$
- Trajectory replanning to route around singular configurations

---

## Practical Workspace Mapping for Workcell Design

### Robot Selection

When designing a robotic workcell, workspace analysis drives robot selection:

1. **Define task requirements**: Enumerate all target positions and required approach angles
2. **Check workspace coverage**: Verify all targets lie within the dexterous workspace (not just reachable)
3. **Evaluate manipulability**: Ensure $w > w_{\min}$ at all task-critical locations (a common threshold is $w_{\min} \geq 0.05 \cdot w_{\max}$)
4. **Identify collision-free regions**: Subtract fixture and tool volumes from the workspace

### Robot Placement Optimization

The position and orientation of the robot base relative to the workpiece affects workspace utilization. The optimization problem is:

$$
\max_{\mathbf{p}_{\text{base}}, \theta_{\text{base}}} \min_{i \in \text{targets}} w(\mathbf{q}_i^*)
$$

where $\mathbf{q}_i^*$ is the IK solution for the $i$-th target. This maximizes the worst-case manipulability across all task targets.

### Real-World Workcell Design Checklist

- Ensure a minimum clearance of 50--100 mm between the robot body and any fixture at every configuration along planned paths
- Reserve at least 10% of the reachable workspace as a safety margin for path variations
- Place the primary work surface at approximately the robot's "shoulder height" (roughly 60--70% of maximum reach from the base) for best manipulability
- Verify the workcell layout in simulation (e.g., RoboDK, ROS 2 + MoveIt 2, or vendor tools like ABB RobotStudio or KUKA WorkVisual) before physical installation

---

## Applications in Robotics

- **Workcell Layout**: Workspace analysis is essential for positioning robots, fixtures, conveyors, and safety fencing in manufacturing cells.
  <br>

- **Robot Selection**: Comparing workspace volumes and manipulability maps helps choose the right robot for a given application.
  <br>

- **Trajectory Planning**: Understanding workspace boundaries and singularity locations enables planners to generate feasible, smooth trajectories.
  <br>

- **Redundancy Exploitation**: For redundant robots, workspace analysis reveals how the extra DoF enlarges the dexterous workspace and improves manipulability.
  <br>

- **Collaborative Robotics**: In human-robot collaboration, workspace overlap between the human and robot must be analyzed for safety (ISO 10218, ISO/TS 15066).
  <br>

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #robotics OR #kinematics WHERE contains(file.outlinks, [[Workspace_Analysis]])
```
