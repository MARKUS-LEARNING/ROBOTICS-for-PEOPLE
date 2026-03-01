---
title: URDF
description: Explains the Unified Robot Description Format (URDF), the XML-based standard used in ROS to describe a robot's physical structure, kinematics, dynamics, visual appearance, and collision geometry.
tags:
  - URDF
  - ROS
  - robot-model
  - kinematics
  - robotics-software
  - XML
  - simulation
  - TF
  - xacro
layout: default
category: robotics
author: Jordan_Smith_&_le_Chat
date: 2025-05-02
permalink: /urdf/
related:
  - "[[Robot_Operating_System_(ROS)]]"
  - "[[ROS_2_Overview]]"
  - "[[TF_and_Topic_Architecture]]"
  - "[[Gazebo_Simulator]]"
  - "[[RViz_Tutorial]]"
  - "[[Forward_Kinematics]]"
  - "[[Inverse_Kinematics]]"
  - "[[DH_Parameters]]"
  - "[[Links]]"
  - "[[Joints]]"
  - "[[Launch_Files]]"
  - "[[ros2_control]]"
  - "[[MoveIt2]]"
---

# URDF — Unified Robot Description Format

The **Unified Robot Description Format (URDF)** is an XML-based file format used in the [[Robot_Operating_System_(ROS)|ROS]] ecosystem to describe the physical structure of a robot. A URDF file encodes a robot's complete model: its kinematic chain of rigid **links** connected by **joints**, visual geometry for rendering, collision geometry for physics simulation, and dynamic properties (mass, inertia) for dynamics calculations.

URDF is consumed by core ROS tools including `robot_state_publisher` (which uses the model and `/joint_states` to broadcast [[TF_and_Topic_Architecture|TF frames]]), [[RViz_Tutorial|RViz]] (for 3D visualization), [[Gazebo_Simulator]] (for physics simulation), and [[MoveIt2]] (for motion planning).

---

## Core Concepts

### Links

A **link** represents a single rigid body in the robot's structure — a physical piece such as a base plate, arm segment, or end effector. Each link can specify:

* **`<visual>`**: Geometry and material for rendering (meshes, primitives).
* **`<collision>`**: Simplified geometry used for collision checking (often simpler than visual).
* **`<inertial>`**: Mass and inertia tensor for dynamics simulation.

### Joints

A **joint** connects two links, defining the relative motion allowed between them. Key joint types:

| Type | Description |
|---|---|
| `revolute` | Rotation about a single axis, with defined limits (e.g., elbow) |
| `continuous` | Unlimited rotation about a single axis (e.g., wheel) |
| `prismatic` | Linear sliding along a single axis (e.g., linear actuator) |
| `fixed` | No relative motion; rigidly connects two links |
| `floating` | Unconstrained 6-DOF (rarely used in standard URDF) |
| `planar` | Motion in a plane perpendicular to the axis |

Each joint specifies a **parent link**, a **child link**, an **origin** (the transform from parent to joint frame), and an **axis** of motion.

---

## URDF Structure and Syntax

A minimal two-link robot arm in URDF:

```xml
<?xml version="1.0"?>
<robot name="simple_arm">

  <!-- Base link (world anchor) -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0"
               iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- First arm segment -->
  <link name="upper_arm">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0"
               iyy="0.005" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Shoulder joint connecting base to upper arm -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1.0"/>
    <dynamics damping="0.1" friction="0.05"/>
  </joint>

</robot>
```

### Geometry Types

URDF supports both **primitive shapes** and **mesh files**:

```xml
<!-- Primitives -->
<geometry><box size="x y z"/></geometry>
<geometry><cylinder radius="r" length="l"/></geometry>
<geometry><sphere radius="r"/></geometry>

<!-- Mesh (STL or DAE/Collada) -->
<geometry>
  <mesh filename="package://my_robot/meshes/upper_arm.stl" scale="0.001 0.001 0.001"/>
</mesh>
</geometry>
```

### Origin Transform

The `<origin>` tag specifies the transform (position + orientation) from the parent frame to the child frame or geometry origin. Orientation is expressed as roll-pitch-yaw (`rpy`):

```xml
<origin xyz="0.1 0.0 0.05" rpy="0 0 1.5708"/>
```

This corresponds to the homogeneous transform:

$$T = \begin{bmatrix} R_{rpy} & \mathbf{p} \\ \mathbf{0}^T & 1 \end{bmatrix}$$

where $\mathbf{p} = [0.1, 0.0, 0.05]^T$ and $R_{rpy}$ is the rotation from roll, pitch, yaw angles.

---

## Inertial Properties

Accurate inertial properties are critical for dynamics simulation in Gazebo. For a link with mass $m$ and inertia tensor $\mathbf{I}$:

$$\mathbf{I} = \begin{bmatrix} I_{xx} & I_{xy} & I_{xz} \\ I_{xy} & I_{yy} & I_{yz} \\ I_{xz} & I_{yz} & I_{zz} \end{bmatrix}$$

For common primitives (computed about the center of mass):

| Shape | $I_{xx}$ | $I_{yy}$ | $I_{zz}$ |
|---|---|---|---|
| Solid cylinder (axis z, radius $r$, length $h$) | $\frac{m}{12}(3r^2 + h^2)$ | $\frac{m}{12}(3r^2 + h^2)$ | $\frac{mr^2}{2}$ |
| Solid box ($x \times y \times z$) | $\frac{m}{12}(y^2 + z^2)$ | $\frac{m}{12}(x^2 + z^2)$ | $\frac{m}{12}(x^2 + y^2)$ |
| Solid sphere (radius $r$) | $\frac{2mr^2}{5}$ | $\frac{2mr^2}{5}$ | $\frac{2mr^2}{5}$ |

---

## Xacro — Macro Language for URDF

Raw URDF becomes verbose and repetitive for complex robots. **Xacro** (XML Macros) is a preprocessor that extends URDF with:

* **Properties** (constants): Avoid magic numbers.
* **Macros**: Reusable link/joint templates (e.g., for repeated finger joints).
* **Math expressions**: Computed values.
* **`include`**: Split models across multiple files.

```xml
<?xml version="1.0"?>
<robot name="my_arm" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Define a constant -->
  <xacro:property name="link_length" value="0.3"/>
  <xacro:property name="link_radius" value="0.025"/>

  <!-- Define a reusable macro -->
  <xacro:macro name="arm_link" params="name mass">
    <link name="${name}">
      <visual>
        <geometry>
          <cylinder radius="${link_radius}" length="${link_length}"/>
        </geometry>
      </visual>
      <inertial>
        <mass value="${mass}"/>
        <inertia ixx="${mass * (3 * link_radius**2 + link_length**2) / 12}"
                 ixy="0" ixz="0"
                 iyy="${mass * (3 * link_radius**2 + link_length**2) / 12}"
                 iyz="0"
                 izz="${mass * link_radius**2 / 2}"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Instantiate the macro -->
  <xacro:arm_link name="upper_arm" mass="0.5"/>
  <xacro:arm_link name="lower_arm" mass="0.4"/>

</robot>
```

Processing a xacro file to URDF:

```bash
xacro robot.urdf.xacro > robot.urdf
# Or load directly in a launch file:
ros2 param set /robot_description "$(xacro /path/to/robot.urdf.xacro)"
```

---

## Gazebo Extensions

To use a URDF in [[Gazebo_Simulator|Gazebo]], additional Gazebo-specific tags are added within `<gazebo>` elements:

```xml
<!-- Material override for Gazebo rendering -->
<gazebo reference="base_link">
  <material>Gazebo/Grey</material>
</gazebo>

<!-- Attach a sensor plugin -->
<gazebo reference="laser_link">
  <sensor type="ray" name="lidar">
    <ray>
      <scan><horizontal><samples>360</samples><min_angle>-3.14</min_angle>
            <max_angle>3.14</max_angle></horizontal></scan>
      <range><min>0.1</min><max>30.0</max></range>
    </ray>
    <plugin name="gazebo_ros_laser" filename="libgazebo_ros_laser.so">
      <topicName>/scan</topicName>
      <frameName>laser_link</frameName>
    </plugin>
  </sensor>
</gazebo>

<!-- ros2_control hardware interface -->
<ros2_control name="my_arm" type="system">
  <hardware>
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
  </hardware>
  <joint name="shoulder_joint">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

---

## Robot State Publisher

The `robot_state_publisher` node is the primary consumer of URDF in a running ROS system. It:

1. Reads the URDF from the `/robot_description` parameter.
2. Subscribes to `/joint_states` (published by the hardware driver or simulator).
3. Uses [[Forward_Kinematics|forward kinematics]] (via the KDL library) to compute the transform of each link.
4. Broadcasts all link transforms to the [[TF_and_Topic_Architecture|TF tree]].

```bash
# Launch robot_state_publisher with a URDF file
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(xacro /path/to/robot.urdf.xacro)"
```

---

## Validation and Inspection

```bash
# Check URDF for errors
check_urdf my_robot.urdf

# Display the kinematic tree structure
urdf_to_graphviz my_robot.urdf && evince my_robot.pdf

# View in RViz
ros2 launch urdf_launch display.launch.py urdf_package:=my_robot urdf_package_path:=urdf/robot.urdf
```

---

## Dataview Plugin Features

```dataview
LIST FROM #URDF OR #robot-model WHERE contains(file.outlinks, [[URDF]])
```

```dataview
TABLE title, description FROM #ROS WHERE contains(file.tags, "URDF")
```
