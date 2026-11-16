---
title: URDF
description: "URDF (Unified Robot Description Format) is the XML file that tells ROS what a robot is made of: links, joints, shapes, masses and limits. Covers the format with a worked example whose inertias are computed, Xacro, the URDF 1.2 additions in ROS 2 Lyrical (quaternions, capsules, acceleration and jerk limits), modern Gazebo integration, and how URDF relates to SDF, MJCF and OpenUSD in simulation and robot learning."
aliases:
  - Unified Robot Description Format
  - Xacro
tags:
  - urdf
  - ros
  - robot-model
  - kinematics
  - robotics-software
  - xml
  - simulation
  - tf
  - xacro
type: tool
level: intermediate
subfield:
  - software
status: reviewed
author: Jordan_Smith
date: 2025-04-28
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[ROS_2_Overview]]"
  - "[[Forward_Kinematics]]"
related:
  - "[[TF_and_Topic_Architecture]]"
  - "[[Gazebo_Simulator]]"
  - "[[MuJoCo]]"
  - "[[ros2_control]]"
  - "[[MoveIt2]]"
  - "[[RViz_Tutorial]]"
key_takeaways:
  - "A URDF is a tree of links (rigid bodies with visual, collision and inertial data) joined by joints (revolute, continuous, prismatic, fixed, planar, floating)."
  - "Wrong inertias and oversized collision meshes are the most common reasons a model that looks right behaves wrongly in simulation; compute inertias from geometry and mass, never copy placeholders."
  - "URDF 1.2 (ROS 2 Lyrical, 2026) adds quaternion orientations, capsule shapes and acceleration, deceleration and jerk limits; opt in with version=\"1.2\"."
sources:
  - "Open Robotics (2026). urdfdom: URDF parser and XML specification. https://github.com/ros/urdfdom"
  - "Open Robotics (2026). ROS 2 Lyrical Luth release notes: URDF improvements. https://github.com/ros2/ros2_documentation/blob/rolling/source/Releases/Release-Lyrical-Luth.rst"
  - "Open Robotics (2026). xacro: XML macro language. https://github.com/ros/xacro"
  - "Open Robotics (2026). ros_gz: integration between ROS and Gazebo. https://github.com/gazebosim/ros_gz"
  - "Google DeepMind (2026). MuJoCo Menagerie: a collection of high-quality simulation models. https://github.com/google-deepmind/mujoco_menagerie"
---

# URDF — Unified Robot Description Format

> **Etymology.** *Unified* because one file serves every tool: the same description drives visualisation, kinematics, planning and simulation. The format was created at Willow Garage around 2009 for the PR2 robot. *Xacro* is short for *XML macros*.

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
      <!-- solid cylinder r=0.05, h=0.1: Ixx=Iyy=m(3r²+h²)/12, Izz=mr²/2 -->
      <inertia ixx="0.00146" ixy="0.0" ixz="0.0"
               iyy="0.00146" iyz="0.0" izz="0.00125"/>
    </inertial>
  </link>

  <!-- First arm segment -->
  <link name="upper_arm">
    <!-- the box's centre sits 0.15 m up the link, so it hangs from the shoulder -->
    <visual>
      <origin xyz="0 0 0.15"/>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.15"/>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0.15"/>
      <mass value="0.5"/>
      <!-- solid box 0.05 x 0.05 x 0.3: Ixx=Iyy=m(y²+z²)/12, Izz=m(x²+y²)/12 -->
      <inertia ixx="0.00385" ixy="0.0" ixz="0.0"
               iyy="0.00385" iyz="0.0" izz="0.000208"/>
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
</geometry>
<!-- scale 0.001 converts a CAD mesh exported in millimetres to metres -->
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

**Worked example (the arm above).** The upper arm is a 0.5 kg box, 0.05 × 0.05 × 0.3 m:

$$I_{xx} = I_{yy} = \tfrac{0.5}{12}(0.05^2 + 0.3^2) = \tfrac{0.5}{12}(0.0925) = 0.00385\ \text{kg·m}^2, \qquad I_{zz} = \tfrac{0.5}{12}(0.05^2+0.05^2) = 0.000208\ \text{kg·m}^2$$

These are about the link's centre of mass, which is why `<inertial>` carries `<origin xyz="0 0 0.15"/>`. Placeholder values such as 0.01 everywhere make the arm 2.6 times "heavier" to rotate than it is, so a simulated controller tuned on it will be badly tuned on the real robot. CAD tools (Onshape, SolidWorks, Fusion) export exact inertias; for primitives, compute them.

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
# Most launch files instead expand the xacro and pass the text to robot_state_publisher:
#   robot_description = Command(['xacro ', PathJoinSubstitution([pkg, 'urdf', 'robot.urdf.xacro'])])
```

---

## Simulation tags: modern Gazebo and ros2_control

Simulator-specific settings live inside `<gazebo>` elements, and the hardware interface inside a `<ros2_control>` element. With **modern Gazebo** (Harmonic for Jazzy, Jetty for Lyrical) the plugin names differ from the old Gazebo Classic tutorials (`libgazebo_ros_*.so`, `gazebo_ros2_control/GazeboSystem`), which no longer apply because Gazebo Classic reached end of life in January 2025:

```xml
<!-- A 2D lidar simulated by Gazebo; ros_gz_bridge then carries /scan into ROS -->
<gazebo reference="laser_link">
  <sensor name="lidar" type="gpu_lidar">
    <topic>scan</topic>
    <update_rate>10</update_rate>
    <gz_frame_id>laser_link</gz_frame_id>
    <lidar>
      <scan><horizontal><samples>360</samples>
        <min_angle>-3.14159</min_angle><max_angle>3.14159</max_angle></horizontal></scan>
      <range><min>0.1</min><max>30.0</max></range>
    </lidar>
  </sensor>
</gazebo>

<!-- ros2_control: the same controllers run in simulation and on hardware -->
<ros2_control name="my_arm" type="system">
  <hardware>
    <plugin>gz_ros2_control/GazeboSimSystem</plugin>   <!-- on the robot: the vendor's hardware plugin -->
  </hardware>
  <joint name="shoulder_joint">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>

<gazebo>
  <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
    <parameters>$(find my_arm)/config/controllers.yaml</parameters>
  </plugin>
</gazebo>
```

The sensor needs the Gazebo `Sensors` system loaded in the world, and a `ros_gz_bridge` entry to appear as a ROS topic. See [[Gazebo_Simulator]] and [[ros2_control]].

---

## Robot State Publisher

The `robot_state_publisher` node is the primary consumer of URDF in a running ROS system. It:

1. Reads the URDF from its `robot_description` parameter and republishes it on the `/robot_description` topic (from Lyrical it can instead *subscribe* to that topic, with `use_robot_description_topic:=true`, so another framework can own the model).
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

## URDF 1.2: what Lyrical added (2026)

Add `version="1.2"` to the `<robot>` tag to use three new features:

```xml
<robot name="capsule_arm" version="1.2">
  <link name="link1">
    <collision>
      <origin xyz="0 0 0.25" quat_xyzw="0 0 0 1"/>      <!-- quaternion instead of rpy -->
      <geometry><capsule radius="0.1" length="0.5"/></geometry>
    </collision>
  </link>
  <joint name="joint1" type="revolute">
    <origin xyz="0 0 0.5" quat_xyzw="0 0.7071068 0 0.7071068"/>   <!-- 90° about y -->
    <limit lower="-1.57" upper="1.57" effort="100" velocity="2.0"
           acceleration="10.0" deceleration="5.0" jerk="50.0"/>
  </joint>
</robot>
```

- **Quaternions** (`quat_xyzw`) avoid roll-pitch-yaw singularities and rounding when converting from CAD.
- **Capsules** are the cheapest smooth collision shape and the default in most learning simulators.
- **Acceleration, deceleration and jerk limits** let planners and controllers read motion limits from the model instead of separate configuration files.

At release, RViz's Robot Model display did not yet draw capsules.

## URDF and the other robot formats

| Format | From | Strength | Limits |
|---|---|---|---|
| **URDF** | ROS (2009) | Universal in ROS; every arm vendor ships one | Tree only (no closed loops); no world, lights or sensors beyond extensions |
| **SDFormat (SDF)** | Gazebo | Whole worlds, sensors, closed chains | Mainly the Gazebo ecosystem |
| **MJCF** | [[MuJoCo]] | Contact, actuator and tendon detail for fast, accurate simulation | MuJoCo-specific |
| **OpenUSD** | Pixar, now the Alliance for OpenUSD | Scenes, materials, large assets; Isaac Sim's native format | Robot physics schemas still maturing |

**In robot learning (2026).** A typical pipeline starts from the manufacturer's URDF and converts it to MJCF or USD for GPU simulation (see [[Isaac_Lab]], [[MuJoCo]]). It then tunes masses, friction, armature and actuator models against the real robot (see [[Sim-to-Real_Transfer]]). Curated, tested models such as **MuJoCo Menagerie** save weeks of this work. Keep the URDF as the single source of truth for kinematics, so the simulated policy and the real robot agree on joint names, zero positions and limits.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[URDF]])
```
