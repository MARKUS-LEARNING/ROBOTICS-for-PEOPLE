---
title: Launch Files
description: "How ROS 2 starts a whole robot with one command: launch files in Python, XML or YAML that start nodes, pass parameters, remap topics, include other launch files and react to events. Covers the three formats side by side, parameters and remapping, the 2025–2026 additions (path concatenation, per-message log levels, string-join and path-join substitutions), and how ROS 1 roslaunch maps across."
aliases:
  - ros2 launch
  - Launch File
tags:
  - ros
  - ros2
  - launch
  - robotics-software
  - configuration
  - software-stack
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
related:
  - "[[Custom_Packages_and_Nodes]]"
  - "[[Gazebo_Simulator]]"
  - "[[URDF]]"
  - "[[Nav2_Navigation]]"
key_takeaways:
  - "A launch file describes a running system: which nodes start, with which parameters, names, namespaces and remappings, and what happens when one exits."
  - "ROS 2 accepts Python, XML and YAML launch files; Python allows logic, XML and YAML are shorter and easier to review, and all three can include each other."
  - "Kilted (2025) and Lyrical (2026) made XML and YAML launch files more capable (string-join and path-join substitutions, log actions with severity), narrowing the reasons to reach for Python."
sources:
  - "Open Robotics (2026). ros2/launch: tools for launching multiple processes. https://github.com/ros2/launch"
  - "Open Robotics (2026). ros2/launch_ros: ROS-specific launch actions. https://github.com/ros2/launch_ros"
  - "Open Robotics (2026). ROS 2 Lyrical Luth release notes. https://github.com/ros2/ros2_documentation/blob/rolling/source/Releases/Release-Lyrical-Luth.rst"
  - "Open Robotics (2025). ROS 2 Kilted Kaiju release notes. https://github.com/ros2/ros2_documentation/blob/rolling/source/Releases/Release-Kilted-Kaiju.rst"
  - "Open Robotics (2026). Using XML, YAML, and Python for ROS Launch Files. https://github.com/ros2/ros2_documentation/blob/rolling/source/Developer-Tools/Launch/Launch-file-different-formats.rst"
---

# Launch Files

A **launch file** starts and configures a whole robot system with one command. A real robot runs tens of [[Custom_Packages_and_Nodes|nodes]]: drivers, state estimation, planners, controllers, visualisation. Each needs parameters, a name, perhaps a namespace and remapped topics. Typing that by hand is error-prone; a launch file makes it reproducible and reviewable.

> **Etymology.** To *launch* is from Old French *lancier*, "to throw a lance": to set something going with force. In software it has meant "start a program" since the 1980s.

## What a launch file can do

- Start nodes (and any other process) with parameters, names, namespaces and remappings.
- Declare **arguments** (`use_sim_time:=true`, `robot:=arm_left`) so one file serves many setups.
- **Include** other launch files, so a robot's bring-up is assembled from sensor, control and navigation pieces.
- Apply **conditions** (start Gazebo only in simulation) and **event handlers** (shut everything down if the driver dies, spawn controllers after the robot appears).
- Load **composable nodes** into one process for zero-copy communication.

## The same launch in three formats

Start a turtlesim node with a parameter and a remapping:

```python
# turtle.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bg_red', default_value='69'),
        Node(package='turtlesim', executable='turtlesim_node', name='sim', namespace='t1',
             parameters=[{'background_r': LaunchConfiguration('bg_red')}],
             remappings=[('/t1/turtle1/cmd_vel', '/cmd_vel')]),
    ])
```

```xml
<!-- turtle.launch.xml -->
<launch>
  <arg name="bg_red" default="69"/>
  <node pkg="turtlesim" exec="turtlesim_node" name="sim" namespace="t1">
    <param name="background_r" value="$(var bg_red)"/>
    <remap from="/t1/turtle1/cmd_vel" to="/cmd_vel"/>
  </node>
</launch>
```

```yaml
# turtle.launch.yaml
launch:
- arg: {name: bg_red, default: "69"}
- node:
    pkg: turtlesim
    exec: turtlesim_node
    name: sim
    namespace: t1
    param: [{name: background_r, value: "$(var bg_red)"}]
    remap: [{from: /t1/turtle1/cmd_vel, to: /cmd_vel}]
```

Run any of them the same way: `ros2 launch my_pkg turtle.launch.xml bg_red:=200`.

## Python launch files

Python launch files (`*.launch.py`) give full programmatic control through the `launch` and `launch_ros` libraries, and are the usual choice for complex systems.

### Basic structure

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    # Declare arguments
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Launch in simulation mode'
    )

    # Create a node
    controller_node = Node(
        package='my_robot',
        executable='controller_node',
        name='controller',
        output='screen',
        parameters=[{
            'max_velocity': 1.5,
            'use_sim_time': LaunchConfiguration('use_sim')
        }],
        remappings=[('/cmd_vel', '/robot/cmd_vel')]
    )

    # Conditionally start modern Gazebo through ros_gz_sim
    sim_launch = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
        condition=IfCondition(LaunchConfiguration('use_sim'))
    )

    return LaunchDescription([
        use_sim_arg,
        sim_launch,
        controller_node,
    ])
```

### Key constructs

* **`LaunchDescription`**: The top-level container returned by `generate_launch_description()`. Contains all actions to execute.
* **`Node`**: Launches a ROS 2 node. Supports `package`, `executable`, `name`, `namespace`, `parameters`, `remappings`, `output`, `condition`.
* **`DeclareLaunchArgument`**: Declares a named argument with an optional default and description. Retrieved via `LaunchConfiguration('arg_name')`.
* **`IncludeLaunchDescription`**: Includes and executes another launch file (`.launch.py`, `.launch.xml`, or `.launch.yaml`).
* **`ExecuteProcess`**: Runs any system process (e.g., `rviz2`, `ros2 bag play`).
* **`SetParameter`**: Sets a global parameter for all nodes in the launch description.
* **`GroupAction`**: Groups actions under a shared namespace or condition.
* **Conditions**: `IfCondition` / `UnlessCondition` allow conditional execution based on launch argument values.
* **Substitutions**: `LaunchConfiguration`, `FindPackageShare`, `PathJoinSubstitution`, `EnvironmentVariable` enable dynamic, computed values.

---

## Parameters in Launch Files

Parameters configure node behavior at startup. In ROS 2, parameters can be passed as:

1. **Inline dictionaries** (as shown above).
2. **YAML parameter files**:

```python
Node(
    package='my_robot',
    executable='controller_node',
    parameters=[PathJoinSubstitution([FindPackageShare('my_robot'), 'config', 'params.yaml'])]
)
```

A typical YAML parameter file (`params.yaml`):

```yaml
controller:
  ros__parameters:
    max_velocity: 1.5
    control_frequency: 50.0
    pid_gains:
      kp: 1.0
      ki: 0.1
      kd: 0.05
```

The `/use_sim_time` parameter is particularly important: setting it to `true` causes nodes to synchronize their time with a simulator clock (e.g., Gazebo) instead of the system clock.

---

## Topic and Name Remapping

Remapping allows the same node executable to be reused in different contexts without changing its source code. For example, a generic velocity controller that publishes on `/cmd_vel` can be remapped to `/robot_1/cmd_vel` when launching multiple robots.

```python
# ROS 2 Python
Node(
    package='teleop_twist_keyboard',
    executable='teleop_twist_keyboard',
    remappings=[('cmd_vel', 'robot_1/cmd_vel')]
)
```

```xml
<!-- ROS 2 XML -->
<node pkg="teleop_twist_keyboard" exec="teleop_twist_keyboard" name="teleop">
  <remap from="cmd_vel" to="robot_1/cmd_vel"/>
</node>
```

---

## Running Launch Files

```bash
ros2 launch <package_name> <launch_file> [arg:=value ...]
ros2 launch my_robot bringup.launch.py use_sim:=true
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true map:=/path/to/map.yaml
ros2 launch my_robot bringup.launch.py --show-args     # list the arguments a file accepts
```

---

## Best Practices

* **Modular launch files**: Break complex systems into composable sub-launches (e.g., `sensors.launch.py`, `navigation.launch.py`, `bringup.launch.py`).
* **Expose key arguments**: Use `DeclareLaunchArgument` to expose tunable options (simulation mode, robot name, map path) without editing the file.
* **Use `FindPackageShare`**: Always resolve package paths programmatically rather than hard-coding absolute paths.
* **Use `output='screen'`** during development for immediate feedback; switch to `output='log'` in production.
* **Namespace multi-robot systems**: Use `namespace` and `PushRosNamespace` to isolate multiple robots running the same software.

* **Prefer XML or YAML for plain wiring** and Python when you need logic; the three formats can include one another.

Launch files are the primary mechanism for defining and deploying complete robotic systems in ROS, enabling reproducible, configurable, and modular system startup.

---

## New in 2025–2026

- **Kilted (2025):** `PathJoinSubstitution` can join strings and substitutions into one path component, for example `[LaunchConfiguration('model'), '.xacro']` → `my_model.xacro`.
- **Lyrical (2026):** XML and YAML launch files gain `string-join` and `path-join` substitutions, and a `log` action with a `level` (plus `log_debug`, `log_info`, `log_warning`, `log_error`), so messages in launch logs carry a severity:

```xml
<launch>
  <log_warning message="Starting in simulation: $(path-join opt ros lyrical)"/>
</launch>
```

## Coming from ROS 1

ROS 1's `roslaunch` used XML `.launch` files with `<node pkg type name>`, `<param>`, `<rosparam>`, `<remap>`, `<include>`, `<group ns>` and `<arg>`. ROS 2's XML format is deliberately close:
- `type` becomes `exec`;
- `$(arg x)` becomes `$(var x)`;
- parameters are always per node.

`roslaunch pkg file.launch` becomes `ros2 launch pkg file.launch.xml`.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Launch_Files]])
```
