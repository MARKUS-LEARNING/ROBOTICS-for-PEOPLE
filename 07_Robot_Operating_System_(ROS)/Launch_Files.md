---
title: Launch Files
description: Explains the ROS launch system, which provides a mechanism for starting and configuring multiple nodes simultaneously with a single command, covering both ROS 1 XML launch files and ROS 2 Python-based launch files.
tags:
  - ROS
  - ROS2
  - launch
  - robotics-software
  - configuration
  - middleware
  - software-stack
  - colcon
layout: default
category: robotics
author: Jordan_Smith_&_le_Chat
date: 2025-05-02
permalink: /launch_files/
related:
  - "[[Robot_Operating_System_(ROS)]]"
  - "[[ROS_2_Overview]]"
  - "[[Custom_Packages_and_Nodes]]"
  - "[[Nodes]]"
  - "[[Topics]]"
  - "[[Services]]"
  - "[[Actions]]"
  - "[[Gazebo_Simulator]]"
  - "[[RViz_Tutorial]]"
  - "[[URDF]]"
  - "[[TF_and_Topic_Architecture]]"
---

# Launch Files

**Launch files** are a core component of the [[Robot_Operating_System_(ROS)|ROS]] framework that allow developers to start, configure, and coordinate multiple [[Nodes|nodes]] and processes with a single command. Rather than manually launching each node in a separate terminal, a launch file describes the entire system — nodes, their parameters, remappings, and inter-process dependencies — in a single declarative or programmatic file. This is essential for managing the complexity of real-world robotic systems, which often involve dozens of concurrent processes.

---

## ROS 1 Launch Files (XML)

In ROS 1, launch files use an XML format with a `.launch` extension. They are executed using the `roslaunch` command.

### Basic Structure

```xml
<launch>
  <!-- Start a single node -->
  <node pkg="my_robot" type="controller_node" name="controller" output="screen">
    <param name="max_velocity" value="1.5"/>
    <remap from="/cmd_vel" to="/robot/cmd_vel"/>
  </node>

  <!-- Include another launch file -->
  <include file="$(find my_robot)/launch/sensors.launch"/>

  <!-- Set a global parameter -->
  <param name="robot_description" command="$(find xacro)/xacro '$(find my_robot)/urdf/robot.urdf.xacro'"/>

  <!-- Conditional launch using arguments -->
  <arg name="use_sim" default="false"/>
  <node if="$(arg use_sim)" pkg="gazebo_ros" type="gzserver" name="gazebo" args="-e ode world.world"/>
</launch>
```

### Key XML Elements

* **`<node>`**: Launches a single ROS node. Key attributes:
  * `pkg`: Package containing the node executable.
  * `type`: Name of the executable.
  * `name`: Name assigned to the node at runtime.
  * `output`: Where to send node output (`screen` or `log`).
  * `respawn`: Restart node automatically if it crashes (`true`/`false`).
  * `ns`: Namespace prefix for the node.
* **`<param>`**: Sets a parameter on the ROS Parameter Server (scalar or loaded from file).
* **`<rosparam>`**: Loads a YAML parameter file or sets grouped parameters.
* **`<remap>`**: Remaps a topic, service, or parameter name to a different name for a specific node.
* **`<include>`**: Includes another `.launch` file, allowing modular composition.
* **`<arg>`**: Declares a launch argument that can be passed from the command line:
  ```
  roslaunch my_robot bringup.launch use_sim:=true
  ```
* **`<group>`**: Groups nodes under a shared namespace or conditional flag.
* **`<env>`**: Sets an environment variable for child processes.

---

## ROS 2 Launch Files (Python)

ROS 2 replaces XML launch files with **Python-based `.launch.py` files**, offering full programmatic control via the `launch` and `launch_ros` Python libraries. XML and YAML formats are also supported for simpler cases, but Python is the standard for complex systems.

### Basic Structure

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

    # Conditionally include another launch file
    sim_launch = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py']),
        condition=IfCondition(LaunchConfiguration('use_sim'))
    )

    return LaunchDescription([
        use_sim_arg,
        sim_launch,
        controller_node,
    ])
```

### Key Python Launch Constructs

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
<!-- ROS 1 XML -->
<node pkg="teleop_twist_keyboard" type="teleop_twist_keyboard.py" name="teleop">
  <remap from="cmd_vel" to="robot_1/cmd_vel"/>
</node>
```

---

## Running Launch Files

```bash
# ROS 1
roslaunch <package_name> <launch_file>.launch [arg:=value ...]

# ROS 2
ros2 launch <package_name> <launch_file>.launch.py [arg:=value ...]

# Examples
roslaunch my_robot bringup.launch use_sim:=true
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true map:=/path/to/map.yaml
```

---

## Best Practices

* **Modular launch files**: Break complex systems into composable sub-launches (e.g., `sensors.launch.py`, `navigation.launch.py`, `bringup.launch.py`).
* **Expose key arguments**: Use `DeclareLaunchArgument` to expose tunable options (simulation mode, robot name, map path) without editing the file.
* **Use `FindPackageShare`**: Always resolve package paths programmatically rather than hard-coding absolute paths.
* **Use `output='screen'`** during development for immediate feedback; switch to `output='log'` in production.
* **Namespace multi-robot systems**: Use `namespace` and `PushRosNamespace` to isolate multiple robots running the same software.

Launch files are the primary mechanism for defining and deploying complete robotic systems in ROS, enabling reproducible, configurable, and modular system startup.

---

## Dataview Plugin Features

```dataview
LIST FROM #ROS OR #robotics-software WHERE contains(file.outlinks, [[Launch_Files]])
```

```dataview
TABLE title, description FROM #ROS WHERE contains(file.tags, "launch")
```
