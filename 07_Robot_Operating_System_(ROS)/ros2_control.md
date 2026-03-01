---
title: ros2_control
description: Explains ros2_control, the hardware abstraction and real-time control framework for ROS 2, covering the controller manager, hardware interfaces, controllers, and the resource manager that enables portable, modular robot control.
tags:
  - ros2_control
  - ROS2
  - control
  - hardware-interface
  - real-time
  - robotics-software
  - middleware
  - actuator
  - joint-control
layout: default
category: robotics
author: Jordan_Smith_&_le_Chat
date: 2025-05-02
permalink: /ros2_control/
related:
  - "[[Robot_Operating_System_(ROS)]]"
  - "[[ROS_2_Overview]]"
  - "[[URDF]]"
  - "[[MoveIt2]]"
  - "[[Control_Systems]]"
  - "[[PID_Controller]]"
  - "[[Actuators]]"
  - "[[Joints]]"
  - "[[Trajectory_Planning]]"
  - "[[Gazebo_Simulator]]"
  - "[[Custom_Packages_and_Nodes]]"
---

# ros2_control

**ros2_control** is the standard hardware abstraction and real-time control framework for ROS 2. It provides a clean separation between **control algorithms** (controllers) and **hardware drivers** (hardware interfaces), enabling the same controller code to run on any robot hardware — physical or simulated — without modification.

The framework is the successor to `ros_control` from ROS 1 and is now the recommended approach for robot joint control, replacing ad-hoc velocity/position command publishers with a well-structured, real-time-capable architecture.

---

## Core Design Principle

ros2_control enforces a strict interface boundary:

```
┌──────────────────────────────────────────────────────────────────┐
│                        Controller Layer                          │
│  JointTrajectoryController  │  DiffDriveController  │  Custom   │
│       (reads state, writes commands via interfaces)              │
└───────────────────────────┬──────────────────────────────────────┘
                            │  State & Command Interfaces
┌───────────────────────────▼──────────────────────────────────────┐
│                   Resource Manager / Controller Manager          │
│           (mediates access to hardware interface data)           │
└───────────────────────────┬──────────────────────────────────────┘
                            │  Read/Write hardware data
┌───────────────────────────▼──────────────────────────────────────┐
│                     Hardware Interface Layer                     │
│   ActuatorInterface  │  SystemInterface  │  SensorInterface      │
│       (talks directly to motors, encoders, IMUs, etc.)           │
└──────────────────────────────────────────────────────────────────┘
```

Controllers only interact with **named interfaces** (e.g., `joint1/position`, `joint2/velocity`). They never interact with hardware directly. This decoupling means switching from a Gazebo simulation to real hardware requires only changing the hardware interface plugin — the controllers remain unchanged.

---

## Key Components

### Hardware Interfaces

A **hardware interface** is a plugin that implements one of three C++ abstract classes:

| Class | Use Case |
|---|---|
| `hardware_interface::SystemInterface` | Full robot system (multiple joints, sensors). Most common. |
| `hardware_interface::ActuatorInterface` | Single actuator (1 joint). |
| `hardware_interface::SensorInterface` | Read-only sensor (IMU, force-torque sensor). |

Each hardware interface exposes **state interfaces** (read-only, e.g., encoder position) and **command interfaces** (write-only, e.g., motor torque setpoint).

### State and Command Interfaces

| Interface Type | Direction | Examples |
|---|---|---|
| State Interface | Hardware → Controller | `position`, `velocity`, `effort`, `temperature` |
| Command Interface | Controller → Hardware | `position`, `velocity`, `effort`, `kp`, `kd` |

### Controller Manager

The **Controller Manager** is the central ROS 2 node that:
1. Loads and manages the lifecycle of controllers and hardware interfaces.
2. Runs the real-time control loop at a fixed rate (typically 100–1000 Hz).
3. Mediates access to shared hardware interface data between controllers.
4. Exposes ROS 2 services for loading, configuring, activating, and deactivating controllers.

### Controllers

Controllers are plugins that implement `controller_interface::ControllerInterface`. At each control loop iteration they:
1. **Read** current robot state from state interfaces (joint positions, velocities).
2. **Compute** control commands (e.g., PID, trajectory interpolation, impedance).
3. **Write** commands to command interfaces.

Available standard controllers from `ros2_controllers`:

| Controller | Function |
|---|---|
| `JointTrajectoryController` | Executes joint-space trajectories; used with MoveIt 2 |
| `DiffDriveController` | Differential drive mobile base; publishes odometry |
| `JointStateBroadcaster` | Publishes `/joint_states` from state interfaces |
| `ForwardCommandController` | Forwards commands directly to command interfaces |
| `PIDController` | Generic PID control loop |
| `AdmittanceController` | Impedance/admittance control for compliant manipulation |
| `JointGroupVelocityController` | Commands joint velocities directly |

---

## URDF Integration

ros2_control is configured through the [[URDF]] or xacro file via the `<ros2_control>` tag:

```xml
<ros2_control name="my_arm_hardware" type="system">
  <hardware>
    <!-- Swap this plugin to switch hardware without changing controllers -->
    <plugin>my_robot_hardware/MyArmHardware</plugin>
    <param name="serial_port">/dev/ttyUSB0</param>
    <param name="baud_rate">115200</param>
  </hardware>

  <joint name="shoulder_joint">
    <command_interface name="position">
      <param name="min">-3.14</param>
      <param name="max">3.14</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>

  <joint name="elbow_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>

  <!-- IMU sensor interface -->
  <sensor name="base_imu">
    <state_interface name="orientation.x"/>
    <state_interface name="orientation.y"/>
    <state_interface name="orientation.z"/>
    <state_interface name="orientation.w"/>
    <state_interface name="angular_velocity.x"/>
    <state_interface name="angular_velocity.y"/>
    <state_interface name="angular_velocity.z"/>
  </sensor>
</ros2_control>
```

For **Gazebo simulation**, replace the hardware plugin with the Gazebo bridge:

```xml
<hardware>
  <plugin>gazebo_ros2_control/GazeboSystem</plugin>
</hardware>
```

---

## Controller Configuration (YAML)

Controllers and the controller manager are configured via a YAML parameter file:

```yaml
# controller_manager configuration
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz — real-time control loop rate

    # Declare which controllers to load
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

# JointTrajectoryController parameters
joint_trajectory_controller:
  ros__parameters:
    joints:
      - shoulder_joint
      - elbow_joint
      - wrist_joint
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    state_publish_rate: 50.0
    action_monitor_rate: 20.0
    allow_partial_joints_goal: false
    constraints:
      stopped_velocity_tolerance: 0.01
      goal_time: 0.5
```

---

## Lifecycle and Controller Management

Controllers follow the **ROS 2 Managed Node lifecycle**: `unconfigured → inactive → active`. The Controller Manager exposes services to transition controllers:

```bash
# List all loaded controllers and their state
ros2 control list_controllers

# Load and configure a controller (inactive state)
ros2 control load_controller joint_trajectory_controller

# Activate a controller (starts running in control loop)
ros2 control set_controller_state joint_trajectory_controller active

# Deactivate a controller
ros2 control set_controller_state joint_trajectory_controller inactive

# Switch controllers atomically (deactivate one, activate another)
ros2 control switch_controllers \
  --activate joint_trajectory_controller \
  --deactivate forward_command_controller \
  --strict
```

---

## Implementing a Custom Hardware Interface

```cpp
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

class MyRobotHardware : public hardware_interface::SystemInterface {
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override
  {
    // Validate URDF hardware info and open serial port
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS) {
      return hardware_interface::CallbackReturn::ERROR;
    }
    joint_positions_.resize(info_.joints.size(), 0.0);
    joint_velocities_.resize(info_.joints.size(), 0.0);
    joint_commands_.resize(info_.joints.size(), 0.0);
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
    std::vector<hardware_interface::StateInterface> interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      interfaces.emplace_back(info_.joints[i].name,
        hardware_interface::HW_IF_POSITION, &joint_positions_[i]);
      interfaces.emplace_back(info_.joints[i].name,
        hardware_interface::HW_IF_VELOCITY, &joint_velocities_[i]);
    }
    return interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
    std::vector<hardware_interface::CommandInterface> interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      interfaces.emplace_back(info_.joints[i].name,
        hardware_interface::HW_IF_POSITION, &joint_commands_[i]);
    }
    return interfaces;
  }

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override
  {
    // Read encoder values from hardware (serial, CAN, EtherCAT, etc.)
    // Populate joint_positions_ and joint_velocities_
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override
  {
    // Send joint_commands_ to actuators
    return hardware_interface::return_type::OK;
  }

private:
  std::vector<double> joint_positions_, joint_velocities_, joint_commands_;
};
```

Register in the package's `CMakeLists.txt` and `pluginlib` XML descriptor to make the plugin discoverable.

---

## Integration with MoveIt 2

[[MoveIt2]] sends `trajectory_msgs/JointTrajectory` goals to the `JointTrajectoryController` via its action interface (`FollowJointTrajectory`). The full execution chain:

```
MoveIt 2 (motion planner)
  → JointTrajectoryController (action server, trajectory interpolation)
    → ros2_control Resource Manager (state/command interface mediation)
      → Hardware Interface (read encoders, write motor commands)
        → Physical Robot or Gazebo
```

This clean layering means MoveIt 2 works identically with a Gazebo simulation (using `gazebo_ros2_control`) and the real robot (using the custom hardware interface plugin) — only the `<hardware><plugin>` line in the URDF changes.

---

## Dataview Plugin Features

```dataview
LIST FROM #ros2_control OR #control WHERE contains(file.outlinks, [[ros2_control]])
```

```dataview
TABLE title, description FROM #ROS WHERE contains(file.tags, "ros2_control")
```
