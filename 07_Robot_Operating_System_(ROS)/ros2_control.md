---
title: ros2_control
description: "ros2_control is ROS 2's framework for real-time robot control: hardware interfaces expose joint states and commands, a controller manager runs controllers in a fixed-rate loop, and the same controllers drive simulation and the real robot. Covers the architecture, URDF tags, controller configuration, lifecycle commands, writing a hardware interface, the Lyrical (2026) changes, and learned policies."
aliases:
  - ros2 control
  - controller_manager
tags:
  - ros2_control
  - ros2
  - control
  - hardware-interface
  - real-time
  - robotics-software
  - actuators
type: tool
level: intermediate
subfield:
  - software
  - electronics-embedded
  - control
status: reviewed
author: Jordan_Smith
date: 2025-04-28
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[ROS_2_Overview]]"
  - "[[URDF]]"
related:
  - "[[MoveIt2]]"
  - "[[Gazebo_Simulator]]"
  - "[[PID_Control]]"
  - "[[Reinforcement_Learning_for_Robots]]"
  - "[[ROS_AI_Integration]]"
key_takeaways:
  - "ros2_control separates what talks to motors (hardware interfaces) from what decides commands (controllers); a controller manager runs both in one real-time loop."
  - "The same controllers run in Gazebo (gz_ros2_control), MuJoCo or Isaac bridges, and on hardware: only the hardware plugin in the URDF changes."
  - "Learned policies usually sit above ros2_control, sending joint position or effort targets at 50–500 Hz to controllers that run at 500 Hz–1 kHz."
sources:
  - "Chitta, S. et al. (2017). ros_control: A generic and simple control framework for ROS. https://joss.theoj.org/papers/10.21105/joss.00456"
  - "ros-controls (2026). ros2_control. https://github.com/ros-controls/ros2_control"
  - "ros-controls (2026). ros2_control release notes: Kilted Kaiju to Lyrical Luth. https://github.com/ros-controls/ros2_control/blob/master/doc/release_notes.rst"
  - "ros-controls (2026). ros2_controllers. https://github.com/ros-controls/ros2_controllers"
  - "ros-controls (2026). gz_ros2_control: ros2_control for modern Gazebo. https://github.com/ros-controls/gz_ros2_control"
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

For **Gazebo simulation** (modern Gazebo: Harmonic with Jazzy, Jetty with Lyrical), replace the hardware plugin with `gz_ros2_control` and load its system plugin in the model:

```xml
<hardware>
  <plugin>gz_ros2_control/GazeboSimSystem</plugin>
</hardware>
<!-- elsewhere in the URDF -->
<gazebo>
  <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
    <parameters>$(find my_robot)/config/controllers.yaml</parameters>
  </plugin>
</gazebo>
```

(`gazebo_ros2_control/GazeboSystem` was the Gazebo Classic plugin; Gazebo Classic reached end of life in January 2025.)

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

This clean layering means MoveIt 2 works identically with a Gazebo simulation (using `gz_ros2_control`) and the real robot (using the custom hardware interface plugin) — only the `<hardware><plugin>` line in the URDF changes.

---

## Worked example: rates and latency budget

A 6-joint arm runs its controller manager at 1 kHz, so each cycle has 1 ms for `read()` → controllers → `write()`. Suppose EtherCAT `read` and `write` take 0.15 ms each and a joint trajectory controller takes 0.05 ms. That leaves 1 − 0.35 = 0.65 ms of slack, enough for a few more controllers but not for anything that allocates memory or waits on the network. That is why non-real-time work (planning, perception, learned policies) runs in other nodes and only *sends targets* into the loop.

A learned policy at 50 Hz sends a new target every 20 ms, so the 1 kHz controller interpolates and tracks each target over 20 cycles. The split, a slow learned brain over a fast classical spine, is how most 2026 robots deploy [[Reinforcement_Learning_for_Robots|RL]] and [[Vision-Language-Action_Models|VLA]] policies.

## What changed for Lyrical (2026)

From the ros2_control release notes (Kilted → Lyrical):
- **Safer switching.** `switch_controller` now defaults to `strict`. If a controller in a chain fails during an update, the controller manager deactivates the whole chain. A new `handle_exceptions` parameter decides whether controller exceptions are caught.
- **More data types.** Handles now carry `float32` and integer types, not only `double` and `bool`.
- **Tooling.** There is a new `cleanup_controller` service (and CLI verb), the spawner accepts per-controller arguments, and a `--reconfigure` option was added.
- **Performance.** Clocks are resolved once per cycle, and the hardware-interface listings now scale linearly.
- **Sensors.** A `MagneticFieldSensor` semantic component was added.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[ros2_control]])
```
