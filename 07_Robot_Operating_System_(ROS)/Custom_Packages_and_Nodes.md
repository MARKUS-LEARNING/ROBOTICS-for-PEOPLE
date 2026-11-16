---
title: Custom Packages and Nodes
description: "How to write your own ROS 2 software: create a package, write a node in Python or C++, define custom messages, services and actions, build with colcon, and follow the 2025–2026 conventions (target_link_libraries instead of the deprecated ament_target_dependencies, AsyncNode in rclpy). Includes a complete minimal publisher and the build-and-run loop."
aliases:
  - ROS Package
  - ROS Node
  - Nodes
  - Packages
  - colcon
tags:
  - ros
  - ros2
  - package
  - nodes
  - robotics-software
  - colcon
  - software-stack
type: tutorial
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
  - "[[Launch_Files]]"
  - "[[Actions]]"
  - "[[Python_ROS_Nodes]]"
  - "[[TF_and_Topic_Architecture]]"
  - "[[Git_Integration_and_Version_Control]]"
key_takeaways:
  - "A package is ROS's unit of build and release: a package.xml manifest plus CMakeLists.txt (ament_cmake) or setup.py (ament_python), and the nodes, interfaces, launch and config files it ships."
  - "A node is one program with one job; it creates publishers, subscriptions, timers, services and actions, and an executor runs their callbacks."
  - "Since Kilted (2025), link dependencies with target_link_libraries() and modern CMake targets; ament_target_dependencies() is deprecated."
sources:
  - "Open Robotics (2026). ROS 2 documentation source: Creating a package. https://github.com/ros2/ros2_documentation/blob/rolling/source/ROS-Framework/client-libraries/Working-with-Client-Libraries/Creating-Your-First-ROS2-Package.rst"
  - "Open Robotics (2026). ROS 2 documentation source: Writing a simple publisher and subscriber (Python). https://github.com/ros2/ros2_documentation/blob/rolling/source/ROS-Framework/client-libraries/Working-with-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.rst"
  - "Open Robotics (2025). ROS 2 Kilted Kaiju release notes: ament_target_dependencies is deprecated. https://github.com/ros2/ros2_documentation/blob/rolling/source/Releases/Release-Kilted-Kaiju.rst"
  - "Open Robotics (2026). ROS 2 Lyrical Luth release notes: AsyncNode. https://github.com/ros2/ros2_documentation/blob/rolling/source/Releases/Release-Lyrical-Luth.rst"
  - "Open Robotics (2017). REP 149: Package Manifest Format Three Specification. https://www.ros.org/reps/rep-0149.html"
---

# Custom Packages and Nodes

Sooner or later every ROS user writes their own software: a driver for a new sensor, a perception step, a controller, a node that runs a learned policy. In ROS 2 that software lives in **packages** and runs as **nodes**.

> **Etymology.** *Node* is Latin *nodus*, "knot": in a graph, the knots where lines meet. *Package* comes from Middle Dutch *pak*, "bundle"; in software, a bundle of code plus the metadata needed to build, install and share it.

## Packages

A **package** is the smallest thing ROS builds, installs and releases. It contains:

| File or folder | Purpose |
|---|---|
| `package.xml` | Manifest (format 3): name, version, licence, maintainers, dependencies |
| `CMakeLists.txt` | Build rules for C++ packages (`ament_cmake`) |
| `setup.py` / `setup.cfg` | Build rules for Python packages (`ament_python`) |
| `src/`, `my_pkg/` | Source code |
| `msg/`, `srv/`, `action/` | Custom interface definitions |
| `launch/`, `config/` | [[Launch_Files]] and parameter YAML |
| `urdf/`, `meshes/` | Robot model (see [[URDF]]) |

Packages live in the `src/` folder of a **workspace**. `colcon build` builds them all in dependency order into `install/`, and `source install/setup.bash` makes them available.

```bash
mkdir -p ~/ws/src && cd ~/ws/src
ros2 pkg create --build-type ament_python --license Apache-2.0 --node-name talker my_talker
cd ~/ws && rosdep install --from-paths src -y --ignore-src   # install system dependencies
colcon build --symlink-install                               # Python edits take effect without rebuilding
source install/setup.bash
ros2 run my_talker talker
```

## Nodes

A **node** is one program in the ROS graph. It creates communication endpoints:
- publishers and subscriptions for topics;
- service servers and clients;
- action servers and clients (see [[Actions]]);
- timers and parameters.

An **executor** then runs the callbacks those endpoints trigger. A minimal Python publisher:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.declare_parameter('rate_hz', 2.0)
        self.pub = self.create_publisher(String, 'chatter', 10)       # queue depth 10
        period = 1.0 / self.get_parameter('rate_hz').value
        self.timer = self.create_timer(period, self.tick)
        self.n = 0

    def tick(self):
        msg = String(data=f'hello {self.n}')
        self.pub.publish(msg)
        self.n += 1

def main():
    rclpy.init()
    rclpy.spin(Talker())

if __name__ == '__main__':
    main()
```

Run it with a different rate: `ros2 run my_talker talker --ros-args -p rate_hz:=10.0`. Check it: `ros2 topic hz /chatter` should report about 10 Hz.

**Good node design.**
- Give each node one job.
- Expose tunables as declared parameters.
- Use the standard message types (`sensor_msgs`, `geometry_msgs`, `nav_msgs`) wherever they fit.
- Put time-critical work in C++ components that can be composed into one process.

New in Lyrical (2026), rclpy's experimental `AsyncNode` runs callbacks on an `asyncio` loop, so a callback can `await client.call(request)` without deadlocking an executor.

## Custom interfaces

When the standard types do not fit, define your own. By convention, interfaces go in a separate package, for example `my_robot_interfaces`, so that other packages can depend on them without depending on your code.

```text
# msg/BatteryHealth.msg
std_msgs/Header header
float32 voltage          # V
float32 temperature      # °C
uint8 cycles_since_service

# srv/SetMode.srv
string mode
---
bool success
string message

# action/Dock.action
string dock_id           # goal
---
bool docked              # result
---
float32 distance_left    # feedback
```

The build system generates C++ and Python code for each. In `CMakeLists.txt`, call `rosidl_generate_interfaces(${PROJECT_NAME} "msg/BatteryHealth.msg" "srv/SetMode.srv" "action/Dock.action" DEPENDENCIES std_msgs)`; in `package.xml`, add `rosidl_default_generators` as a build tool and `<member_of_group>rosidl_interface_packages</member_of_group>`.

## C++ packages: modern CMake (2025 onward)

Since Kilted, `ament_target_dependencies()` prints a deprecation warning. Link targets directly instead:

```cmake
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
add_executable(scan_filter src/scan_filter.cpp)
target_link_libraries(scan_filter PUBLIC rclcpp::rclcpp ${sensor_msgs_TARGETS})
install(TARGETS scan_filter DESTINATION lib/${PROJECT_NAME})
```

## Habits that save time

- Pin dependencies in `package.xml` and install them with `rosdep`; never rely on "it works on my machine".
- Add tests: `colcon test` runs `ament_cmake_gtest`, `pytest` and linters (`ament_lint_auto`).
- Release through `bloom` to the ROS build farm when other people need binaries.
- Keep the workspace under version control; see [[Git_Integration_and_Version_Control]].

## Coming from ROS 1

| ROS 1 | ROS 2 |
|---|---|
| `catkin_create_pkg` | `ros2 pkg create` |
| `catkin_make` / `catkin build` | `colcon build` |
| `rospy.init_node`, global node | `rclpy.init()`, `Node` subclass |
| `roscpp` `NodeHandle` | `rclcpp::Node` |
| Global parameter server | Per-node declared parameters |

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Custom_Packages_and_Nodes]])
```
