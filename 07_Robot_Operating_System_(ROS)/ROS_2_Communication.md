---
title: "ROS 2 Communication: Topics, Services, Messages and Parameters"
description: "The four ways ROS 2 nodes talk: topics (streams), services (request and reply), actions (long goals with feedback) and parameters (configuration), plus the message definitions they all share. Covers naming, the .msg type system, choosing a pattern, a worked bandwidth example, and the synchronous-service deadlock that catches most beginners."
aliases:
  - Topics
  - Services
  - Messages
  - Parameters
  - ROS Topics
  - ROS Services
tags:
  - ros
  - ros2
  - topics
  - middleware
  - communication-protocols
  - robotics-software
type: concept
level: beginner
subfield:
  - software
status: reviewed
author: Jordan_Smith
date: 2026-11-15
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Robot_Operating_System_(ROS)]]"
related:
  - "[[ROS_2_Overview]]"
  - "[[Actions]]"
  - "[[Custom_Packages_and_Nodes]]"
  - "[[TF_and_Topic_Architecture]]"
  - "[[rosbag2]]"
key_takeaways:
  - "Topics stream data many-to-many; services answer short questions; actions run long goals with feedback and cancellation; parameters configure a node."
  - "Every message type is defined in a small .msg, .srv or .action file; use the standard packages (std_msgs, sensor_msgs, geometry_msgs, nav_msgs) before inventing your own."
  - "Never block a callback waiting for a service reply on a single-threaded executor: use call_async, a multi-threaded executor, or rclpy's AsyncNode (Lyrical)."
sources:
  - "Open Robotics (2026). ROS 2 documentation source: About topics. https://github.com/ros2/ros2_documentation/blob/rolling/source/ROS-Framework/interfaces/About-Topics.rst"
  - "Open Robotics (2026). ROS 2 documentation source: About services. https://github.com/ros2/ros2_documentation/blob/rolling/source/ROS-Framework/interfaces/About-Services.rst"
  - "Open Robotics (2026). ROS 2 documentation source: Synchronous vs. asynchronous service clients. https://github.com/ros2/ros2_documentation/blob/rolling/source/ROS-Framework/interfaces/services/Working-with-services/Sync-Vs-Async.rst"
  - "Open Robotics (2019). ROS 2 design: Interface definition and language mapping. https://design.ros2.org/articles/interface_definition.html"
  - "Open Robotics (2019). ROS 2 design: Topic and service name mapping to DDS. https://design.ros2.org/articles/topic_and_service_names.html"
---

# ROS 2 Communication: Topics, Services, Messages and Parameters

[[Robot_Operating_System_(ROS)|ROS]] programs (nodes) cooperate by exchanging typed data in four patterns. Picking the right pattern for each piece of data is most of the craft of designing a ROS system.

> **Etymology.** *Topic* is Greek *topos*, "place": a named place where messages gather. *Service* is Latin *servitium*, "the work of a servant": you ask, it answers. *Message* is Latin *missus*, "sent".

## The four patterns

| Pattern | Shape | Timing | Example |
|---|---|---|---|
| **Topic** | Many publishers → many subscribers | Continuous stream, fire and forget | `/scan`, `/camera/image_raw`, `/joint_states`, `/cmd_vel` |
| **Service** | One client ↔ one server | Short request, one reply | `/reset_odometry`, `/get_map`, `/set_mode` |
| **Action** | Client ↔ server, with feedback and cancel | Seconds to minutes | `/navigate_to_pose`, `/follow_joint_trajectory` (see [[Actions]]) |
| **Parameter** | Values owned by one node | Read at start, changed occasionally | `max_velocity`, `frame_id`, controller gains |

**Rule of thumb.**
- Is it data that keeps coming? Use a topic.
- Is it a question with a quick answer? Use a service.
- Does it take a while, and might you want progress or to stop it? Use an action.
- Is it configuration? Use a parameter.

## Messages: the shared vocabulary

Every pattern is typed by an **interface** definition. A message is a list of typed fields:

```text
# geometry_msgs/msg/Twist (the velocity command almost every mobile base accepts)
Vector3 linear    # m/s along x (forward), y (left), z (up)
Vector3 angular   # rad/s about x, y, z
```

Field types are primitives (`bool`, `int8`…`int64`, `uint8`…`uint64`, `float32`, `float64`, `string`), other messages, and arrays (`float64[]` unbounded, `float64[3]` fixed, `float64[<=10]` bounded). Services (`.srv`) put a request and a response either side of `---`; actions (`.action`) have goal, result and feedback.

Use the standard interface packages before writing your own:

| Package | Contains |
|---|---|
| `std_msgs` | `Header` (timestamp + frame) and simple wrappers |
| `geometry_msgs` | `Pose`, `Twist`, `Transform`, `Wrench`, stamped variants |
| `sensor_msgs` | `Image`, `CameraInfo`, `LaserScan`, `PointCloud2`, `Imu`, `JointState`, `BatteryState` |
| `nav_msgs` | `Odometry`, `Path`, `OccupancyGrid` |
| `trajectory_msgs`, `control_msgs` | Joint trajectories and controller interfaces |

Standard types are what make drivers, tools and algorithms interchangeable: any lidar driver that publishes `sensor_msgs/msg/LaserScan` works with SLAM Toolbox, Nav2 and RViz.

## Names and namespaces

Names are hierarchical, like file paths:
- `/robot1/camera/image_raw` is absolute;
- `camera/image_raw` is relative to the node's namespace;
- `~/status` is private to the node.

Launching the same software twice under the namespaces `robot1` and `robot2` keeps two robots' topics apart without code changes. Remapping changes any name at launch time (see [[Launch_Files]]).

## Worked example: does it fit on the network?

A mobile manipulator publishes:

| Topic | Size per message | Rate | Bandwidth |
|---|---|---|---|
| `/joint_states` (7 joints) | ≈ 7 × 3 × 8 B + names ≈ 0.3 kB | 500 Hz | ≈ 150 kB/s |
| `/scan` (1,080 ranges + intensities) | ≈ 8.6 kB | 10 Hz | ≈ 86 kB/s |
| `/camera/image_raw` 1280×720 RGB | 2.76 MB | 30 Hz | ≈ 83 MB/s |
| `/camera/image_raw/compressed` (JPEG, ~10:1) | ≈ 0.28 MB | 30 Hz | ≈ 8.3 MB/s |

Everything except the raw image fits comfortably on Wi-Fi. The raw image needs a wired link, compression, or to stay inside one process (composition or zero-copy). This back-of-envelope check is worth doing before every system design; `ros2 topic bw` measures the real numbers.

## Services and the classic deadlock

A service call waits for a reply. In rclpy, calling the *synchronous* `client.call(request)` from inside a callback, on a single-threaded executor, blocks the only thread, which is also the thread that would deliver the reply. So it never returns.

```python
# Wrong (inside a timer or subscription callback, single-threaded executor):
result = self.client.call(req)          # waits forever

# Right: ask asynchronously and handle the answer in a callback
future = self.client.call_async(req)
future.add_done_callback(lambda f: self.get_logger().info(f'got {f.result()}'))
```

The other fixes are to use a `MultiThreadedExecutor` with a reentrant callback group, or, from Lyrical, rclpy's experimental `AsyncNode`, where callbacks can simply `await self.client.call(req)`.

## Parameters

Each node **declares** its parameters (name, type, default, optional range) and owns them. They can be set:
- at launch, from YAML files;
- from the command line: `ros2 param set /controller max_velocity 0.8`;
- by other nodes.

A node can reject invalid changes in a callback. From Lyrical (2026), `ros2 param get` can read a parameter from all nodes at once, and get or set several parameters on one node in a single command.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[ROS_2_Communication]])
```
