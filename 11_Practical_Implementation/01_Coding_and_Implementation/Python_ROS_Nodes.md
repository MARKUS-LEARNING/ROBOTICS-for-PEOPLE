---
title: Python ROS Nodes
description: "Writing ROS 2 nodes in Python with rclpy: structure, publishers and subscriptions with QoS, timers, parameters, services and actions, executors and the asyncio AsyncNode (Lyrical), packaging and launching, a computed timing budget for Python callbacks, and when to move code to C++. Updated for ROS 2 Lyrical (Python 3.12–3.14); ROS 1 rospy is end of life."
aliases:
  - rclpy
  - Python ROS 2 Nodes
tags:
  - ros2
  - python
  - robotics-software
type: tutorial
level: intermediate
subfield:
  - software
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Custom_Packages_and_Nodes]]"
  - "[[Python_for_Robotics]]"
related:
  - "[[ROS_2_Communication]]"
  - "[[Actions]]"
  - "[[Launch_Files]]"
  - "[[C++]]"
key_takeaways:
  - "rclpy is the Python client library for ROS 2; a node subclasses Node and creates publishers, subscriptions, timers, services and actions."
  - "A 100 Hz callback has a 10 ms budget; if each call takes 2 ms of Python, that is 20% of a core, so heavy per-message work belongs in NumPy, compiled extensions or C++."
  - "ROS 2 Lyrical supports Python 3.12–3.14 and adds the experimental AsyncNode for asyncio; ROS 1's rospy reached end of life with Noetic in May 2025."
sources:
  - "Open Robotics (2026). rclpy: ROS 2 client library for Python. https://github.com/ros2/rclpy"
  - "Open Robotics (2026). ROS 2 Lyrical Luth supported platforms (Python 3.12–3.14). https://github.com/ros2/ros2_documentation/blob/rolling/source/Releases/lyrical/supported-platforms.rst"
  - "Open Robotics (2026). ROS 2 documentation source: Writing a simple publisher and subscriber (Python). https://github.com/ros2/ros2_documentation/blob/rolling/source/ROS-Framework/client-libraries/Working-with-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.rst"
---

# Python ROS Nodes

Python is the fastest way to write a ROS 2 node: perception glue, high-level behaviours, bridges to machine-learning models, test tools. The client library is **rclpy**. (ROS 1's `rospy` reached end of life with Noetic in May 2025.)

> **Etymology.** *rclpy* = **R**OS **c**lient **l**ibrary for **Py**thon, built on the common C library *rcl*.

## A complete node

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class Stopper(Node):
    """Stop the robot if anything is closer than a threshold in front of it."""
    def __init__(self):
        super().__init__('stopper')
        self.declare_parameter('stop_distance', 0.5)
        self.sub = self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.closest = float('inf')
        self.timer = self.create_timer(0.05, self.tick)          # 20 Hz control loop

    def on_scan(self, msg: LaserScan):
        n = len(msg.ranges); front = msg.ranges[n // 2 - 20 : n // 2 + 20]
        self.closest = min((r for r in front if msg.range_min < r < msg.range_max), default=float('inf'))

    def tick(self):
        cmd = Twist()
        if self.closest > self.get_parameter('stop_distance').value:
            cmd.linear.x = 0.2
        self.pub.publish(cmd)

def main():
    rclpy.init()
    rclpy.spin(Stopper())

if __name__ == '__main__':
    main()
```

Notes: the scan subscription uses the **sensor-data QoS** (best effort) to match lidar drivers (see [[ROS_2_Communication]]); a timer runs the control loop independently of the scan rate; the threshold is a declared parameter, adjustable at launch.

## Executors, threads and async

`rclpy.spin` uses a single-threaded executor. Long callbacks block everything else on that node, and calling a service synchronously from a callback deadlocks (see [[ROS_2_Communication]]). Options:
- `MultiThreadedExecutor` with reentrant callback groups;
- `call_async` for services;
- in Lyrical, the experimental **AsyncNode**, which runs callbacks on an `asyncio` loop so they can `await` service calls and sleeps, using less CPU than the default executor.

## Worked example: Python's time budget

At 100 Hz each callback has 10 ms. If processing one message in Python takes 2 ms, the node uses /10 = 20\%$ of one CPU core for that topic alone, and other callbacks queue behind it. Ways to keep within budget:
- vectorise with NumPy (see [[Python_for_Robotics]]);
- move hot loops to compiled code;
- reduce rates;
- for 1 kHz control or large point clouds, write the node in [[C++]] and compose it in-process.

## Packaging and running

Create with `ros2 pkg create --build-type ament_python`, declare entry points in `setup.py`, build with `colcon build --symlink-install` (edits take effect without rebuilding), and start with `ros2 run` or a [[Launch_Files|launch file]]. Lyrical targets Ubuntu 26.04 with Python 3.14; it supports Python 3.12–3.14 across its platforms.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Python_ROS_Nodes]])
```
