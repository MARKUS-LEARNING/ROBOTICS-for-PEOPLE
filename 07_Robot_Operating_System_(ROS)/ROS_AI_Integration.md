---
title: ROS AI Integration
description: "How learned models run inside a ROS 2 system: perception nodes, policy nodes that drive ros2_control controllers, remote GPU inference, language and VLA task layers over ROS 2 actions, recording data for training, and the latency and safety rules that keep it working."
tags:
  - ros2
  - robot-learning
  - deep-learning
  - integration
  - real-time
  - computer-vision
type: tool
level: intermediate
subfield:
  - software
  - robot-learning
status: reviewed
author: Jordan_Smith
date: 2025-04-28
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[ROS_2_Overview]]"
  - "[[Policy]]"
related:
  - "[[ros2_control]]"
  - "[[rosbag2]]"
  - "[[Vision-Language-Action_Models]]"
  - "[[Imitation_Learning]]"
  - "[[Actions]]"
  - "[[Computer_Vision_in_Robotics]]"
key_takeaways:
  - "Put learned models in their own nodes; let ros2_control controllers own the motors, limits and timing."
  - "Budget latency end to end (sensor, transport, inference, command) and design around it with action chunks or slower motion."
  - "ROS 2 bags are the raw material for robot learning: record synchronised topics, then convert them to a training dataset format."
sources:
  - "ROS 2 Release Team (2026). ROS 2 Lyrical Luth Released! https://discourse.openrobotics.org/t/ros-2-lyrical-luth-released/55021"
  - "ros2_control maintainers (2026). ros2_control documentation. https://control.ros.org/"
  - "Hugging Face (2026). LeRobot v0.7.0 release notes. https://github.com/huggingface/lerobot/releases/tag/v0.7.0"
  - "NVIDIA (2026). Isaac ROS documentation. https://nvidia-isaac-ros.github.io/"
---

# ROS AI Integration

Modern robots run learned models (object detectors, segmenters, manipulation policies, vision-language-action models) alongside classical ROS 2 software. **ROS AI integration** is the engineering of that combination: where the model runs, how data reaches it, how its outputs reach the motors, and what stops it from doing harm when it is wrong.

> **Etymology.** *ROS* is the Robot Operating System, started in 2007 at Stanford and Willow Garage. *Integration* is from Latin *integrare*, "to make whole", from *integer*, "untouched, entire": turning separate parts into one working system.

## The architecture that works

```
cameras, joints ──► [perception node] ──► objects, poses ──┐
                                                            ├──► [policy node] ──► targets ──► [ros2_control controller] ──► motors
language / task ──► [task node: LLM / VLA planner] ─────────┘          ▲                        (owns limits, timing, safety)
                                                              robot state (joint_states)
```

| Node type | Input | Output | Typical rate |
|---|---|---|---|
| **Perception** | `sensor_msgs/Image`, point clouds | Detections, poses (e.g. `vision_msgs`) | 10–30 Hz |
| **Policy** | Images, `joint_states`, task | Joint or end-effector targets, action chunks | 10–50 Hz |
| **Task layer** | Instructions, perception | Goals for the policy or classical planners, via ROS 2 [[Actions]] | 0.1–2 Hz |
| **Controller** ([[ros2_control]]) | Targets | Motor commands | 100–1000 Hz |

The key rule: **the learned model proposes; the controller disposes.** Controllers enforce joint limits, velocity and torque limits, and a watchdog that stops the robot if commands stop arriving.

## Where inference runs

- **On the robot** (an embedded GPU or NPU): lowest latency; models must be small or optimised (ONNX Runtime, TensorRT). NVIDIA's Isaac ROS packages provide GPU-accelerated perception nodes and zero-copy transport on Jetson hardware.
- **On a nearby workstation or server**: big models such as [[Vision-Language-Action_Models|VLAs]] run on a GPU server; the robot streams observations and receives action chunks. LeRobot v0.7.0 (November 2026) ships a remote policy server and client for exactly this pattern.
- **Split**: a small on-robot model for reflexes, a large remote model for decisions.

## Worked example: a latency budget

A wrist camera runs at 30 Hz, so a frame is up to 33 ms old when captured. Add 5 ms of ROS transport, 60 ms of model inference and 5 ms to reach the controller: about 100 ms from the world changing to the arm reacting. If the gripper approaches an object at 0.5 m/s, it travels $0.5 \times 0.1 = 5$ cm "blind" in that time. Three ways to live with it:

1. **Slow down** near contact (at 0.1 m/s the blind distance is 1 cm).
2. **Predict ahead**: [[Imitation_Learning|action chunking]] lets the policy plan the next second, and the controller executes smoothly while the next chunk is computed.
3. **Close fast loops classically**: let a force or impedance controller handle the moment of contact.

## Recording data for learning

[[rosbag2]] records any set of topics with timestamps. For [[Imitation_Learning]] and [[Teleoperation_and_Data_Collection|teleoperation]] data:

- Record camera images, `joint_states`, commanded actions and the operator's input on the same clock.
- Use a fixed, documented set of topics and rates for every episode.
- Convert bags to a training format (for example the LeRobot dataset format, Parquet tables plus compressed video), aligning all streams to the control rate.

## Practical rules

| Rule | Why |
|---|---|
| Use `BEST_EFFORT` QoS for high-rate camera topics, `RELIABLE` for commands and state | Old images are worthless; lost commands are dangerous |
| Run heavy Python inference in its own process, not the controller's | Python's global interpreter lock and garbage collection cause jitter |
| Timestamp everything at capture, not at receipt | Latency compensation needs the true sensing time |
| Publish model confidence and failure signals | Lets a supervisor slow down or stop |
| Gate every learned output through limits in the controller | A network can output anything, including nonsense |

## State of the art (as of November 2026)

- **ROS 2 Lyrical Luth** (May 2026, supported until May 2031) is the current long-term-support distribution, paired with [[Gazebo_Simulator|Gazebo Jetty]].
- **Remote VLA inference** with streamed action chunks is a common deployment pattern for large generalist policies.
- **GPU-accelerated ROS packages** (NVIDIA Isaac ROS) handle perception and transport on embedded GPUs.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[ROS_AI_Integration]])
```
