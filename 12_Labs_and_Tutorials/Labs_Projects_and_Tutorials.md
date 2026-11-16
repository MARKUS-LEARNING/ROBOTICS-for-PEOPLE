---
title: Labs, Projects, and Tutorials
description: "A learning path of hands-on robotics projects, from a first ROS 2 node to training a policy for a real low-cost arm, using free tools available in 2026 (ROS 2 Lyrical, turtlesim, Gazebo, MuJoCo and MuJoCo Playground, Isaac Lab, LeRobot with the SO-101 arm). Includes a computed forward-kinematics warm-up and links to the labs in this textbook."
aliases:
  - Robotics Labs
  - Robotics Projects
  - Labs Projects and Tutorials (Robotics)
tags:
  - education
  - ros2
type: overview
level: beginner
subfield:
  - creative-education
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Robots]]"
related:
  - "[[Turtlesim_Projects]]"
  - "[[Manipulator_Control_Experiments]]"
  - "[[Sim2Real_Experiments]]"
  - "[[Robotics_Lab_Template]]"
  - "[[Course_Notes_Collection]]"
key_takeaways:
  - "Learn by building in small steps: simulation first, then cheap hardware, then real tasks."
  - "In 2026 the whole path, from first node to a learned manipulation policy, can be done with free software and a low-cost arm such as the SO-101."
  - "Every project should end with numbers and a write-up, so that it teaches you something and others can reproduce it."
sources:
  - "Hugging Face (2026). LeRobot documentation: SO-101 arm. https://huggingface.co/docs/lerobot/so101"
  - "Hugging Face (2026). Robotics course. https://huggingface.co/learn/robotics-course"
  - "Google DeepMind (2026). MuJoCo Playground. https://github.com/google-deepmind/mujoco_playground"
  - "Open Robotics (2026). ROS 2 documentation source: Introducing turtlesim. https://github.com/ros2/ros2_documentation/blob/rolling/source/Get-Started/Introducing-Turtlesim/Introducing-Turtlesim.rst"
---

# Labs, Projects, and Tutorials

This page is a **learning path**: a sequence of projects that build real skills, using tools that are free (or cheap) in 2026. Each step links to the notes that explain the theory.

> **Etymology.** *Laboratory* from Medieval Latin *laboratorium*, "a place for work" (*laborare*, "to work").

## Warm-up: compute before you code

A planar two-link arm has links $l_1 = 0.3$ m and $l_2 = 0.25$ m. With joint angles $\theta_1 = 30°$ and $\theta_2 = 45°$, its tip is at

$$x = l_1\cos\theta_1 + l_2\cos(\theta_1 + \theta_2) = 0.3(0.866) + 0.25(0.259) = 0.325\ \text{m},$$
$$y = l_1\sin\theta_1 + l_2\sin(\theta_1 + \theta_2) = 0.3(0.5) + 0.25(0.966) = 0.391\ \text{m}.$$

Check it in code, then in simulation: three ways of getting the same answer is how you learn to trust each (see [[Forward_Kinematics]]).

## The path

| Stage | Project | Tools | Read |
|---|---|---|---|
| 1. ROS 2 basics | Drive turtles, write a publisher and subscriber, a service, a launch file | ROS 2 Lyrical, turtlesim | [[Turtlesim_Projects]], [[ROS_2_Communication]] |
| 2. A simulated mobile robot | Map a world with SLAM, then navigate with Nav2 | Gazebo (Jetty), SLAM Toolbox, Nav2 | [[SLAM_with_ROS]], [[Nav2_Navigation]] |
| 3. Arm control | PD control, gravity compensation, computed torque | MuJoCo | [[Manipulator_Control_Experiments]] |
| 4. Sim-to-real thinking | Domain randomisation on a simple system | Python, MuJoCo | [[Sim2Real_Experiments]] |
| 5. Reinforcement learning | Train a locomotion or manipulation policy in parallel simulation | MuJoCo Playground or Isaac Lab | [[Reinforcement_Learning_for_Robots]], [[GPU-Parallel_Simulation]] |
| 6. A real arm and imitation learning | Build or buy an SO-101 leader–follower arm, record teleoperated demonstrations, train ACT or a diffusion policy, evaluate | LeRobot | [[Imitation_Learning]], [[Teleoperation_and_Data_Collection]], [[Diffusion_Policy]] |
| 7. Fine-tune a generalist policy | Fine-tune an open VLA on your own demonstrations | LeRobot, open VLA checkpoints | [[Vision-Language-Action_Models]] |

The **SO-101** is an open-source low-cost arm used with LeRobot. Its follower arm uses six Feetech STS3215 servos, and a matching leader arm is moved by hand to teleoperate it.

## Habits that make projects worth doing

- **One change at a time**, with a hypothesis.
- **Measure**: success rates over many trials, not one good video.
- **Record everything** (code version, parameters, data) and write it up (see [[Robotics_Lab_Template]]).
- **Safety**: even small arms pinch fingers; keep an emergency stop within reach.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Labs_Projects_and_Tutorials]])
```
