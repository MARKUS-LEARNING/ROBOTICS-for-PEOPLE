---
title: Deep Learning in Robotics
description: "Section overview: where deep neural networks sit in a robot today, in perception, state estimation, control, and end-to-end learned policies, and how the field moved from task-specific networks to robot foundation models."
tags:
  - deep-learning
  - robot-learning
  - neural-networks
  - computer-vision
  - foundation-models
type: overview
level: intermediate
subfield:
  - robot-learning
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Deep_Learning]]"
related:
  - "[[Reinforcement_Learning_for_Robots]]"
  - "[[Imitation_Learning]]"
  - "[[Vision-Language-Action_Models]]"
  - "[[Computer_Vision]]"
  - "[[Neural_Networks_in_Control]]"
key_takeaways:
  - "Deep learning entered robots through perception, then moved into control and finally into end-to-end policies."
  - "Today's stack mixes learned and classical parts: learned perception and high-level policies on top of classical estimation, planning and low-level control."
  - "The 2023-26 shift is from one network per task to pretrained generalist models adapted per robot."
sources:
  - "Levine, S. et al. (2015). End-to-End Training of Deep Visuomotor Policies. https://arxiv.org/abs/1504.00702"
  - "Brohan, A. et al. (2022). RT-1: Robotics Transformer for Real-World Control at Scale. https://arxiv.org/abs/2212.06817"
  - "Chi, C. et al. (2023). Diffusion Policy: Visuomotor Policy Learning via Action Diffusion. https://arxiv.org/abs/2303.04137"
  - "Rudin, N. et al. (2021). Learning to Walk in Minutes Using Massively Parallel Deep Reinforcement Learning. https://arxiv.org/abs/2109.11978"
---

# Deep Learning in Robotics

**Deep learning** (training many-layered [[Neural_Networks|neural networks]] on data) entered robotics through the eyes and has been moving toward the hands ever since. This overview maps where deep networks sit in a robot in 2026 and points to the notes that go deeper.

> **Etymology.** *Deep* refers to the number of layers between input and output. The phrase "deep learning" was introduced to machine learning by Rina Dechter (1986) and to neural networks by Igor Aizenberg and colleagues (2000); it became the field's name after Geoffrey Hinton's 2006 work on training deep belief networks.

## Where deep networks sit in a robot

| Part of the stack | What the network does | Classical alternative it competes with | Notes |
|---|---|---|---|
| **Perception** | Detect, segment and track objects; estimate depth and pose | Hand-designed features, geometric vision | [[Computer_Vision]], [[Convolutional_Neural_Networks]] |
| **State estimation** | Learned odometry, learned sensor models | Kalman filters, factor graphs | Usually hybrid; see [[Sensor_Fusion]] |
| **Dynamics models** | Learn how the robot or objects move | Rigid-body models | [[Neural_Networks_in_Control]], [[World_Models]] |
| **Low-level control** | Locomotion policies trained in simulation | PD, MPC | [[Reinforcement_Learning_for_Robots]] |
| **Manipulation policies** | Map camera images to arm and gripper motion | Grasp planners, scripted skills | [[Imitation_Learning]], [[Diffusion_Policy]] |
| **Task-level reasoning** | Understand instructions, plan steps | Symbolic planners | [[Vision-Language-Action_Models]] |

## Three eras

1. **Perception (2012–).** After deep convolutional networks won the 2012 ImageNet challenge, robots adopted learned object detection and segmentation, while control stayed classical.
2. **End-to-end skills (2015–).** Levine et al. (2015) trained a single network from camera pixels to motor torques for tasks like screwing a cap on a bottle. Simulation-trained locomotion followed (Rudin et al. 2021). Each skill still had its own network.
3. **Generalist models (2022–).** RT-1 (2022) trained one transformer on 130,000 real episodes; [[Diffusion_Policy]] (2023) made imitation reliable; [[Vision-Language-Action_Models]] connected web-scale knowledge to action. In 2026 the frontier is generalist models pretrained across many robots and adapted per robot.

## The hybrid reality

Almost no deployed robot is learned end to end. A typical 2026 system:

```
cameras ─► learned perception / VLA policy (10–50 Hz) ─► targets ─► classical controller (500–1000 Hz) ─► motors
                 ▲                                                     ▲
   classical localisation / maps                          safety limits, collision checks
```

Learned parts handle what is hard to write down (recognising a crumpled shirt, choosing a grasp); classical parts handle what must be guaranteed (joint limits, stability, emergency stops).

## Where to go next

- How networks learn from reward: [[Reinforcement_Learning]] → [[Reinforcement_Learning_for_Robots]].
- How they learn from demonstrations: [[Imitation_Learning]] → [[Diffusion_Policy]] → [[Vision-Language-Action_Models]].
- How they learn to predict the world: [[Generative_Models]] → [[World_Models]].

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Deep_Learning_in_Robotics]])
```
