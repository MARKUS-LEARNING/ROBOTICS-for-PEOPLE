---
title: Research Papers Index
description: A curated index of seminal and influential research papers in robotics, organized by topic, with a paper note template and guide for building a personal research library within this vault.
tags:
  - index
  - research
  - papers
  - reading-list
  - references
  - bibliography
  - literature
  - SLAM
  - manipulation
  - reinforcement-learning
  - navigation
layout: default
category: robotics
author: Jordan_Smith_&_Claude
date: 2025-05-02
permalink: /research_papers_index/
related:
  - "[[Resources_Index]]"
  - "[[SLAM_with_ROS]]"
  - "[[Reinforcement_Learning_(RL)]]"
  - "[[Nav2_Navigation]]"
  - "[[MoveIt2]]"
  - "[[Computer_Vision_in_Robotics]]"
  - "[[MOOCs_and_Courses]]"
---

# Research Papers Index

A curated index of landmark and influential research papers in robotics, organized by topic. Use this as a reading guide and a scaffold for building your own paper notes within this vault. Papers marked **⭐ Landmark** are widely considered foundational to their field.

See also: [[Resources_Index]], [[MOOCs_and_Courses]]

---

## Paper Note Template

Create individual notes for each paper you read (e.g., `[[ORB-SLAM3_Notes]]`) using this template:

```markdown
---
title: "[Paper Title]"
authors: [Author 1, Author 2]
venue: "ICRA 2021"
year: 2021
tags: [research-paper, SLAM, visual-odometry]
link: "https://arxiv.org/abs/2007.11898"
status: "Read"   # To Read | Reading | Read | Summarized
date_read: 2025-05-01
rating: 5
related:
  - "[[Research_Papers_Index]]"
  - "[[SLAM_with_ROS]]"
---

# [Paper Title]

**Authors:** | **Venue:** | **Year:** | **Link:**

## Summary
[1–3 sentence plain-language summary of what the paper does and why it matters]

## Key Contributions
- [Main innovation 1]
- [Main innovation 2]

## Method
[Brief description of the technical approach]

## Results
[Key benchmarks or experiments]

## Personal Notes
[Your questions, connections to other work, critiques]

## BibTeX
\`\`\`bibtex
@inproceedings{key2021,
  author    = {},
  title     = {},
  booktitle = {},
  year      = {2021},
  doi       = {},
}
\`\`\`
```

---

## Seminal Papers by Topic

### SLAM and Localization

| Paper | Authors | Venue | Notes |
|---|---|---|---|
| **⭐ A Solution to the Simultaneous Localization and Map Building (SLAM) Problem** | Dissanayake et al. | ICRA 2001 | Original EKF-SLAM formulation |
| **⭐ Probabilistic Robotics** | Thrun, Burgard, Fox | MIT Press 2005 | Textbook; definitive treatment of Bayesian localization and SLAM |
| **⭐ FastSLAM: A Factored Solution to the Simultaneous Localization and Mapping Problem** | Montemerlo et al. | AAAI 2002 | Particle filter SLAM; scales to large maps |
| **⭐ ORB-SLAM: A Versatile and Accurate Monocular SLAM System** | Mur-Artal et al. | T-RO 2015 | Feature-based visual SLAM; landmark system |
| **ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM** | Campos et al. | T-RO 2021 | State-of-the-art visual-inertial SLAM |
| **⭐ LOAM: Lidar Odometry and Mapping in Real-time** | Zhang & Singh | RSS 2014 | Foundational LiDAR SLAM |
| **LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping** | Shan et al. | IROS 2020 | LiDAR + IMU tightly coupled; widely deployed |
| **KISS-ICP: In Defense of Point-to-Point ICP – Simple, Accurate, and Robust Mobile Robot Odometry** | Vizzo et al. | RA-L 2023 | Surprisingly effective simple LiDAR odometry |
| **Cartographer: Real-Time Loop Closure in 2D LIDAR SLAM** | Hess et al. | ICRA 2016 | Google's submap-based SLAM; open-source |

---

### Navigation and Path Planning

| Paper | Authors | Venue | Notes |
|---|---|---|---|
| **⭐ A* Search Algorithm** | Hart, Nilsson, Raphael | IEEE Trans. 1968 | Foundational heuristic search |
| **⭐ Probabilistic Roadmaps for Path Planning in High-Dimensional Configuration Spaces** | Kavraki et al. | T-RO 1996 | PRM — sampling-based planning |
| **⭐ RRT: Rapidly-Exploring Random Trees** | LaValle | Tech Report 1998 | Foundational sampling-based motion planner |
| **Sampling-Based Algorithms for Optimal Motion Planning (RRT*)** | Karaman & Frazzoli | IJRR 2011 | Asymptotically optimal RRT |
| **The Dynamic Window Approach to Collision Avoidance** | Fox, Burgard, Thrun | IEEE RA Magazine 1997 | DWA local planner; basis of ROS DWB |
| **Nav2: A Production-Grade Lifecycle Robot Autonomy System** | Macenski et al. | IROS 2020 | Nav2 architecture paper |
| **Behavior Trees in Robotics and AI** | Colledanchise & Ögren | CRC Press 2018 | Textbook; basis of Nav2 BT Navigator |

---

### Manipulation and Kinematics

| Paper | Authors | Venue | Notes |
|---|---|---|---|
| **⭐ A Mathematical Introduction to Robotic Manipulation** | Murray, Li, Sastry | CRC Press 1994 | Screw theory, Lie groups for kinematics; free PDF |
| **⭐ Modern Robotics: Mechanics, Planning, and Control** | Lynch & Park | Cambridge 2017 | The modern standard textbook; free PDF |
| **MoveIt: An Introduction** | Chitta et al. | IEEE RA Magazine 2016 | Original MoveIt paper |
| **⭐ Dexterous Manipulation from Images: Autonomous and Image-Conditioned vs. Purely Autonomous Learning** | Levine et al. | IJRR 2016 | GPS algorithm; end-to-end manipulation learning |
| **Learning Hand-Eye Coordination for Robotic Grasping with Deep Learning and Large-Scale Data Collection** | Levine et al. | IJRR 2018 | Google's large-scale grasping study |
| **Dexterity from Touch: Self-Supervised Pre-Training of Tactile Representations with Robotic Play** | Guzey et al. | CoRL 2023 | Tactile sensing for dexterous manipulation |
| **UniSim: Learning Interactive Real-World Simulators** | Yang et al. | ICLR 2024 | Generative world model for robot training |

---

### Robot Learning and Reinforcement Learning

| Paper | Authors | Venue | Notes |
|---|---|---|---|
| **⭐ Reinforcement Learning: An Introduction** | Sutton & Barto | MIT Press 2018 | The RL textbook; free PDF |
| **⭐ Human-Level Control Through Deep Reinforcement Learning (DQN)** | Mnih et al. | Nature 2015 | Deep RL breakthrough; Atari games |
| **⭐ Proximal Policy Optimization Algorithms (PPO)** | Schulman et al. | arXiv 2017 | Widely used policy gradient; standard baseline |
| **⭐ Soft Actor-Critic: Off-Policy Maximum Entropy Deep RL (SAC)** | Haarnoja et al. | ICML 2018 | State-of-the-art continuous control RL |
| **Learning to Walk in Minutes Using Massively Parallel Deep RL** | Rudin et al. | CoRL 2022 | Isaac Gym locomotion; GPU-accelerated RL |
| **Legged Locomotion in Challenging Terrains using Egocentric Vision** | Kumar et al. | CoRL 2021 | Parkour-style locomotion with vision |
| **⭐ Learning Agile and Dynamic Motor Skills for Legged Robots** | Hwangbo et al. | Science Robotics 2019 | Anymal locomotion RL with actuator network |
| **RT-1: Robotics Transformer for Real-World Control at Scale** | Brohan et al. | RSS 2023 | Large-scale robot learning transformer |
| **RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control** | Brohan et al. | CoRL 2023 | VLA model; web-pretrained for robot control |
| **Open X-Embodiment: Robotic Learning Datasets and RT-X Models** | Open X-Embodiment Collaboration | ICRA 2024 | Cross-embodiment training dataset & model |

---

### Imitation Learning and Foundation Models

| Paper | Authors | Venue | Notes |
|---|---|---|---|
| **⭐ A Reduction of Imitation Learning and Structured Prediction to No-Regret Online Learning (DAgger)** | Ross et al. | AISTATS 2011 | Dataset aggregation for imitation learning |
| **Behavior Cloning from Observation** | Torabi et al. | IJCAI 2018 | Imitation from observations without actions |
| **Action Chunking with Transformers (ACT)** | Zhao et al. | RSS 2023 | Bimanual manipulation with diffusion-like chunking |
| **Diffusion Policy: Visuomotor Policy Learning via Action Diffusion** | Chi et al. | RSS 2023 | Diffusion models for robot policy; strong baseline |
| **π0: A Vision-Language-Action Flow Model for General Robot Control** | Black et al. | arXiv 2024 | Physical Intelligence foundation model |

---

### Computer Vision and Perception

| Paper | Authors | Venue | Notes |
|---|---|---|---|
| **⭐ ImageNet Classification with Deep CNNs (AlexNet)** | Krizhevsky et al. | NeurIPS 2012 | Deep learning in vision |
| **⭐ Deep Residual Learning for Image Recognition (ResNet)** | He et al. | CVPR 2016 | Skip connections; dominant backbone architecture |
| **⭐ You Only Look Once: Unified Real-Time Object Detection (YOLO)** | Redmon et al. | CVPR 2016 | Real-time detection; widely deployed in robotics |
| **⭐ Attention Is All You Need (Transformer)** | Vaswani et al. | NeurIPS 2017 | Transformer architecture; basis of VLA models |
| **PointNet: Deep Learning on Point Sets for 3D Classification and Segmentation** | Qi et al. | CVPR 2017 | Direct learning on 3D point clouds |
| **Segment Anything (SAM)** | Kirillov et al. | ICCV 2023 | Foundation model for image segmentation |

---

### Multi-Robot Systems and Swarms

| Paper | Authors | Venue | Notes |
|---|---|---|---|
| **⭐ Swarm Intelligence: From Natural to Artificial Systems** | Bonabeau, Dorigo, Theraulaz | Oxford 1999 | Foundational swarm robotics text |
| **Task Allocation for Multi-Robot Systems** | Gerkey & Matarić | IJRR 2004 | Taxonomy and analysis of multi-robot task allocation |
| **Flocking for Multi-Agent Dynamic Systems** | Olfati-Saber | IEEE TAC 2006 | Distributed consensus and flocking algorithms |

---

## Where to Find Papers

| Resource | URL | Best For |
|---|---|---|
| **arXiv** | arxiv.org/list/cs.RO | Preprints; free access to nearly all robotics papers |
| **IEEE Xplore** | ieeexplore.ieee.org | ICRA, IROS, T-RO, RA-L official proceedings |
| **Semantic Scholar** | semanticscholar.org | Citation network, free PDFs, paper discovery |
| **Google Scholar** | scholar.google.com | Broad search; citation tracking |
| **Papers With Code** | paperswithcode.com | Papers linked to open-source code |
| **Hugging Face Papers** | huggingface.co/papers | Daily arXiv highlights; community discussion |
| **Connected Papers** | connectedpapers.com | Visual citation graph for literature exploration |
| **ACM DL** | dl.acm.org | HRI, RSS proceedings |

---

## Major Robotics Conferences

| Conference | Abbreviation | Scope |
|---|---|---|
| Int'l Conf on Robotics and Automation | ICRA | Broad robotics; largest venue |
| Intelligent Robots and Systems | IROS | Broad robotics |
| Robotics: Science and Systems | RSS | High selectivity; high impact |
| Conference on Robot Learning | CoRL | Learning-based robotics |
| Humanoids | Humanoids | Legged and humanoid robots |
| Human-Robot Interaction | HRI | HRI and social robotics |

---

## Dynamic Index with Dataview

Add `#research-paper` to any paper note to automatically include it in this index:

```dataview
TABLE venue AS "Venue", authors AS "Authors", status AS "Status", rating AS "Rating"
FROM ""
WHERE contains(file.tags, "research-paper")
SORT year DESC, title ASC
```

```dataview
LIST FROM #research-paper WHERE status = "To Read"
SORT file.cday DESC
```
