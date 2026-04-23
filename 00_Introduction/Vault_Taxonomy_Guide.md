---
title: Vault Taxonomy Guide
description: The organizational structure (folders, tags, linking) of the ROBOTICS-for-PEOPLE vault.
tags:
  - guide
  - taxonomy
  - organization
  - vault-meta
layout: default
category: robotics
author: Jordan_Smith
date: 2026-04-23
permalink: /vault_taxonomy_guide/
related:
  - "[[How_to_Use_This_Vault]]"
  - "[[Robotics_Vault_Dashboard]]"
  - "[[Resources_Index]]"
---

# Vault Taxonomy Guide

This guide explains the organizational principles of the ROBOTICS-for-PEOPLE vault: the folder tree, the tagging vocabulary, and the linking strategy. Following these conventions keeps the graph coherent as the vault grows.

---

## Folder Structure

The vault uses a flat, numbered folder system. Numbers imply a suggested reading order for newcomers; they are not strict dependencies.

| Folder | Scope |
|---|---|
| `00_Introduction` | Vault meta: usage guide, taxonomy, dashboard, resources. |
| `01_Fundamentals_of_Robotics` | Definitions, history, ethics, industry landscape, safety. |
| `02_Mathematics_for_Robotics` | Linear algebra, calculus, probability, Lie groups SO(3)/SE(3), optimization. |
| `03_Kinematics_and_Dynamics` | Forward/inverse kinematics, DH parameters, Jacobians, Newton–Euler, Lagrangian formulations. |
| `04_Sensors_and_Perception` | Cameras, LiDAR, IMUs, encoders, sensor fusion, ICP, NDT. |
| `05_AI_and_Machine_Learning` | Classical ML, deep learning, reinforcement learning, imitation learning, VLAs (Vision-Language-Action models). |
| `06_Robot_Control` | PID, state-space, MPC (Model Predictive Control), adaptive, robust, impedance, HRI (Human-Robot Interaction). |
| `07_Robot_Operating_System_(ROS)` | ROS 2 (Humble/Iron/Jazzy), DDS, topics, services, actions, lifecycle nodes, TF, Gazebo. |
| `08_Robot_Types_and_Applications` | Industrial arms, mobile, humanoid, drones, medical, soft, swarm, exoskeletons, etc. |
| `09_Advanced_Topics` | Multi-agent, manipulation, grasping, tactile, neuromorphic, quantum, ethics deep-dive. |
| `10_Research_and_Development` | Reading papers, benchmarks, conferences (ICRA, IROS, RSS, CoRL), reproducibility. |
| `11_Practical_Implementation` | Dev environments, Docker, CI/CD, sim-to-real, rosbag2, debugging. |
| `12_Labs_and_Tutorials` | Hands-on exercises: Turtlesim, Sim2Real, URDF (Unified Robot Description Format) authoring. |
| `13_Tools_References_and_Links` | MOOCs, papers index, simulators, IDEs, hardware recommendations. |

---

## Tagging Strategy

Tags supplement folders with orthogonal metadata. A note on PID tuning sits in `06_Robot_Control`, but its tags (`#control-theory`, `#pid`, `#tuning`) connect it to every other note that cares about those dimensions.

**Specific concepts** — `#kinematics`, `#slam`, `#computer-vision`, `#reinforcement-learning`, `#sensor-fusion`, `#control-theory`, `#optimization`

**Robot types and components** — `#mobile-robot`, `#manipulator-arm`, `#humanoid`, `#lidar`, `#imu`, `#actuator`, `#camera`, `#encoder`

**Software and tools** — `#ros`, `#ros2`, `#python`, `#cpp`, `#rust`, `#cuda`, `#gazebo`, `#rviz`, `#opencv`, `#pcl`, `#eigen`

**Document type** — `#tutorial`, `#project-log`, `#research-paper`, `#course-notes`, `#concept`, `#example-code`

**Status and meta** — `#todo`, `#in-progress`, `#review-needed`, `#stub`, `#vault-meta`, `#guide`

**Rule of thumb.** Prefer reusing an existing tag over inventing a new one. Check the Obsidian Tags pane before adding a tag — if the concept is already covered under a different spelling (e.g., `#slam` vs `#SLAM`), normalize to the existing form.

The legacy tag `#glossary-term` is being retired. Files that outgrew pure glossary entries should drop it.

---

## Linking Strategy

Wikilinks are what turn a collection of notes into a knowledge graph. Four guidelines keep the graph useful.

**Link on first mention.** The first time a file references "PID Control" or "SLAM", link it: `[[PID_Control]]`, `[[SLAM]]`. After that, use the term freely without re-linking.

**Link across chapters.** The deepest graph edges cross folder boundaries. A note on [[Forward_Kinematics]] (in `03_Kinematics_and_Dynamics`) should link to **Linear Algebra** (in `02_Mathematics_for_Robotics`) and to the **TF** transform library (in `07_Robot_Operating_System_(ROS)`). A note on [[Autonomous_Navigation]] (in `06_Robot_Control`) should link to [[SLAM]] (in `04_Sensors_and_Perception`) and [[Reinforcement_Learning]] (in `05_AI_and_Machine_Learning`). (The bold names above are stubs scheduled for later PRs — do not link to them until they exist.)

**Populate `related:` frontmatter.** Every note carries a `related:` list of 2–5 adjacent topics. This creates graph edges without cluttering prose. Example (for a note on inverse kinematics):

```yaml
related:
  - "[[Forward_Kinematics]]"
  - "[[Jacobian_Matrix]]"
  - "[[Degrees_of_Freedom]]"
```

**Do not link to nonexistent files.** Broken wikilinks pollute `MetadataCache.unresolvedLinks` and show as dimmed nodes in Graph View. Either create the target as a stub or drop the link.

---

## Filename Conventions

- **Case**: `Title_Case_With_Underscores.md`.
- **Acronyms in filenames**: keep uppercase (`SLAM.md`, `PID_Control.md`, `URDF.md`).
- **No spaces, no punctuation** other than underscores.
- **No duplicate filenames** anywhere in the vault — wikilinks resolve by filename, so duplicates create ambiguity.

---

## Frontmatter Schema

Every content note carries YAML frontmatter:

```yaml
---
title: Human-readable title
description: One-sentence summary of what the note covers.
tags:
  - primary-topic
  - secondary-topic
category: robotics
author: Jordan_Smith
date: YYYY-MM-DD
related:
  - "[[Adjacent_Topic_1]]"
  - "[[Adjacent_Topic_2]]"
---
```

**Required**: `title`, `tags`, `author`, `date`.
**Strongly recommended**: `description`, `related`.
**Author**: always `Jordan_Smith` — do not add co-authors or AI model names.

---

*This guide is itself part of the vault-meta layer. Keep it in sync whenever folder names change or new tagging conventions emerge.*
