---
title: Git Integration and Version Control
description: "Version control for robot projects: Git basics and a branching workflow, what belongs in the repository and what does not (with a computed example of why datasets and bags need Git LFS or DVC), managing ROS 2 workspaces with vcstool, continuous integration in simulation, and reproducibility for robot learning in 2026."
aliases:
  - Git
  - Version Control
  - GitHub
tags:
  - git
  - robotics-software
  - software-engineering
type: tutorial
level: beginner
subfield:
  - software
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Custom_Packages_and_Nodes]]"
related:
  - "[[rosbag2]]"
  - "[[Robot_Learning_Datasets_and_Benchmarks]]"
  - "[[Gazebo_Simulator]]"
key_takeaways:
  - "Git records every change to code and configuration so a team can collaborate, review and roll back; a robot's URDF, launch files and parameters belong in it as much as its code."
  - "Large binary data does not: 50 recorded bags of 2 GB each would add 100 GB to every clone; use Git LFS or DVC for datasets, models and meshes."
  - "Continuous integration that builds, lints and runs simulation tests on every change catches regressions before they reach a physical robot."
sources:
  - "Chacon, S. and Straub, B. (2026). Pro Git, 2nd edition. https://git-scm.com/book/en/v2"
  - "Git LFS contributors (2026). Git Large File Storage. https://git-lfs.com/"
  - "Iterative (2026). DVC: data version control. https://dvc.org/"
---

# Git Integration and Version Control

**Version control** records every change to a project so that people can work together, see who changed what and why, and return to any earlier state. **Git** is the standard. For robots, "the project" means more than code: robot descriptions, calibration, parameters, launch files, simulation worlds and, separately, data and trained models.

> **Etymology.** Linus Torvalds wrote Git in 2005 for Linux kernel development; he joked that he names projects after himself (*git* is British slang for an unpleasant person). GitHub launched in 2008.

## Everyday workflow

```bash
git switch -c fix/gripper-timeout            # work on a branch
# ...edit, build, test...
git add -p                                   # stage changes in reviewable pieces
git commit -m "Gripper: retry once on CAN timeout"
git push -u origin fix/gripper-timeout       # open a pull request for review
```

Small, focused commits with clear messages, reviewed through pull requests, are the habit that matters most.

## What goes where

| Content | Where |
|---|---|
| Source code, URDF/Xacro, launch files, parameters, small configs | Git |
| Calibration files | Git (versioned, with robot serial numbers) |
| Meshes and textures | Git LFS if large |
| Datasets, rosbags, trained model weights | Git LFS, DVC, or an artefact store, referenced by version from Git |
| Secrets (API keys, passwords) | Never in Git; use a secrets manager or environment |

## Worked example: why data stays out of plain Git

A robot-learning project records 50 demonstration bags of 2 GB each: 100 GB. Committed directly, every clone of the repository downloads all 100 GB, forever, including after the files are deleted (they remain in history). Git LFS stores large files on a separate server and keeps only small pointers in Git. DVC also tracks versions of datasets and models, links them to the code and parameters that produced them, and stores them in cloud or local storage. Either way, a commit identifies exactly which data trained which policy.

## ROS 2 workspaces

A workspace usually combines your packages with forks of others. **vcstool** (`vcs import < robot.repos`) checks out all repositories at pinned versions, and `rosdep` installs system dependencies. Pinning versions makes a robot's software reproducible.

## Continuous integration

On every pull request, CI should:
- build (`colcon build`);
- run linters and unit tests (`colcon test`);
- for robots, run **simulation tests**: launch the robot in [[Gazebo_Simulator|Gazebo]] or MuJoCo headless and check that it reaches a goal or a policy succeeds on a set of scenarios.

Catching a broken controller in CI is far cheaper than catching it on hardware.

## Reproducibility for robot learning (2026)

Track together the code commit, the dataset version, the training configuration and random seeds, the simulator version, and the evaluation results. Tools such as DVC, experiment trackers and Hugging Face Hub model and dataset versions make "which data and code produced this policy?" answerable, which is essential when a deployed policy misbehaves.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Git_Integration_and_Version_Control]])
```
