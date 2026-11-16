---
title: Robotics Lab Template
description: "A template and guide for planning, running and writing up robotics labs so results are trustworthy and reproducible: question and hypothesis, setup with versions and seeds, safety, procedure, measurement with enough trials (with a computed confidence-interval example for success rates), results, and discussion, adapted for simulation, hardware and robot-learning experiments in 2026."
aliases:
  - Lab Template
  - Lab Report Template
tags:
  - education
  - research
type: tutorial
level: beginner
subfield:
  - research-practice
  - creative-education
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Labs_Projects_and_Tutorials]]"
related:
  - "[[Manipulator_Control_Experiments]]"
  - "[[Git_Integration_and_Version_Control]]"
  - "[[Robot_Learning_Datasets_and_Benchmarks]]"
key_takeaways:
  - "A good lab starts with a question and a prediction, changes one thing at a time, and records everything needed to repeat it."
  - "Success rates need enough trials: 8 successes in 10 trials has a 95% interval of roughly 49–94%, so small differences between policies are usually noise."
  - "For robot learning, report the code commit, data version, seeds, simulator or hardware details, and evaluation protocol, not just a video."
sources:
  - "Hugging Face (2026). LeRobot documentation. https://huggingface.co/docs/lerobot/so101"
  - "Chacon, S. and Straub, B. (2026). Pro Git, 2nd edition. https://git-scm.com/book/en/v2"
  - "Iterative (2026). DVC: data version control. https://dvc.org/"
---

# Robotics Lab Template

Use this template for any experiment, in simulation or on hardware. Its purpose is that your result is believable to someone else, and to you in six months.

> **Etymology.** *Template* from French *templet*, a pattern used as a guide.

## The template

```markdown
---
title: "Lab: <short descriptive title>"
type: lab
level: intermediate
subfield: [control]
status: draft
tags: [control, mujoco]
---

# Lab: <title>

## Question and prediction
What do you want to know? What do you expect, and why?

## Setup
- Robot / model (URDF or MJCF file and version)
- Software: OS, ROS 2 distribution, simulator version, library versions
- Code commit and branch; dataset version; random seeds
- Hardware: controllers, sensors, calibration date

## Safety
Hazards and how they are controlled (e-stop location, speed and force limits, guarding).

## Procedure
Numbered steps another person could follow. Change one variable at a time.

## Measurements
What is measured, how, how often, and how many trials.

## Results
Tables and plots with numbers and uncertainty. Link raw data.

## Discussion
Was the prediction right? What else could explain the result? Limitations.

## Next steps
The next question this raises.
```

## How many trials?

Robot experiments are noisy, so report uncertainty. For a success rate, a simple 95% interval (the Wilson score interval) for 8 successes out of 10 trials is roughly **49% to 94%**. With 80 successes out of 100 it narrows to about 71–87%.

So "policy A succeeded 8/10 times and policy B 7/10" does **not** show that A is better. Plan enough trials for the difference you care about, and randomise conditions (object positions, lighting) the same way for both.

## Extra items for robot-learning labs

- Training data: source, size, version, and how it was split.
- Training configuration: hyperparameters, compute, training time, seeds.
- Evaluation protocol: number of episodes, initial conditions, what counts as success, held-out objects and scenes.
- Failure analysis: categorise failures (perception, grasp, collision, timeout).
- Artefacts: model checkpoint and evaluation videos linked from the write-up.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Robotics_Lab_Template]])
```
