---
title: Research and Development
description: "Where robotics research is heading as of late 2026 and how to do it well: the open problems (generalist manipulation, data, evaluation, safety, hardware cost and reliability, long-horizon autonomy), how research moves from paper to product, reading and reproducing papers, avoiding false discoveries (a computed multiple-comparisons example), and where to follow the field."
aliases:
  - Robotics Research
  - R&D
  - Research and Development (Robotics)
tags:
  - research
  - research-and-development
  - future-of-robotics
type: overview
level: intermediate
subfield:
  - research-practice
  - frontiers
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Robots]]"
related:
  - "[[Advanced_AI_Learning_for_Robotics]]"
  - "[[Research_Papers_Index]]"
  - "[[Robotics_Lab_Template]]"
  - "[[Robot_Learning_Datasets_and_Benchmarks]]"
  - "[[Ethical_and_Social_Implications]]"
key_takeaways:
  - "The central open problems in 2026 are generalisation and reliability of learned robot skills, data at scale, trustworthy evaluation, safety, and the cost and durability of hardware."
  - "Testing 20 independent ideas at p < 0.05 gives a 64% chance of at least one false 'discovery'; pre-registration, held-out evaluation and enough trials guard against it."
  - "Research moves to products faster than ever (months, not decades), so reproducible code, data and honest evaluation matter for industry as much as academia."
sources:
  - "Stanford HAI (2026). The 2026 AI Index Report. https://hai.stanford.edu/ai-index/2026-ai-index-report"
  - "International Federation of Robotics (2026). Five Million Robots now Operate in Factories Globally (World Robotics 2026). https://ifr.org/ifr-press-releases/news/five-million-robots-now-operate-in-factories-globally"
  - "Physical Intelligence (2026). π0.7: a Steerable Generalist Robotic Foundation Model with Emergent Capabilities. https://arxiv.org/abs/2604.15483"
  - "NVIDIA, Mittal, M. et al. (2025). Isaac Lab: A GPU-Accelerated Simulation Framework for Multi-Modal Robot Learning. https://arxiv.org/abs/2511.04831"
---

# Research and Development

Robotics research in 2026 is moving faster than at any time in its history: foundation models for robot control, GPU-parallel simulation, humanoids leaving labs, and investment pouring in. This page maps the open problems and describes how to do research that holds up.

> **Etymology.** *Research* from Old French *recercher*, "to search closely"; *development*, from *développer*, "to unfold". R&D is the path from discovery to working systems.

## Open problems (late 2026)

| Problem | Why it is hard | See |
|---|---|---|
| **Generalist manipulation** | Policies that handle any object and task reliably, not just demos | [[Vision-Language-Action_Models]] |
| **Data** | Robot data is scarce and costly; human video and simulation differ from robot experience | [[Robot_Learning_Datasets_and_Benchmarks]], [[Teleoperation_and_Data_Collection]] |
| **Evaluation** | Real-robot tests are slow and hard to compare; most headline numbers are self-reported | [[Robotics_Lab_Template]] |
| **Long-horizon autonomy** | Small per-step errors compound over long tasks | [[Autonomous_Robots]] |
| **Dexterity and touch** | Hands, tactile sensing and contact-rich control | [[Robot_Hands]] |
| **Safety and verification** | Guarantees for learned behaviour near people | [[Formal_Methods_in_Control]], [[Ethical_and_Social_Implications]] |
| **Hardware** | Cost, durability, energy and repairability of humanoids and hands | [[Humanoid_Robots]] |
| **Sim-to-real** | Remaining gaps in contact, deformables and sensing | [[Sim-to-Real_Transfer]] |

## From paper to product

Methods published as arXiv preprints now reach products within months: generalist robot foundation models (the π0 family, GR00T, Gemini Robotics), GPU simulation frameworks (Isaac Lab, MuJoCo Playground), and learned locomotion on commercial quadrupeds and humanoids. Open-source code and models, shared datasets and common tools (ROS 2, LeRobot) speed the transfer. The flip side is pressure to publish quickly, which makes careful evaluation more important.

## Doing research that holds up

**Worked example: false discoveries.** Test 20 independent ideas, each at the conventional 5% significance level, when none actually works. The chance that at least one *looks* significant is $1 - 0.95^{20} = 0.64$: a 64% chance of a false discovery. In robot learning, trying many variants and reporting the best one is the same trap. Defences:
- decide the evaluation protocol before looking at results;
- hold out test conditions (objects, scenes, robots) and touch them once;
- run enough trials and report uncertainty (see [[Robotics_Lab_Template]]);
- report failures and ablations, not just the best run;
- release code, data and models so others can reproduce.

## Following the field

- **Preprints**: arXiv cs.RO, cs.LG and cs.CV.
- **Conferences**: ICRA, IROS, RSS, CoRL, Humanoids, and machine-learning venues (NeurIPS, ICML, ICLR).
- **Journals**: Science Robotics, IJRR, IEEE T-RO, RA-L.
- **Reports**: the IFR's *World Robotics* (industry numbers), the Stanford AI Index (AI trends).
- **This vault**: [[Research_Papers_Index]], and the generated Frontier and What's New pages.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Research_and_Development]])
```
