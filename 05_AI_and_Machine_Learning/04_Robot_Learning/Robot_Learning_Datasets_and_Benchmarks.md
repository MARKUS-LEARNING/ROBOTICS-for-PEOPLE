---
title: Robot Learning Datasets and Benchmarks
description: "The shared data and tests of robot learning: large multi-robot datasets (Open X-Embodiment, DROID), dataset formats (LeRobot), simulation benchmarks (LIBERO, ManiSkill) and real-world evaluation, plus why measuring robot policies is harder than measuring language models."
aliases:
  - Robot Learning Datasets
  - Robot Learning Benchmarks
tags:
  - dataset
  - benchmarks
  - robot-learning
  - imitation-learning
  - simulation
type: reference
level: intermediate
subfield:
  - robot-learning
  - research-practice
status: reviewed
author: Jordan_Smith
date: 2026-11-15
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Imitation_Learning]]"
  - "[[Teleoperation_and_Data_Collection]]"
related:
  - "[[Vision-Language-Action_Models]]"
  - "[[Research_and_Development]]"
  - "[[Simulators_and_IDEs]]"
key_takeaways:
  - "Pooling data across labs and robots (Open X-Embodiment, DROID) made cross-embodiment generalist policies possible."
  - "Simulation benchmarks (LIBERO, ManiSkill, RLBench, CALVIN) are cheap and reproducible but saturate and differ from reality."
  - "Real-world evaluation needs many trials, fixed protocols and honest reporting; a single success video proves little."
sources:
  - "Open X-Embodiment Collaboration (2023). Open X-Embodiment: Robotic Learning Datasets and RT-X Models. https://arxiv.org/abs/2310.08864"
  - "Khazatsky, A. et al. (2024). DROID: A Large-Scale In-The-Wild Robot Manipulation Dataset. https://arxiv.org/abs/2403.12945"
  - "Walke, H. et al. (2023). BridgeData V2: A Dataset for Robot Learning at Scale. https://arxiv.org/abs/2308.12952"
  - "Liu, B. et al. (2023). LIBERO: Benchmarking Knowledge Transfer for Lifelong Robot Learning. https://arxiv.org/abs/2306.03310"
  - "Tao, S. et al. (2024). ManiSkill3: GPU Parallelized Robotics Simulation and Rendering for Generalizable Embodied AI. https://arxiv.org/abs/2410.00425"
  - "James, S. et al. (2019). RLBench: The Robot Learning Benchmark & Learning Environment. https://arxiv.org/abs/1909.12271"
  - "Mees, O. et al. (2021). CALVIN: A Benchmark for Language-Conditioned Policy Learning for Long-Horizon Robot Manipulation Tasks. https://arxiv.org/abs/2112.03227"
  - "Hugging Face (2026). LeRobot v0.7.0 release notes. https://github.com/huggingface/lerobot/releases/tag/v0.7.0"
---

# Robot Learning Datasets and Benchmarks

Language models learned from the internet. Robots have no internet of actions: every (observation, action) pair has to be recorded on a real robot or generated in simulation. **Robot learning datasets** are the shared pools of that data; **benchmarks** are the shared tests used to compare policies. Both shape the field as much as any algorithm.

> **Etymology.** *Data* is the plural of Latin *datum*, "a thing given". *Benchmark* comes from surveying: a mark cut into stone, often a horizontal line with a broad arrow beneath it, where a levelling staff could be set on a "bench" bracket to give a fixed reference height. A benchmark in computing is likewise a fixed reference point to measure against.

## Major datasets

| Dataset | Year | What is in it | Why it matters |
|---|---|---|---|
| **BridgeData V2** | 2023 | About 60,000 trajectories on a low-cost WidowX arm in many kitchens and toy scenes | Diverse, open, easy to reproduce |
| **Open X-Embodiment** | 2023 | Over 1 million trajectories pooled from 22 robot types, contributed by 21 institutions | Showed that training on many robots helps each robot (RT-X) |
| **DROID** | 2024 | About 76,000 trajectories, roughly 350 hours, in 564 scenes on Franka arms | "In-the-wild" diversity of scenes and objects |
| **LeRobot Hub datasets** | 2024– | Thousands of community datasets in a common format | Low barrier to sharing; standard loader |

The trend since 2024: vendors train on far larger private mixes that add handheld-gripper data and many thousands of hours of human video (see [[Teleoperation_and_Data_Collection]]).

## Dataset formats

Robot data is multimodal and time-synchronised: several camera streams, joint states, actions, language and metadata. The **LeRobot dataset format** stores tabular data in Parquet and camera streams as compressed video, with per-episode indices and statistics; it can be streamed from the Hugging Face Hub without downloading everything (LeRobot v0.7.0 added a disk-less streaming reader). **RLDS** (a TensorFlow Datasets convention) is the format of Open X-Embodiment.

## Simulation benchmarks

| Benchmark | Focus | Note |
|---|---|---|
| **LIBERO** | Lifelong and multi-task manipulation; 130 language-specified tasks | Standard VLA fine-tuning test; top models now score in the high 90s, so it is near saturation |
| **ManiSkill3** | GPU-parallel manipulation and embodied AI tasks | Fast enough for RL at scale |
| **RLBench** | 100 hand-designed manipulation tasks | Classic benchmark for vision-based manipulation |
| **CALVIN** | Long-horizon, language-conditioned chains of tasks | Tests sequencing, not just single skills |

Simulation benchmarks are cheap and reproducible, but a policy can score highly by exploiting quirks of the simulator, and high scores saturate quickly.

## Evaluating on real robots

Real-world evaluation is slow and noisy. Good practice:

- **Many trials per task** (20+), with the success criterion written down before testing.
- **Fixed protocols**: the same object positions, lighting and resets for every policy compared.
- **Report failures and partial successes**, not only a highlight video.
- **Blind A/B comparisons**: shared real-robot evaluation services in 2025–26 let anyone submit a policy and receive pairwise comparisons on physical robots, reducing cherry-picking.

## Worked example: why 20 trials

A policy succeeds in 16 of 20 trials: 80%. The 95% confidence interval for a binomial proportion with $n = 20$ (Wilson interval) is roughly 58% to 92%. Another policy that scores 90% on 20 trials is *not* demonstrably better: the intervals overlap heavily. To separate 80% from 90% reliably you need on the order of 200 trials per policy. Most papers run far fewer, which is why small reported differences should be read with caution.

## Reading list

- Open X-Embodiment Collaboration 2023: the case for pooling robot data.
- Khazatsky et al. 2024 (DROID): what "in the wild" means for robot data.
- Liu et al. 2023 (LIBERO): the benchmark most VLA papers report.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Robot_Learning_Datasets_and_Benchmarks]])
```
