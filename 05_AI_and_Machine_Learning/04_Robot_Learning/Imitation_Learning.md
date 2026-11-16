---
title: Imitation Learning
description: "Teaching a robot by showing it: learning a policy from demonstrations instead of rewards. Covers behaviour cloning, compounding errors and DAgger, action chunking (ACT), generative policies, and why imitation became the main route to real-world manipulation."
aliases:
  - Imitation Learning
  - Behavior Cloning
  - Behaviour Cloning
  - Learning from Demonstration
  - Robot Programming by Demonstration
tags:
  - imitation-learning
  - robot-learning
  - manipulation
  - teleoperation
  - supervised-learning
type: method
level: intermediate
subfield:
  - robot-learning
  - manipulation
status: keystone
author: Jordan_Smith
date: 2026-11-15
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Policy]]"
  - "[[Supervised_Learning]]"
related:
  - "[[Teleoperation_and_Data_Collection]]"
  - "[[Diffusion_Policy]]"
  - "[[Vision-Language-Action_Models]]"
  - "[[Reinforcement_Learning]]"
  - "[[Robot_Learning_Datasets_and_Benchmarks]]"
key_takeaways:
  - "Behaviour cloning is supervised learning from (observation, expert action) pairs; it is simple and works surprisingly well with good data."
  - "Its classic failure is compounding error: small mistakes lead to states the expert never visited. DAgger, action chunking and noise injection all fight it."
  - "Since 2023, imitation with action chunking and generative (diffusion / flow) policies is the main route to real-world manipulation, and the pretraining recipe behind VLAs."
sources:
  - "Ross, S., Gordon, G. and Bagnell, J. A. (2011). A Reduction of Imitation Learning and Structured Prediction to No-Regret Online Learning. https://arxiv.org/abs/1011.0686"
  - "Zhao, T. et al. (2023). Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware. https://arxiv.org/abs/2304.13705"
  - "Chi, C. et al. (2023). Diffusion Policy: Visuomotor Policy Learning via Action Diffusion. https://arxiv.org/abs/2303.04137"
  - "Fu, Z., Zhao, T. and Finn, C. (2024). Mobile ALOHA: Learning Bimanual Mobile Manipulation with Low-Cost Whole-Body Teleoperation. https://arxiv.org/abs/2401.02117"
  - "Mandlekar, A. et al. (2023). MimicGen: A Data Generation System for Scalable Robot Learning using Human Demonstrations. https://arxiv.org/abs/2310.17596"
  - "Levine, S. et al. (2015). End-to-End Training of Deep Visuomotor Policies. https://arxiv.org/abs/1504.00702"
  - "Hugging Face (2026). LeRobot v0.7.0 release notes. https://github.com/huggingface/lerobot/releases/tag/v0.7.0"
---

# Imitation Learning

**Imitation learning** teaches a robot by showing it what to do. A person demonstrates a task, usually by teleoperating the robot, and a neural network learns to reproduce the demonstrated actions from what the robot sees. There is no reward to design: the demonstrations *are* the specification.

It has become the main way robots learn real-world manipulation, such as folding laundry, loading a dishwasher or assembling boxes, because for those tasks it is far easier to show the behaviour than to write a reward for it (see [[Reinforcement_Learning]]).

> **Etymology.** *Imitation* is from Latin *imitari*, "to copy", related to *imago*, "image". *Behaviour cloning*, the simplest form, borrows *clone* from Greek *klōn*, "twig": a plant grown from a cutting is a genetic copy of its parent. The term appeared in early-1990s work on learning to fly simulators from pilot data; Dean Pomerleau's ALVINN (1989) had already driven a van by imitating a human driver.

## Behaviour cloning

Collect demonstrations $\mathcal{D} = \{(o_i, a_i)\}$, pairs of observation and the expert's action. Train a [[Policy]] $\pi_\theta$ by ordinary [[Supervised_Learning]]:

$$\min_\theta \; \mathbb{E}_{(o,a) \sim \mathcal{D}} \big[ -\log \pi_\theta(a \mid o) \big]$$

With a Gaussian output this is mean-squared error between predicted and demonstrated actions. That is the whole algorithm. Most of the difficulty is in the data and in two failure modes.

## Failure mode 1: compounding errors

A cloned policy makes small mistakes. A slightly wrong action puts the robot in a slightly unfamiliar state, which it handles slightly worse, and the errors snowball. The policy was trained only on states the *expert* visited, but at test time it visits *its own* states.

Ross, Gordon and Bagnell (2011) made this precise: with per-step error $\epsilon$ over horizon $T$, behaviour cloning's total cost can grow as $O(\epsilon T^2)$, while their **DAgger** algorithm brings it to $O(\epsilon T)$. DAgger runs the learned policy, asks the expert what it *should* have done in the states the policy actually reached, adds those labels to the dataset, and retrains.

Practical defences used today:

| Defence | How it helps |
|---|---|
| **DAgger-style corrections** | A human takes over when the robot drifts; those corrections become training data |
| **Action chunking** | Predict the next $k$ actions at once and execute them; fewer decisions means fewer chances to drift (ACT, Zhao et al. 2023) |
| **Noise injection during demonstrations** | Perturb the robot while recording so the data includes recoveries |
| **More, more diverse data** | Cover more states, so fewer are unfamiliar |

## Failure mode 2: multimodal demonstrations

Humans do the same task different ways. Half the demonstrations go around a cup on the left, half on the right. A policy trained with mean-squared error predicts the *average*, straight through the cup. The fix is a policy that represents a full distribution over actions:

- **Discretised actions** with a classifier (used by early VLAs such as RT-2 and OpenVLA).
- **Diffusion and flow-matching policies**, which generate an action by refining noise, and can represent any number of modes. See [[Diffusion_Policy]].
- **Latent-variable models** such as the conditional VAE inside ACT.

## Worked example: how much data?

ACT (Zhao et al. 2023) learned fine bimanual tasks such as opening a condiment cup and slotting a battery from about **50 demonstrations per task**, recorded with the low-cost ALOHA teleoperation rig. Each demonstration lasted roughly 10–20 seconds at 50 Hz, so a task's dataset was on the order of 50 × 15 × 50 ≈ 37,500 (observation, action) pairs: tiny by deep-learning standards. Two design choices made it work. Action chunks of 100 steps cut the number of decisions per episode by a factor of 100, and *temporal ensembling* averaged overlapping chunks for smooth motion.

By contrast, generalist [[Vision-Language-Action_Models]] are pretrained on tens of thousands of hours across many robots, then fine-tuned with a few hours on a new task.

## From one task to many

| Stage | Data | Example |
|---|---|---|
| Single task | 50–200 demonstrations | ACT, Diffusion Policy |
| Multi-task, one robot | Thousands of demonstrations with language labels | Mobile ALOHA co-training (Fu et al. 2024) |
| Many robots, many tasks | Pooled datasets across labs and embodiments | Open X-Embodiment, DROID; see [[Robot_Learning_Datasets_and_Benchmarks]] |
| Generalist foundation model | Robot data plus web-scale vision-language data | π0.x, GR00T, Gemini Robotics |

**Synthetic demonstrations** stretch scarce human data: MimicGen (Mandlekar et al. 2023) takes a few human demonstrations and automatically generates thousands of new ones in simulation by re-targeting object-centric segments to new object poses.

## Where the data comes from

Demonstrations are only as good as the interface that records them: leader-follower arms (ALOHA), VR headsets, handheld grippers (UMI) and exoskeletons each trade cost, speed and fidelity. See [[Teleoperation_and_Data_Collection]].

## State of the art (as of November 2026)

- **Imitation is the pretraining recipe for generalist robots.** Every major [[Vision-Language-Action_Models|VLA]] is trained mostly by imitation over large multi-robot datasets, increasingly followed by reinforcement-learning fine-tuning.
- **Generative action heads are the default.** Flow-matching and diffusion heads (π0.x, GR00T N1.x) have largely replaced single-action regression for manipulation.
- **Open tooling has matured.** Hugging Face's LeRobot (v0.7.0, November 2026) packages dataset formats, ACT, diffusion and VLA policies, remote inference and low-cost hardware drivers.
- **Human video as demonstration.** Learning from egocentric video of people working, with no robot in the loop, is now part of pretraining for the largest models.

## Tooling

| Tool | What it gives you |
|---|---|
| LeRobot | Datasets, training scripts for ACT, Diffusion Policy and VLAs, real-robot drivers |
| robomimic | Benchmarks and baselines for learning from demonstration |
| ALOHA / Mobile ALOHA | Open-source low-cost bimanual teleoperation hardware |
| MimicGen | Synthetic demonstration generation in simulation |

## Reading list

- Ross, Gordon and Bagnell 2011, *A Reduction of Imitation Learning and Structured Prediction to No-Regret Online Learning* (DAgger).
- Zhao et al. 2023, *Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware* (ALOHA and ACT).
- Chi et al. 2023, *Diffusion Policy*.
- Fu, Zhao and Finn 2024, *Mobile ALOHA*.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Imitation_Learning]])
```
