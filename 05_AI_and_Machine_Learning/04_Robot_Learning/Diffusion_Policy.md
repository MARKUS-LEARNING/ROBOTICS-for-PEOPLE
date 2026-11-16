---
title: Diffusion Policy
description: "A robot policy that generates actions the way image diffusion models generate pictures: start from noise and refine it into a chunk of future actions, conditioned on what the robot sees. Handles demonstrations that solve the same task in different ways; the ancestor of the flow-matching action heads in today's VLAs."
aliases:
  - Diffusion Policies
  - Flow Matching Policy
tags:
  - diffusion-models
  - imitation-learning
  - generative-models
  - robot-learning
  - manipulation
type: method
level: advanced
subfield:
  - robot-learning
  - manipulation
status: keystone
author: Jordan_Smith
date: 2026-11-15
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Imitation_Learning]]"
  - "[[Generative_Models]]"
related:
  - "[[Vision-Language-Action_Models]]"
  - "[[Policy]]"
  - "[[Teleoperation_and_Data_Collection]]"
  - "[[World_Models]]"
key_takeaways:
  - "Diffusion Policy treats a chunk of future actions as the thing to generate, and denoises it conditioned on camera images and robot state."
  - "It captures multimodal demonstrations, trains stably, and works well with action chunking and receding-horizon execution."
  - "Flow matching does the same job in fewer steps and is the action head of π0-family and GR00T models."
sources:
  - "Chi, C. et al. (2023). Diffusion Policy: Visuomotor Policy Learning via Action Diffusion. https://arxiv.org/abs/2303.04137"
  - "Ho, J., Jain, A. and Abbeel, P. (2020). Denoising Diffusion Probabilistic Models. https://arxiv.org/abs/2006.11239"
  - "Lipman, Y. et al. (2022). Flow Matching for Generative Modeling. https://arxiv.org/abs/2210.02747"
  - "Black, K. et al. (2024). π0: A Vision-Language-Action Flow Model for General Robot Control. https://arxiv.org/abs/2410.24164"
  - "NVIDIA (2025). GR00T N1: An Open Foundation Model for Generalist Humanoid Robots. https://arxiv.org/abs/2503.14734"
  - "Zhao, T. et al. (2023). Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware. https://arxiv.org/abs/2304.13705"
---

# Diffusion Policy

**Diffusion Policy** is an [[Imitation_Learning]] method that generates robot actions with a diffusion model. Instead of predicting one action directly, the policy starts from random noise shaped like a sequence of future actions and refines it, step by step, into a smooth, plausible motion, guided by what the robot's cameras currently see. Introduced by Cheng Chi and colleagues (Columbia, Toyota Research Institute, MIT) in 2023, it changed what "default" means for learning manipulation from demonstrations.

> **Etymology.** *Diffusion* is from Latin *diffundere*, "to pour out, spread". In physics it names the spreading of particles from high to low concentration, described by Fick (1855) and explained by Einstein's 1905 work on Brownian motion. Diffusion *models* run that spreading in reverse: noise is gradually "un-diffused" back into structure.

## The problem it solves

Human demonstrations are **multimodal**: the same task is done in different, equally good ways. Regression collapses them into an average that may be worse than any of them (see the worked example in [[Generative_Models]]). Earlier fixes, such as mixtures of Gaussians or discretised actions, struggled with high-dimensional, continuous action sequences. Diffusion handles both naturally.

## How it works

1. **What is generated.** A chunk of future actions $A_t = (a_t, \dots, a_{t+H-1})$, for example $H = 16$ steps of end-effector poses and gripper commands.
2. **Conditioning.** The observation $O_t$: the last few camera images (through a vision encoder) and robot proprioception.
3. **Training.** Add noise to demonstrated action chunks and train a network to predict that noise:
$$\mathcal{L} = \mathbb{E}\,\big\lVert \epsilon - \epsilon_\theta(A_t^k, k, O_t) \big\rVert^2, \qquad A_t^k = \sqrt{\bar\alpha_k}\,A_t + \sqrt{1-\bar\alpha_k}\,\epsilon$$
4. **Sampling.** Start from Gaussian noise $A^K$ and denoise for $K$ steps to obtain an action chunk.
5. **Receding-horizon execution.** Execute the first few actions (for example 8 of 16), observe again, and generate a new chunk. This combines the smoothness of chunking with the ability to react.

The original paper tested two denoising networks, a 1D temporal convolutional U-Net and a transformer, and reported an average improvement of 46.9% over the prior state of the art across 12 tasks from four manipulation benchmarks, plus real-robot tasks such as pouring and spreading sauce.

## Flow matching: the faster sibling

Diffusion needs tens to a hundred denoising steps, which is slow at 10–50 Hz. **Flow matching** learns a velocity field that moves noise to data along nearly straight paths, so about 10 integration steps suffice. π0 (Black et al. 2024) attached a flow-matching "action expert" to a vision-language model, generating 50-step action chunks; NVIDIA's GR00T N1 uses a diffusion-transformer action head trained with flow matching. In 2026 these heads are standard in [[Vision-Language-Action_Models]].

## Worked example: timing on a real robot

Suppose the policy runs on a GPU and one denoising step takes 4 ms.

| Setup | Steps | Inference time | Actions executed per inference at 10 Hz |
|---|---|---|---|
| Diffusion (DDPM) | 100 | 400 ms | must cover at least 4 steps |
| Diffusion (DDIM, fewer steps) | 16 | 64 ms | 1 step is enough |
| Flow matching | 10 | 40 ms | 1 step is enough |

With chunking, even the slow setup works: the robot executes the previous chunk while the next one is being generated. That overlap ("asynchronous" or "real-time chunking") is how large policies keep motion smooth despite heavy inference.

## Strengths and limits

| Strengths | Limits |
|---|---|
| Captures multimodal behaviour | Slower inference than regression |
| Stable training, few hyperparameters | Still imitation: cannot beat its demonstrations without RL |
| Smooth, temporally consistent actions | Needs good, consistent demonstrations |
| Works from tens to hundreds of demonstrations per task | Generalisation beyond the data is limited for single-task models |

## State of the art (as of November 2026)

- **Inside foundation models.** Diffusion and flow-matching action heads are the norm in generalist policies (π0.x, GR00T N1.x).
- **Joint video and action generation.** World action models extend the idea, generating future video and actions together from a video diffusion backbone; see [[World_Models]].
- **Fine-tuning with reward.** Methods that adjust diffusion policies with reinforcement learning, so they improve beyond their demonstrations, are an active research area.

## Tooling

LeRobot ships a Diffusion Policy implementation with training scripts and pretrained checkpoints; the original authors' code is open source.

## Reading list

- Chi et al. 2023, *Diffusion Policy: Visuomotor Policy Learning via Action Diffusion*.
- Lipman et al. 2022, *Flow Matching for Generative Modeling*.
- Black et al. 2024, *π0*: flow matching inside a vision-language-action model.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Diffusion_Policy]])
```
