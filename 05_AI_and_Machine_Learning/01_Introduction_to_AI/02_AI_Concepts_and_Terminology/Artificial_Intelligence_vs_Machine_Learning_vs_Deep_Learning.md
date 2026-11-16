---
title: Artificial Intelligence vs Machine Learning vs Deep Learning
description: "How the terms nest: artificial intelligence is the goal, machine learning is one way to reach it (learning from data), deep learning is one kind of machine learning (many-layered neural networks), and foundation models are deep networks pretrained at scale. With one robot task solved four ways to show the difference, and a guide to which approach fits which robotics problem in 2026."
aliases:
  - AI vs ML vs DL
tags:
  - artificial-intelligence
  - machine-learning
  - deep-learning
  - foundation-models
type: concept
level: beginner
subfield:
  - robot-learning
  - foundations
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Artificial_Intelligence]]"
related:
  - "[[Machine_Learning]]"
  - "[[Deep_Learning]]"
  - "[[Vision-Language-Action_Models]]"
  - "[[Embodied_AI]]"
key_takeaways:
  - "AI ⊃ machine learning ⊃ deep learning ⊃ foundation models: each is a subset of the one before."
  - "Not all AI learns (search, planning and logic are AI too), and not all learning is deep (linear models, trees and Gaussian processes are machine learning)."
  - "Pick the simplest approach that meets the requirement: hand-written logic for safety rules, classical ML for small tabular problems, deep learning for perception, foundation models when open-ended generalisation is worth their cost."
sources:
  - "Russell, S. and Norvig, P. (2020). Artificial Intelligence: A Modern Approach, 4th edition. https://aima.cs.berkeley.edu/"
  - "Goodfellow, I., Bengio, Y. and Courville, A. (2016). Deep Learning, chapter 1 (figure 1.4: AI, ML, representation learning, DL). https://www.deeplearningbook.org/"
  - "LeCun, Y., Bengio, Y. and Hinton, G. (2015). Deep learning. https://www.nature.com/articles/nature14539"
  - "Bommasani, R. et al. (2021). On the Opportunities and Risks of Foundation Models. https://arxiv.org/abs/2108.07258"
---

# Artificial Intelligence vs Machine Learning vs Deep Learning

The three terms are often used interchangeably, but they are nested, each a subset of the one before:

```
Artificial intelligence  (machines that act intelligently, by any method)
└── Machine learning      (… by learning from data)
    └── Deep learning     (… using many-layered neural networks)
        └── Foundation models (… pretrained at very large scale, then adapted)
```

> **Etymology.** *Artificial intelligence* was named in 1955 (McCarthy et al.), *machine learning* in 1959 (Samuel), *deep learning* in the 2000s, and *foundation model* in 2021 by a Stanford report (Bommasani et al.).

## The differences in one table

| | Artificial intelligence | Machine learning | Deep learning | Foundation models |
|---|---|---|---|---|
| What it is | The goal: intelligent behaviour | Learning behaviour from data | ML with deep neural networks | Deep networks pretrained on huge, broad data |
| Includes | Search, planning, logic, probability, learning | Linear models, trees, kernels, neural networks | CNNs, RNNs, transformers, diffusion models | LLMs, VLMs, CLIP, DINOv2, VLAs |
| Who writes the knowledge | People write rules or models | People choose features; data sets the parameters | Data sets features *and* parameters | Mostly data, much of it from the web |
| Data needed | None to some | Hundreds to thousands of examples | Thousands to millions | Billions of tokens or images to pretrain; little to adapt |
| Robotics example | A* path planner, STRIPS task planner | Boosted trees predicting motor faults | CNN detector, RL locomotion policy | VLA following spoken instructions |

## One task, four ways: "sort recyclables on a conveyor"

1. **AI without learning.** Rules: "if the object is metallic (inductive sensor) → bin A; if transparent (light sensor) → bin B." Predictable and certifiable, but it fails on anything the rules did not foresee.
2. **Classical machine learning.** Hand-designed features (colour histogram, shape, weight) feed a random forest trained on a few thousand labelled items. Better coverage; needs feature engineering.
3. **Deep learning.** A CNN trained on tens of thousands of labelled images learns its own features. High accuracy on the trained categories; retraining is needed for new ones.
4. **Foundation model.** An open-vocabulary detector or VLM, prompted with "aluminium can, PET bottle, cardboard", recognises new items with few or no new labels. It is the most flexible, but heavier to run and harder to certify.

Real systems mix them: a foundation model labels data, a small deep network runs on the line, and hand-written rules handle safety and jams.

## Which approach when (robotics, 2026)

| Need | Usually best |
|---|---|
| Hard guarantees (safety limits, interlocks) | Hand-written logic and classical control |
| Small tabular data (diagnostics, battery health) | Classical ML (gradient-boosted trees) |
| Real-time perception of known categories | Compact deep networks |
| Open-ended instructions, novel objects | Foundation models (VLMs, VLAs), with classical safety layers |
| Locomotion and agile control | Deep RL trained in simulation |

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Artificial_Intelligence_vs_Machine_Learning_vs_Deep_Learning]])
```
