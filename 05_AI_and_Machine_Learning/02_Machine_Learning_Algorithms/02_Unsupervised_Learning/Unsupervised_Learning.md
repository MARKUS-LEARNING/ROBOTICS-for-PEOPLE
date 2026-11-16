---
title: Unsupervised Learning
description: "Unsupervised learning finds structure in data without labels: clusters, low-dimensional representations, densities and anomalies. Covers k-means with a computed example on motor-vibration data, PCA and autoencoders, density estimation, the rise of self-supervised learning (the unsupervised idea that powers foundation models), and robotics uses in 2026."
aliases:
  - Unsupervised Machine Learning
  - Self-Supervised Learning
tags:
  - machine-learning
  - clustering
  - robot-learning
type: concept
level: beginner
subfield:
  - robot-learning
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Machine_Learning]]"
related:
  - "[[Supervised_Learning]]"
  - "[[Semi-Supervised_Learning]]"
  - "[[Autoencoders]]"
  - "[[Classification_vs_Regression_vs_Clustering]]"
  - "[[World_Models]]"
key_takeaways:
  - "Without labels, a model can still learn groups (clustering), compact descriptions (dimensionality reduction), what is normal (density), and therefore what is abnormal."
  - "Self-supervised learning, which predicts hidden parts of the data from the rest, turned the unsupervised idea into the main way large models are pretrained."
  - "Robots use it to compress sensor data, discover skills or modes in demonstrations, detect faults and novel situations, and pretrain visual and world models."
sources:
  - "Lloyd, S. (1982). Least squares quantization in PCM. https://ieeexplore.ieee.org/document/1056489"
  - "Hastie, T., Tibshirani, R. and Friedman, J. (2009). The Elements of Statistical Learning, chapter 14: Unsupervised Learning. https://hastie.su.domains/ElemStatLearn/"
  - "Chen, T. et al. (2020). A Simple Framework for Contrastive Learning of Visual Representations. https://arxiv.org/abs/2002.05709"
  - "He, K. et al. (2021). Masked Autoencoders Are Scalable Vision Learners. https://arxiv.org/abs/2111.06377"
  - "Oquab, M. et al. (2023). DINOv2: Learning Robust Visual Features without Supervision. https://arxiv.org/abs/2304.07193"
---

# Unsupervised Learning

**Unsupervised learning** looks for structure in data that has no labels. Nobody says what the right answer is; the algorithm finds groups, patterns and compact descriptions on its own. Since most of the data a robot collects is unlabelled, this matters a great deal.

> **Etymology.** *Un-* + *supervised*: learning with no teacher supplying answers (see [[Supervised_Learning]]). *Self-supervised*, popularised in the late 2010s, names the variant where the data supplies its own targets.

## What it can find

| Goal | Methods | Robot example |
|---|---|---|
| **Clusters** | k-means, Gaussian mixtures, DBSCAN | Group demonstrations into strategies; segment point clouds into objects |
| **Lower-dimensional structure** | PCA, [[Autoencoders]], t-SNE and UMAP (for visualisation) | Compress images for a policy; find the few "synergies" that explain most hand motion |
| **Density (what is normal)** | Gaussian mixtures, normalising flows, autoencoder reconstruction error | Detect faults and novel situations |
| **Representations** | Self-supervised learning (contrastive, masked prediction) | Pretrained visual encoders for perception and control |

## Worked example: k-means on motor vibration

A robot logs the RMS vibration (in g) of one joint motor in six sessions: 1.0, 1.2, 0.9, 3.1, 2.9, 3.3. Find $k = 2$ groups, starting from centres $c_1 = 1.0$ and $c_2 = 1.2$.

| Iteration | Assign each point to the nearest centre | New centres (means) |
|---|---|---|
| 1 | {1.0, 0.9} · {1.2, 3.1, 2.9, 3.3} | 0.95, 2.625 |
| 2 | {1.0, 1.2, 0.9} · {3.1, 2.9, 3.3} | 1.033, 3.100 |
| 3 | unchanged → converged | 1.033, 3.100 |

Two regimes appear: normal (≈ 1.0 g) and something different (≈ 3.1 g), perhaps a worn bearing or a different payload. The algorithm cannot say which; a person, or a later supervised model, interprets the clusters. This is typical: unsupervised learning proposes structure, and meaning comes from elsewhere.

k-means alternates two steps, assigning points to the nearest centre and moving each centre to its points' mean, until nothing changes. It needs $k$ chosen in advance, assumes roughly round clusters, and depends on the starting centres (run it several times, or use k-means++).

## Self-supervised learning: the unsupervised idea at scale

Self-supervised methods create a training signal from the data itself:
- **Masked prediction**: hide 75% of image patches and reconstruct them (MAE), or hide words and predict them (as in language models).
- **Contrastive learning**: pull together two augmented views of the same image, push apart different images (SimCLR, 2020).
- **Self-distillation**: a student network matches a teacher on different crops (DINOv2, 2023).
- **Predicting the future**: predict the next video frames or the next sensor readings, which is the principle behind [[World_Models|world models]].

Pretrained this way on huge unlabelled collections, models learn general features that need only a little labelled data afterwards. This is how almost every 2026 foundation model, including those inside robot policies, begins.

## Uses in robotics

- **Anomaly and novelty detection**: flag unusual motor currents, unexpected objects, or inputs unlike the training data (a cue to slow down or ask for help).
- **Demonstration analysis**: cluster or segment long demonstrations into skills and sub-tasks.
- **Perception**: segment point clouds into objects without class labels.
- **Pretraining**: visual encoders, video world models and latent action models trained on unlabelled human and robot video.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Unsupervised_Learning]])
```
