---
title: Classification vs Regression vs Clustering
description: "Three basic kinds of prediction: classification picks a category, regression predicts a number, clustering finds groups without labels. Compares them on the same robot-grasping data, explains the loss and metrics each uses, and shows with a computed example how robot actions can be treated either way: discretised into tokens (classification, as in RT-2 and OpenVLA) or predicted continuously (regression and generative models)."
aliases:
  - Classification
  - Regression
  - Clustering
tags:
  - machine-learning
  - classification
  - regression
  - clustering
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
  - "[[Unsupervised_Learning]]"
  - "[[Optimization_and_Loss_Functions]]"
  - "[[Vision-Language-Action_Models]]"
key_takeaways:
  - "Classification answers 'which one?', regression answers 'how much?', clustering answers 'what groups exist?' (with no labels)."
  - "Each comes with its own loss and metrics: cross-entropy and precision/recall; squared error and RMSE; within-cluster spread and silhouette."
  - "Robot actions can be framed as classification over discretised bins (token-based VLAs) or as regression or generation of continuous values (diffusion and flow policies); the choice trades resolution against multimodality and compatibility with language models."
sources:
  - "Hastie, T., Tibshirani, R. and Friedman, J. (2009). The Elements of Statistical Learning. https://hastie.su.domains/ElemStatLearn/"
  - "Bishop, C. M. (2006). Pattern Recognition and Machine Learning. https://www.microsoft.com/en-us/research/publication/pattern-recognition-machine-learning/"
  - "Brohan, A. et al. (2023). RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control. https://arxiv.org/abs/2307.15818"
  - "Kim, M. J. et al. (2024). OpenVLA: An Open-Source Vision-Language-Action Model. https://arxiv.org/abs/2406.09246"
  - "Chi, C. et al. (2023). Diffusion Policy: Visuomotor Policy Learning via Action Diffusion. https://arxiv.org/abs/2303.04137"
---

# Classification vs Regression vs Clustering

Most machine-learning problems are one of three questions. Recognising which one you are asking decides the model's output layer, its loss, how you measure success and what data you need.

> **Etymology.** *Classification* is from Latin *classis*, "a division of the people": sorting into divisions. *Regression* is from *regredi*, "to go back": Francis Galton (1886) observed that the children of very tall parents "regress" toward average height, and the fitting method kept the name. *Cluster* is from Old English *clyster*, "a bunch".

## Side by side

| | Classification | Regression | Clustering |
|---|---|---|---|
| Question | Which category? | How much? | What groups exist? |
| Labels needed | Yes, categories | Yes, numbers | **No** |
| Output | Class probabilities | Real number(s) | Group assignments |
| Typical loss | Cross-entropy | Squared or absolute error | Within-cluster distance (k-means) |
| Metrics | Accuracy, precision, recall, F1 | RMSE, MAE, R² | Silhouette score, inspection |
| Learning type | [[Supervised_Learning\|Supervised]] | Supervised | [[Unsupervised_Learning\|Unsupervised]] |

## One dataset, three questions

A robot records 10,000 grasp attempts: a depth-image crop, the grasp pose, and the outcome.

- **Classification**: will this grasp succeed? (success / failure)
- **Regression**: how much lifting force will the grasp hold before slipping? (newtons)
- **Clustering**: what kinds of grasp does the robot use? Without labels, k-means on the grasp poses might reveal top-down pinches, side grasps and edge grasps, which a person then names.

## Robot actions: classification or regression?

Robot actions are continuous (joint angles, end-effector velocities), but they can be predicted either way.

**As classification (tokens).** RT-2 (2023) and OpenVLA (2024) divide each action dimension into **256 bins** and predict one bin per dimension, exactly as a language model predicts the next word. This lets them reuse a pretrained vision-language model unchanged.

*Resolution check.* For a joint with a full turn of range, one bin is $2\pi/256 = 0.0245$ rad ≈ 1.4°. For an end-effector displacement range of 0.5 m, one bin is about 2 mm. That is fine for picking up a cup and too coarse for inserting a peg with sub-millimetre clearance, unless the range is narrowed or more bins are used (1,024 bins over 0.5 m give about 0.5 mm).

**As regression or generation.** A regression head predicts continuous values directly, which gives precise values but, with squared error, averages different valid behaviours into an invalid one. [[Diffusion_Policy|Diffusion and flow-matching]] heads generate continuous action chunks *and* represent several valid behaviours. They became the most common choice in 2025–26 generalist policies such as π0 and GR00T.

**Classification also captures multiple modes.** A softmax over bins can put probability on two separate bins, which is one reason token-based policies handle multimodal demonstrations better than plain regression.

## Choosing in practice

- Is the target a category? Classification. Is it a quantity? Regression.
- No labels but a lot of data? Start with clustering or other unsupervised methods to understand it.
- Is the quantity continuous but multimodal (several right answers)? Use a classifier over bins, a mixture model, or a generative model, not plain regression.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Classification_vs_Regression_vs_Clustering]])
```
