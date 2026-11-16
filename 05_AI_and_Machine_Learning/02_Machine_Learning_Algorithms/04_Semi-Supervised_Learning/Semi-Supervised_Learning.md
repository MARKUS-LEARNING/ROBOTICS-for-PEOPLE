---
title: Semi-Supervised Learning
description: "Semi-supervised learning trains on a few labelled examples plus many unlabelled ones. Covers self-training and pseudo-labels with a worked error budget, consistency regularisation (FixMatch), teacher–student distillation, and the 2026 robotics forms of the idea: foundation-model auto-labelling and latent-action pretraining on unlabelled video."
aliases:
  - Semi-supervised
  - Pseudo-labelling
  - Self-training
tags:
  - machine-learning
  - robot-learning
  - classification
type: concept
level: intermediate
subfield:
  - robot-learning
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Supervised_Learning]]"
  - "[[Unsupervised_Learning]]"
related:
  - "[[Imitation_Learning]]"
  - "[[World_Models]]"
  - "[[Robot_Learning_Datasets_and_Benchmarks]]"
key_takeaways:
  - "When labels are scarce but raw data is plentiful, use the unlabelled data too: label it with the model itself (pseudo-labels), or train the model to be consistent under perturbations."
  - "Pseudo-labels carry errors; keep only confident ones, measure their precision on a checked sample, and expect mistakes to be reinforced if you do not."
  - "In robotics the idea now appears as auto-labelling with foundation models and as learning 'latent actions' from unlabelled human video, then grounding them with a small amount of robot data."
sources:
  - "Chapelle, O., Schölkopf, B. and Zien, A. (2006). Semi-Supervised Learning. https://mitpress.mit.edu/9780262514125/semi-supervised-learning/"
  - "Sohn, K. et al. (2020). FixMatch: Simplifying Semi-Supervised Learning with Consistency and Confidence. https://arxiv.org/abs/2001.07685"
  - "Xie, Q. et al. (2019). Self-training with Noisy Student improves ImageNet classification. https://arxiv.org/abs/1911.04252"
  - "Ye, S. et al. (2024). Latent Action Pretraining from Videos. https://arxiv.org/abs/2410.11758"
  - "Bruce, J. et al. (2024). Genie: Generative Interactive Environments. https://arxiv.org/abs/2402.15391"
---

# Semi-Supervised Learning

**Semi-supervised learning** sits between [[Supervised_Learning|supervised]] and [[Unsupervised_Learning|unsupervised]] learning. You have a small labelled set and a large unlabelled one, and you use both. Robotics is full of this situation: hours of camera footage, a few hundred annotated frames; thousands of hours of human video, a few hundred robot demonstrations with actions.

> **Etymology.** Latin *semi-*, "half": half supervised. The field took shape in the 1990s and 2000s; Chapelle, Schölkopf and Zien's 2006 book collected its methods.

## Why unlabelled data helps

Unlabelled data shows the *shape* of the input space: where the dense regions are and where the gaps lie. The key assumption is that decision boundaries should run through gaps, not through dense clusters. Labelled points then only need to say which cluster is which.

## Main techniques

| Technique | Idea |
|---|---|
| **Self-training (pseudo-labels)** | Train on the labelled data, predict on the unlabelled data, add confident predictions as labels, retrain |
| **Consistency regularisation** | A model should give the same answer for an input and a perturbed version of it; FixMatch (2020) combines this with confidence-thresholded pseudo-labels |
| **Teacher–student (Noisy Student)** | A teacher labels unlabelled data; a larger, noised student learns from it and becomes the next teacher |
| **Pretrain then fine-tune** | Self-supervised pretraining on everything, then supervised fine-tuning on the labelled part (the dominant recipe in 2026) |
| **Graph methods** | Propagate labels along a similarity graph between examples |

## Worked example: the pseudo-label error budget

A robot team has 1,000 hand-labelled images and 50,000 unlabelled ones. The detector trained on the 1,000 is confident (score > 0.95) on 60% of the unlabelled images: 30,000 pseudo-labelled images. A person checks a random 200 of them and finds 97% correct.

So about $30{,}000 \times 0.03 = 900$ wrong labels are entering the training set, almost as many as the original clean labels. Whether that helps depends on whether the errors are *random* (usually tolerable; the extra data wins) or *systematic*, such as always missing transparent objects. Systematic errors get learned and reinforced. Good practice:
- check precision per class and per condition;
- raise the threshold for weak classes;
- re-check after each round of self-training.

## The 2026 robotics forms

**Auto-labelling with foundation models.** Open-vocabulary detectors and segmenters (Grounding DINO, SAM 2) act as the "teacher" for a new robot task: prompt with "cardboard box", label thousands of frames, have people spot-check, and train a small fast model for the robot. This is semi-supervised learning with a teacher trained on someone else's data.

**Latent actions from unlabelled video.** Human videos show tasks being done but contain no robot action labels. Latent-action methods (Genie, 2024; LAPA, 2024) learn a discrete "action" that explains the change between consecutive frames, pretrain a policy to predict those latent actions from huge amounts of video, then use a small set of real robot demonstrations to map latent actions to motor commands. Several 2025–26 robot foundation models use variants of this to exploit human video. See [[World_Models]] and [[Imitation_Learning]].

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Semi-Supervised_Learning]])
```
