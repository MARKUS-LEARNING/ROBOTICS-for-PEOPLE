---
title: Feature Engineering and Data Preprocessing
description: "What happens to data before a model sees it: cleaning, synchronising, normalising, encoding angles and categories, augmenting, and splitting. Covers classical feature engineering versus learned features, a worked normalisation and angle-encoding example for robot observations, preprocessing of images and point clouds, and the dataset hygiene that robot-learning pipelines need in 2026."
aliases:
  - Data Preprocessing
  - Feature Engineering
  - Normalization
tags:
  - machine-learning
  - data-analysis
  - robot-learning
type: method
level: intermediate
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
  - "[[Bias,_Variance,_and_Overfitting]]"
  - "[[Robot_Learning_Datasets_and_Benchmarks]]"
  - "[[rosbag2]]"
  - "[[Teleoperation_and_Data_Collection]]"
key_takeaways:
  - "Models learn from what you give them: misaligned timestamps, mixed units or leaked test data quietly ruin results."
  - "Normalise inputs to similar scales, encode angles as (sin, cos) to remove the jump at ±π, and compute normalisation statistics on training data only."
  - "Deep learning replaced most hand-made features, but not preprocessing: synchronisation, normalisation, augmentation and careful splits are where robot-learning projects succeed or fail."
sources:
  - "Hastie, T., Tibshirani, R. and Friedman, J. (2009). The Elements of Statistical Learning. https://hastie.su.domains/ElemStatLearn/"
  - "scikit-learn developers (2026). Preprocessing data. https://scikit-learn.org/stable/modules/preprocessing.html"
  - "Ioffe, S. and Szegedy, C. (2015). Batch Normalization: Accelerating Deep Network Training by Reducing Internal Covariate Shift. https://arxiv.org/abs/1502.03167"
  - "Rudin, N., Hoeller, D., Reist, P. and Hutter, M. (2021). Learning to Walk in Minutes Using Massively Parallel Deep Reinforcement Learning. https://arxiv.org/abs/2109.11978"
  - "Tobin, J. et al. (2017). Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World. https://arxiv.org/abs/1703.06907"
---

# Feature Engineering and Data Preprocessing

Before any model learns anything, data has to be turned into numbers a model can use well. **Preprocessing** cleans, aligns and scales the data; **feature engineering** builds informative inputs from it. Practitioners often say that most of the work in a machine-learning project is here, and robotics is no exception.

> **Etymology.** *Feature* is from Latin *factura*, "a making, a formation": the distinctive parts of a face, and by extension of data. *Preprocess*: process *before* (*prae*) the main step.

## The pipeline

1. **Collect** with metadata: units, frames, robot model, calibration.
2. **Clean**: drop corrupted frames, sensor dropouts, failed recordings; handle missing values.
3. **Synchronise**: align cameras, joints and commands to one clock. A 30 ms misalignment between camera and joint data teaches a policy to react to the past.
4. **Transform**: normalise, encode angles and categories, resize and crop images, downsample point clouds.
5. **Augment** (training only): random crops, colour jitter and noise, so the model cannot rely on incidental details.
6. **Split** by episode, scene or object, never by individual frame, to avoid leakage.

## Worked example: normalising robot observations

A policy sees a joint angle $q$ (radians) and a joint velocity $\dot q$ (rad/s). In the training data, $q$ has mean 0.2 and standard deviation 0.8, and $\dot q$ has mean 0 and standard deviation 6.0. **Standardise** each: $z = (x - \mu)/\sigma$.

- A reading $q = 1.0$ becomes $(1.0 - 0.2)/0.8 = 1.0$.
- A reading $\dot q = 12$ becomes $(12 - 0)/6 = 2.0$.

Without this, velocities (tens of rad/s) would dominate angles (around 1 rad) in the first layer's weighted sums, and training would be slow and unstable. Compute $\mu$ and $\sigma$ on the **training set only** and store them with the model: the robot must apply exactly the same transform at run time. A mismatch between training and deployment normalisation is a classic silent bug. RL frameworks for locomotion apply running observation normalisation or fixed scales for this reason.

**Angles wrap around.** A heading of +3.1 rad and one of −3.1 rad are only 0.08 rad apart, but as numbers they differ by 6.2. Encode angles as $(\sin\theta, \cos\theta)$: both headings become $(\pm 0.042, -0.999)$, which are close, as they should be. Orientations in 3D are better as rotation matrices, the 6D representation (two columns of the rotation matrix), or quaternions with a sign convention, than as Euler angles.

## Classical features versus learned features

| Era | Inputs to the model | Example |
|---|---|---|
| Classical ML | Hand-designed features | Edge histograms (HOG), SIFT keypoints, FFT of motor current |
| Deep learning | Raw or lightly processed signals; the network learns features | Pixels into a CNN or ViT; joint histories into an RNN |
| Foundation models (2026) | Pretrained encoders produce features; a small head is trained | DINOv2 or SigLIP image features into a policy |

Hand-made features remain valuable where data is scarce or physics is well understood. Frequency-domain features of vibration still beat raw signals for many fault detectors, and with tabular data, trees on good features often beat neural networks (see [[Decision_Trees_and_Rule-Based_Systems]]).

## Preprocessing robot sensor data

| Data | Typical steps |
|---|---|
| Images | Undistort, resize or crop (often 224×224 or 256×256), normalise with the encoder's mean and std; augment with crops and colour jitter |
| Depth | Fill holes, clip to working range, convert to metres |
| Point clouds | Crop to workspace, downsample (voxel grid or farthest-point sampling), remove the table plane |
| Joint data | Unit conversion, normalisation, finite differences or filtering for velocity |
| Actions | Normalise per dimension (often to [−1, 1]); choose absolute or relative (delta) actions consistently |

## Dataset hygiene for robot learning

- Record **everything needed to reproduce**: robot description, camera calibration, control rate, software versions.
- Keep one **episode** per demonstration, with success labels.
- Split **by scene and object** to measure generalisation honestly.
- Version datasets as carefully as code; a silent change in normalisation or camera placement can explain a "mysterious" drop in policy success.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Feature_Engineering_and_Data_Preprocessing]])
```
