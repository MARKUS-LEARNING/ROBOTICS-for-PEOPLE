---
title: Machine Learning
description: "Machine learning is how a program improves at a task from data instead of from hand-written rules. Covers the three classic settings (supervised, unsupervised, reinforcement) plus self-supervised pretraining, the train–validate–test discipline, a fully computed sensor-calibration example, why deep learning and scale took over, and what machine learning does and does not yet do well on robots in 2026."
aliases:
  - ML
tags:
  - machine-learning
  - artificial-intelligence
  - robot-learning
  - statistics
type: concept
level: beginner
subfield:
  - robot-learning
status: keystone
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Artificial_Intelligence]]"
  - "[[Probability_and_Statistics_for_Robotics]]"
related:
  - "[[Supervised_Learning]]"
  - "[[Unsupervised_Learning]]"
  - "[[Reinforcement_Learning]]"
  - "[[Deep_Learning]]"
  - "[[Bias,_Variance,_and_Overfitting]]"
  - "[[Imitation_Learning]]"
key_takeaways:
  - "A machine-learning system is a model with adjustable parameters, a loss that scores its mistakes, data, and an optimiser that changes the parameters to reduce the loss."
  - "Judge a model only on data it never trained on; a separate test set is the difference between learning and memorising."
  - "In robotics, learning now dominates perception and increasingly produces control policies, while classical methods keep the guarantees: state estimation, low-level control and safety."
sources:
  - "Samuel, A. L. (1959). Some Studies in Machine Learning Using the Game of Checkers. https://ieeexplore.ieee.org/document/5392560"
  - "Mitchell, T. M. (1997). Machine Learning. http://www.cs.cmu.edu/~tom/mlbook.html"
  - "Bishop, C. M. (2006). Pattern Recognition and Machine Learning. https://www.microsoft.com/en-us/research/publication/pattern-recognition-machine-learning/"
  - "LeCun, Y., Bengio, Y. and Hinton, G. (2015). Deep learning. https://www.nature.com/articles/nature14539"
  - "Kaplan, J. et al. (2020). Scaling Laws for Neural Language Models. https://arxiv.org/abs/2001.08361"
  - "Sutton, R. S. (2019). The Bitter Lesson. http://www.incompleteideas.net/IncIdeas/BitterLesson.html"
---

# Machine Learning

**Machine learning (ML)** is the study of programs that get better at a task by learning from data rather than by following rules a person wrote down. A spam filter learns which emails are junk from millions of examples; a robot learns to recognise a mug from labelled images, or to grasp it from its own attempts.

> **Etymology.** Arthur Samuel of IBM popularised the phrase in 1959, describing a checkers program that improved by playing itself. Tom Mitchell's 1997 definition is still the standard: a program *learns* from experience E with respect to task T and performance measure P if its performance at T, as measured by P, improves with E.

## The four ingredients

Every ML method, from a straight-line fit to a robot foundation model, has the same parts:

| Ingredient | What it is | Straight-line example | Robot-policy example |
|---|---|---|---|
| **Model** | A function with adjustable parameters $\theta$ | $\hat y = a x + b$ | A neural network from camera images to joint targets |
| **Data** | Examples of the task | Sensor readings with true distances | Teleoperated demonstrations |
| **Loss** | A number that says how wrong the model is | Mean squared error | Error between predicted and demonstrated actions |
| **Optimiser** | A procedure that changes $\theta$ to reduce the loss | Closed-form least squares | Stochastic gradient descent (Adam) |

Training means solving $\theta^* = \arg\min_\theta \frac{1}{N}\sum_i \ell\big(f_\theta(x_i), y_i\big)$, and the hope is that $\theta^*$ also works on inputs it has never seen. That hope, **generalisation**, is the whole game.

## The learning settings

| Setting | Data | Learns | Robotics use |
|---|---|---|---|
| **[[Supervised_Learning\|Supervised]]** | Inputs with correct outputs (labels) | A mapping input → output | Object detection, grasp-quality prediction, [[Imitation_Learning\|imitation]] of demonstrations |
| **[[Unsupervised_Learning\|Unsupervised]]** | Inputs only | Structure: clusters, low-dimensional representations | Anomaly detection, compressing sensor data |
| **Self-supervised** | Inputs only, with labels made from the data itself (predict the hidden part) | General-purpose representations | Pretraining vision and [[World_Models\|world models]] on video |
| **[[Reinforcement_Learning\|Reinforcement]]** | Rewards from trial and error | A policy that maximises reward | Locomotion, dexterous manipulation, fine-tuning |

Self-supervised pretraining followed by supervised or reinforcement fine-tuning is the recipe behind the 2020s' foundation models, including the [[Vision-Language-Action_Models|vision-language-action models]] that control robots.

## Worked example: calibrating a range sensor

A cheap distance sensor reads slightly high. Four measurements against a tape measure:

| True distance $y$ (m) | 0.50 | 1.00 | 1.50 | 2.00 |
|---|---|---|---|---|
| Sensor reading $x$ (m) | 0.52 | 1.07 | 1.55 | 2.10 |

Fit $\hat y = a x + b$ by least squares. The means are $\bar x = 1.31$ and $\bar y = 1.25$, and

$$a = \frac{\sum (x_i-\bar x)(y_i-\bar y)}{\sum (x_i-\bar x)^2} = 0.9572, \qquad b = \bar y - a\bar x = -0.0039.$$

The corrected readings are 0.494, 1.020, 1.480 and 2.006 m: a root-mean-square error of 0.015 m, against 0.067 m before calibration. A new reading of 1.80 m is corrected to $0.9572 \times 1.80 - 0.0039 = 1.719$ m.

That is machine learning in miniature: model, data, loss and optimiser. It also shows the catch: with only four points, you cannot tell whether the sensor is really linear. Collect more points across the range and **hold some back** to test the fit.

## The discipline: train, validate, test

- **Training set**: the model learns from it.
- **Validation set**: you choose between models and settings with it.
- **Test set**: touched once, at the end, to report performance honestly.

A model that scores well on training data but badly on test data has **overfit**: it memorised noise. One that does badly on both has **underfit**. See [[Bias,_Variance,_and_Overfitting]]. In robotics there is a fourth, harsher test: **deployment**. A policy that works in the lab can fail with new lighting, objects or floors (distribution shift). For this reason robot-learning results are reported on held-out objects, scenes and robots.

## Why deep learning and scale took over

Classical ML needed people to design *features* (edges, corners, histograms) by hand. **[[Deep_Learning|Deep learning]]** learns the features too, from raw pixels or sensor values, and it keeps improving as data and compute grow:
- AlexNet (2012) won the ImageNet challenge by a wide margin;
- transformers (2017) unified vision, language and action;
- scaling-law studies (Kaplan et al., 2020) showed that the error of language models falls predictably as models, data and compute grow.

Rich Sutton's essay *The Bitter Lesson* (2019) summarises 70 years of AI: general methods that scale with computation beat methods built on human knowledge.

Robotics is following the same curve, with one difference: robot data is scarce. That is why 2026 robot learning leans on:
- pretraining from web images and video;
- simulation;
- pooled multi-robot datasets (see [[Robot_Learning_Datasets_and_Benchmarks]]).

## What ML does on robots in 2026

| Area | State in 2026 |
|---|---|
| **Perception** | Learned almost everywhere: detection, segmentation, depth, pose estimation, open-vocabulary recognition |
| **Locomotion** | RL policies trained in simulation are the standard for quadrupeds and humanoids |
| **Manipulation** | Imitation learning and VLAs handle many household and factory tasks; reliability and dexterity are still improving |
| **Planning and reasoning** | Language and vision-language models decompose tasks and call skills |
| **Low-level control, estimation, safety** | Still mostly classical: PID, MPC, Kalman filters, verified safety layers |

## Where to go next

[[Supervised_Learning]] → [[Neural_Networks]] → [[Deep_Learning]] → [[Imitation_Learning]] → [[Vision-Language-Action_Models]]

[[Markov_Decision_Process]] → [[Reinforcement_Learning]] → [[Reinforcement_Learning_for_Robots]]

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Machine_Learning]])
```
