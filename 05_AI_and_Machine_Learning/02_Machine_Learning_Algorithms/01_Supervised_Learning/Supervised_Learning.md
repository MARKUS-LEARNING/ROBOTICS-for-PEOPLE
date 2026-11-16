---
title: Supervised Learning
description: "Supervised learning fits a model to examples of inputs paired with correct outputs, for classification (which category?) and regression (how much?). Covers the setup, common models, how to measure performance with a worked grasp-success confusion matrix, where labels come from in robotics (human annotation, self-supervision from robot outcomes, simulation, foundation-model auto-labelling), and imitation learning as supervised learning of actions."
aliases:
  - Supervised Machine Learning
tags:
  - machine-learning
  - classification
  - regression
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
  - "[[Classification_vs_Regression_vs_Clustering]]"
  - "[[Bias,_Variance,_and_Overfitting]]"
  - "[[Imitation_Learning]]"
  - "[[Unsupervised_Learning]]"
  - "[[Semi-Supervised_Learning]]"
key_takeaways:
  - "Supervised learning needs labelled pairs (x, y); classification predicts categories, regression predicts numbers."
  - "Pick the metric that matches the cost of mistakes: for a robot, a false 'grasp will succeed' (dropped object) may cost more than a missed grasp, so precision can matter more than accuracy."
  - "Labels are the bottleneck; robotics gets them from humans, from the robot's own outcomes (self-supervision), from simulation, and increasingly from foundation models that auto-label data."
sources:
  - "Hastie, T., Tibshirani, R. and Friedman, J. (2009). The Elements of Statistical Learning, 2nd edition. https://hastie.su.domains/ElemStatLearn/"
  - "Bishop, C. M. (2006). Pattern Recognition and Machine Learning. https://www.microsoft.com/en-us/research/publication/pattern-recognition-machine-learning/"
  - "Pinto, L. and Gupta, A. (2015). Supersizing Self-supervision: Learning to Grasp from 50K Tries and 700 Robot Hours. https://arxiv.org/abs/1509.06825"
  - "Liu, S. et al. (2023). Grounding DINO: Marrying DINO with Grounded Pre-Training for Open-Set Object Detection. https://arxiv.org/abs/2303.05499"
  - "Mahler, J. et al. (2017). Dex-Net 2.0: Deep Learning to Plan Robust Grasps with Synthetic Point Clouds and Analytic Grasp Metrics. https://arxiv.org/abs/1703.09312"
---

# Supervised Learning

**Supervised learning** is learning from examples with answers. You give the algorithm many inputs $x$, each paired with the correct output $y$ (its **label**), and it learns a function $f(x) \approx y$ that also works on new inputs. It is the most widely used form of [[Machine_Learning|machine learning]].

> **Etymology.** Latin *supervidere*, "to oversee": a teacher oversees learning by supplying the right answers. The term contrasts with [[Unsupervised_Learning|unsupervised]] learning, which has no answers to learn from.

## Two kinds of question

| Task | Output | Robot example | Typical loss |
|---|---|---|---|
| **Classification** | A category | Is this grasp going to succeed? Which object is this? | Cross-entropy |
| **Regression** | A number or vector | Distance to the wall; the 6-DoF pose of a part; the next joint command | Squared error |

See [[Classification_vs_Regression_vs_Clustering]].

## The workflow

1. **Collect and label** data, covering the conditions the robot will meet (lighting, objects, backgrounds).
2. **Split** into training, validation and test sets, and keep them independent. Frames from the same video must not be split across sets, or the test score will be optimistic.
3. **Choose a model**: linear or logistic regression, decision trees and gradient-boosted trees for tabular data, neural networks for images and other raw signals.
4. **Train** by minimising the loss on the training set; tune settings on the validation set.
5. **Evaluate once** on the test set, then on the robot.

## Worked example: is the grasp classifier good enough?

A network predicts whether a planned grasp will succeed. On 200 held-out real grasps:

| | Actually succeeded | Actually failed |
|---|---|---|
| **Predicted success** | 90 (true positive) | 10 (false positive) |
| **Predicted failure** | 20 (false negative) | 80 (true negative) |

- **Accuracy** = (90 + 80)/200 = 0.85.
- **Precision** = 90/(90 + 10) = 0.90: when it says "go", it is right 90% of the time.
- **Recall** = 90/(90 + 20) = 0.82: it finds 82% of the grasps that would have worked.
- **F1** = 2(0.90)(0.82)/(0.90 + 0.82) = 0.86.

Which number matters depends on the robot. A bin-picking robot that drops parts on false positives wants high **precision**: it can simply try another grasp candidate. Raising the decision threshold trades recall for precision; the precision–recall curve shows the whole trade-off.

## Where robot labels come from

| Source | How | Example |
|---|---|---|
| **Human annotation** | People draw boxes and masks, or rate outcomes | Detection datasets |
| **Self-supervision from outcomes** | The robot tries, and the result is the label | 50,000 grasp attempts in 700 robot hours (Pinto and Gupta, 2015) |
| **Simulation** | Physics or rendering supplies perfect labels | Dex-Net's 6.7 million synthetic depth images |
| **Foundation-model auto-labelling** | Open-vocabulary detectors and segmenters (Grounding DINO, SAM 2) label new data from a text prompt; people check a sample | Common in 2025–26 data pipelines |
| **Demonstrations** | The human's action is the label for the observation | [[Imitation_Learning]]: behaviour cloning is supervised learning of actions |

## Limits

- **Distribution shift**: a model is reliable only on data like its training data.
- **Label noise**: inconsistent labels cap performance.
- **Compounding errors**: when the model's outputs drive the robot, small errors lead to unfamiliar states. This is why pure supervised imitation needs corrections (DAgger) or large, diverse data.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Supervised_Learning]])
```
