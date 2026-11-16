---
title: Decision Trees and Rule-Based Systems
description: "Decision trees learn if-then rules from data; rule-based systems are written by experts; behaviour trees organise a robot's actions. Covers how a tree chooses splits with a computed information-gain example on terrain data, random forests and gradient boosting (still the best tools for many tabular problems), expert systems, and behaviour trees as the standard way to structure robot tasks in 2026 (Nav2, BehaviorTree.CPP)."
aliases:
  - Decision Tree
  - Random Forest
  - Gradient Boosting
  - Rule-Based Systems
  - Behavior Trees
tags:
  - machine-learning
  - classification
  - behavior-tree
  - decision-making
type: concept
level: intermediate
subfield:
  - robot-learning
  - planning
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Supervised_Learning]]"
related:
  - "[[Classification_vs_Regression_vs_Clustering]]"
  - "[[Knowledge_Representation_and_Reasoning]]"
  - "[[Nav2_Navigation]]"
  - "[[Agents_and_Environments]]"
key_takeaways:
  - "A decision tree asks a sequence of yes/no questions about features; learning picks, at each node, the question that best separates the classes (largest information gain)."
  - "Ensembles of trees (random forests, gradient boosting such as XGBoost) remain state of the art on tabular data, and are fast and interpretable enough for onboard diagnostics."
  - "Behaviour trees, related in name and spirit, are hand-designed trees of conditions and actions; they are how Nav2 and many robot task frameworks structure decisions, and increasingly what language-model planners generate."
sources:
  - "Quinlan, J. R. (1986). Induction of decision trees. https://link.springer.com/article/10.1007/BF00116251"
  - "Breiman, L. (2001). Random Forests. https://link.springer.com/article/10.1023/A:1010933404324"
  - "Chen, T. and Guestrin, C. (2016). XGBoost: A Scalable Tree Boosting System. https://arxiv.org/abs/1603.02754"
  - "Grinsztajn, L., Oyallon, E. and Varoquaux, G. (2022). Why do tree-based models still outperform deep learning on tabular data? https://arxiv.org/abs/2207.08815"
  - "Colledanchise, M. and Ögren, P. (2017). Behavior Trees in Robotics and AI: An Introduction. https://arxiv.org/abs/1709.00084"
  - "BehaviorTree.CPP contributors (2026). BehaviorTree.CPP. https://github.com/BehaviorTree/BehaviorTree.CPP"
---

# Decision Trees and Rule-Based Systems

Some of the most practical tools in AI are trees of if-then questions. They come in three forms that robotics uses: **decision trees** learned from data, **rule-based systems** written by experts, and **behaviour trees** that organise what a robot does next.

> **Etymology.** The *tree* picture is a root question branching into answers, ending in leaves. Ross Quinlan's ID3 (1986) and Breiman et al.'s CART (1984) made learned trees practical. *Behaviour trees* came from video-game AI in the mid-2000s and entered robotics around 2012.

## Learned decision trees

A tree asks a question at each node ("slope > 15°?") and follows the branch, until a leaf gives the prediction. Learning grows the tree top-down, choosing at each node the question that best separates the outcomes.

### Worked example: which question first?

A legged robot logged 10 terrain patches: 6 traversable, 4 not. Two candidate questions:

The **entropy** of the labels before splitting is $H = -0.6\log_2 0.6 - 0.4\log_2 0.4 = 0.971$ bits.

| Split | Branch contents | Branch entropy | Weighted entropy | **Information gain** |
|---|---|---|---|---|
| Slope > 15°? | yes: 1 traversable, 4 not · no: 5 traversable, 0 not | 0.722 · 0 | 0.5(0.722) + 0.5(0) = 0.361 | 0.971 − 0.361 = **0.610** |
| Surface wet? | yes: 2, 2 · no: 4, 2 | 1.000 · 0.918 | 0.4(1.0) + 0.6(0.918) = 0.951 | 0.971 − 0.951 = **0.020** |

Slope separates the outcomes far better, so the tree asks about slope first. It then repeats inside each branch, and stops when leaves are pure or too small (to avoid overfitting).

## Ensembles: forests and boosting

A single tree overfits easily (see [[Bias,_Variance,_and_Overfitting]]). Ensembles fix that:
- **Random forests** average many trees, each trained on a random sample of data and features.
- **Gradient boosting** (XGBoost, LightGBM, CatBoost) adds trees one at a time, each correcting the errors of the ensemble so far.

On **tabular data**, rows of measured features such as motor temperatures, currents, error counts and battery data, boosted trees still match or beat deep networks in most benchmarks (Grinsztajn et al., 2022), train in seconds, and show which features matter. That makes them a good fit for predictive maintenance, battery health and fault classification on robots.

## Rule-based and expert systems

Rules written by people ("IF gripper current > 2 A AND object not detected THEN report jam") were the core of 1970s–80s **expert systems**. They are transparent and easy to certify, but brittle and costly to extend by hand. Robots still use them for safety interlocks and diagnostics, where predictability matters more than flexibility; see [[Knowledge_Representation_and_Reasoning]].

## Behaviour trees: structuring robot tasks

A **behaviour tree (BT)** is not learned from data. It is a hand-designed tree that decides which action to run, re-evaluated many times per second:

| Node | Behaviour |
|---|---|
| **Sequence** (→) | Run children in order; fail as soon as one fails |
| **Fallback / Selector** (?) | Try children in order; succeed as soon as one succeeds |
| **Condition** | Check something ("battery > 20%?") |
| **Action** | Do something ("navigate to dock"), returning running, success or failure |

```
? Fallback
├── → Sequence: [battery low?] → [navigate to dock] → [dock]
└── → Sequence: [have goal?] → [compute path] → [follow path]
        (on failure: ? Fallback → [clear costmap] → [spin] → [back up])
```

BTs are modular, readable and reactive, which is why [[Nav2_Navigation|Nav2]] is built around them (BehaviorTree.CPP) and why many manipulation frameworks use them. In 2025–26, language-model planners often *generate* behaviour trees or skill sequences, which can then be inspected and checked before the robot runs them.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Decision_Trees_and_Rule-Based_Systems]])
```
