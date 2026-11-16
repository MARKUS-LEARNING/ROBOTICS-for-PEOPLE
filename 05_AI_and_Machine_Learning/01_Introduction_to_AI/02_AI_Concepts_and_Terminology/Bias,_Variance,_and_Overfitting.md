---
title: Bias, Variance, and Overfitting
description: "Why models fail to generalise: bias (too simple to capture the pattern), variance (so flexible it fits the noise), and the overfitting and underfitting they cause. Includes a reproducible polynomial-fitting experiment with train and test errors, the remedies (more data, regularisation, augmentation, early stopping), the modern 'double descent' picture, and the robotics version of overfitting: failing under distribution shift and the sim-to-real gap."
aliases:
  - Overfitting
  - Underfitting
  - Bias-Variance Tradeoff
  - Generalization
tags:
  - machine-learning
  - statistics
  - robot-learning
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
  - "[[Machine_Learning]]"
  - "[[Supervised_Learning]]"
related:
  - "[[Optimization_and_Loss_Functions]]"
  - "[[Sim-to-Real_Transfer]]"
  - "[[Feature_Engineering_and_Data_Preprocessing]]"
  - "[[Robot_Learning_Datasets_and_Benchmarks]]"
key_takeaways:
  - "Test error has three parts: bias (systematic error from too simple a model), variance (sensitivity to the particular training sample) and irreducible noise."
  - "A model that is much better on training data than on held-out data has overfit; fixes are more and more varied data, regularisation, augmentation and early stopping."
  - "For robots the decisive test is a shift in conditions (new objects, lighting, floors, the real world after simulation); diverse data and domain randomisation are the robotics forms of regularisation."
sources:
  - "Geman, S., Bienenstock, E. and Doursat, R. (1992). Neural Networks and the Bias/Variance Dilemma. https://direct.mit.edu/neco/article/4/1/1/5624/Neural-Networks-and-the-Bias-Variance-Dilemma"
  - "Hastie, T., Tibshirani, R. and Friedman, J. (2009). The Elements of Statistical Learning, chapter 7: Model Assessment and Selection. https://hastie.su.domains/ElemStatLearn/"
  - "Belkin, M., Hsu, D., Ma, S. and Mandal, S. (2018). Reconciling modern machine learning practice and the bias-variance trade-off. https://arxiv.org/abs/1812.11118"
  - "Srivastava, N. et al. (2014). Dropout: A Simple Way to Prevent Neural Networks from Overfitting. https://jmlr.org/papers/v15/srivastava14a.html"
  - "Tobin, J. et al. (2017). Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World. https://arxiv.org/abs/1703.06907"
---

# Bias, Variance, and Overfitting

A model is useful only if it works on data it has not seen. **Overfitting** is learning the training examples, noise included, instead of the pattern behind them. **Underfitting** is failing to learn the pattern at all. Bias and variance are the two forces behind these failures.

> **Etymology.** *Bias* is from Old French *biais*, "slant, oblique": a systematic lean in one direction. *Variance* is from Latin *variare*, "to change": how much the result changes from one sample to another. Geman, Bienenstock and Doursat's 1992 paper framed neural-network learning as a "bias/variance dilemma".

## The decomposition

For squared error, the expected error of a model on a new point splits into three parts:

$$\mathbb E\big[(y - \hat f(x))^2\big] = \underbrace{\big(f(x) - \mathbb E[\hat f(x)]\big)^2}_{\text{bias}^2} + \underbrace{\mathbb E\big[(\hat f(x) - \mathbb E[\hat f(x)])^2\big]}_{\text{variance}} + \underbrace{\sigma^2}_{\text{noise}}$$

- **High bias**: the model is too simple; it is wrong in the same way whatever data you give it.
- **High variance**: the model is so flexible that a different training sample would give a very different fit.
- **Noise**: the error you cannot remove (sensor noise, genuinely random outcomes).

## A reproducible experiment

Twelve noisy samples of $y = \sin(2\pi x) + \varepsilon$, $\varepsilon \sim \mathcal N(0, 0.2^2)$, fitted with polynomials of three degrees and tested on 200 fresh points (NumPy, random seed 0):

| Polynomial degree | Training RMSE | Test RMSE | Diagnosis |
|---|---|---|---|
| 1 (a line) | 0.376 | 0.697 | **Underfit**: high bias |
| 3 | 0.130 | 0.256 | Good fit, close to the noise level of 0.2 |
| 9 | 0.064 | 0.956 | **Overfit**: high variance; best on training data, worst on test data |
| 9, with 200 training points | 0.193 | 0.203 | More data tames the variance |

The degree-9 polynomial passes almost exactly through the twelve noisy points, so its training error is tiny. Between the points it swings wildly, so its test error is the worst of all. The same flexible model trained on 200 points behaves well. **Training error alone tells you nothing about generalisation.**

## Remedies

| Problem | Symptom | Remedies |
|---|---|---|
| **Underfitting** | Training and test errors both high | Bigger model, better features, train longer |
| **Overfitting** | Training error ≪ test error | More data; **data augmentation**; **regularisation** (weight decay, dropout); **early stopping**; simpler model; ensembles |

## The modern twist: double descent

Very large neural networks often have enough parameters to fit their training data perfectly, yet they still generalise well. Belkin et al. (2018) showed that test error can fall, rise near the point where the model just fits the data, then fall *again* as the model grows further ("double descent"). The classical U-shaped trade-off still describes small models. For huge models, the training procedure (stochastic gradient descent, weight decay, data augmentation) acts as an implicit regulariser. This is part of why scaling worked.

## Overfitting in robotics

For a robot, "test data" means the real world tomorrow: different objects, lighting, floors, people. The robotics versions of overfitting are:

- **Memorising the scene.** A grasping policy trained in one kitchen learns that kitchen's countertop colour.
- **Distribution shift during deployment.** A policy's own small errors lead it into states that never appeared in its demonstrations (see [[Imitation_Learning]]).
- **The sim-to-real gap.** A policy that overfits the simulator's exact physics fails on hardware (see [[Sim-to-Real_Transfer]]).

The fixes are the same in spirit:
- **diverse data**: many scenes, objects and robots; pooled datasets;
- **domain randomisation**: randomise textures, lighting, masses and friction so the policy cannot rely on any one value (Tobin et al., 2017);
- **evaluation on held-out conditions**: new objects, new rooms, new robots.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Bias,_Variance,_and_Overfitting]])
```
