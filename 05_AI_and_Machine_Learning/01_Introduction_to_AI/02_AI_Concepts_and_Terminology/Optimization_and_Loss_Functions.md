---
title: Optimization and Loss Functions
description: "Learning is optimisation: choose a loss that measures what you care about, then adjust parameters to minimise it. Covers the common losses, gradient descent with a computed example of slow, exact and divergent learning rates, SGD, momentum and Adam/AdamW, convex versus non-convex problems, and how the same tools run robot controllers, trajectory optimisation and system identification."
aliases:
  - Loss Function
  - Gradient Descent
  - Optimization
  - Cost Function
tags:
  - optimization
  - machine-learning
  - deep-learning
type: concept
level: intermediate
subfield:
  - robot-learning
  - mathematics
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Calculus_for_Robotics]]"
  - "[[Machine_Learning]]"
related:
  - "[[Neural_Networks_and_Backpropagation]]"
  - "[[Bias,_Variance,_and_Overfitting]]"
  - "[[Model_Predictive_Control_(MPC)]]"
  - "[[Diffusion_Policy]]"
key_takeaways:
  - "The loss defines what the model is trying to do; choose it to match the real cost of errors (squared error for continuous values, cross-entropy for classes, robust losses when there are outliers)."
  - "Gradient descent steps against the gradient; the learning rate decides whether it crawls, converges or diverges. Adam and AdamW, with a warm-up and decaying schedule, are the defaults for neural networks."
  - "Robotics optimises everywhere: training policies, fitting dynamics models, and solving trajectory and MPC problems online, often within milliseconds."
sources:
  - "Boyd, S. and Vandenberghe, L. (2004). Convex Optimization. https://web.stanford.edu/~boyd/cvxbook/"
  - "Bottou, L., Curtis, F. E. and Nocedal, J. (2016). Optimization Methods for Large-Scale Machine Learning. https://arxiv.org/abs/1606.04838"
  - "Kingma, D. P. and Ba, J. (2014). Adam: A Method for Stochastic Optimization. https://arxiv.org/abs/1412.6980"
  - "Loshchilov, I. and Hutter, F. (2017). Decoupled Weight Decay Regularization. https://arxiv.org/abs/1711.05101"
  - "Goodfellow, I., Bengio, Y. and Courville, A. (2016). Deep Learning, chapter 8: Optimization for Training Deep Models. https://www.deeplearningbook.org/"
---

# Optimization and Loss Functions

Every learning algorithm, and many robot controllers, comes down to the same two steps: write down a number that says how bad things are (the **loss** or **cost**), then change the parameters to make that number smaller (**optimisation**).

> **Etymology.** *Optimise* is from Latin *optimus*, "best". *Loss* is from Old English *los*, "destruction": what you lose by being wrong. Engineers often say *cost*; reinforcement learning flips the sign and says *reward*.

## Choosing a loss

| Loss | Formula (one example) | Use when | Robot example |
|---|---|---|---|
| **Squared error (L2)** | $(y - \hat y)^2$ | Continuous targets with Gaussian-like noise | Depth, joint positions, behaviour cloning of actions |
| **Absolute error (L1)** | $\lvert y - \hat y\rvert$ | Outliers present; want the median | Action chunks in ACT |
| **Huber** | L2 near zero, L1 far away | Mostly Gaussian noise plus some outliers | Value functions in deep RL |
| **Cross-entropy** | $-\sum_c y_c \log \hat p_c$ | Classes and probabilities | Object classes; action tokens in token-based VLAs |
| **KL divergence** | $\sum p \log(p/q)$ | Matching distributions | VAE regularisation; keeping RL policy updates small (TRPO, KL-penalised PPO) |
| **Denoising / flow-matching** | $\lVert \epsilon - \epsilon_\theta \rVert^2$ | Generative models | [[Diffusion_Policy]] and flow-matching action heads |

The loss *is* the specification. A policy trained with L2 on multimodal demonstrations averages two good behaviours into a bad one, a failure that comes from the loss, not from the network (see [[Generative_Models]]).

## Gradient descent, computed

Minimise $L(w) = (w - 3)^2$, whose gradient is $2(w - 3)$, starting from $w = 0$, with update $w \leftarrow w - \eta \cdot 2(w - 3)$:

| Learning rate $\eta$ | $w$ after steps 1, 2, 3, 4, 5 | Behaviour |
|---|---|---|
| 0.1 | 0.6, 1.08, 1.464, 1.771, 2.017 | Steady convergence toward 3 |
| 0.5 | 3, 3, 3, 3, 3 | Exact in one step (lucky: matches the curvature) |
| 1.1 | 6.6, −1.32, 8.18, −3.22, 10.47 | Overshoots further each step: **diverges** |

On a quadratic with curvature $a$ (here $L'' = 2$), gradient descent converges only if $\eta < 2/a$, here $\eta < 1$. Neural-network losses have different curvature in different directions, so one learning rate is too big for some directions and too small for others. That is the problem adaptive optimisers address.

## Optimisers used in practice

- **Stochastic gradient descent (SGD)**: use the gradient of a random mini-batch instead of all data. It is cheap and noisy, and the noise even helps it escape poor regions.
- **Momentum**: accumulate a velocity so consistent directions speed up and oscillations cancel.
- **Adam**: per-parameter step sizes from running averages of the gradient and its square. Robust defaults; the most used optimiser in deep learning.
- **AdamW**: Adam with weight decay applied correctly (decoupled). The standard for transformers and robot foundation models.
- **Schedules**: a short linear **warm-up**, then cosine or linear **decay** of the learning rate.

## Convex and non-convex

A **convex** problem has one valley, so any local minimum is the global one, and solvers come with guarantees and predictable run times. Many robotics problems are made convex on purpose for this reason: least-squares calibration, quadratic programs in [[Model_Predictive_Control_(MPC)|MPC]] and whole-body control. Neural-network training is **non-convex**, with many valleys and saddle points, but in very high dimensions SGD-type methods find good solutions reliably.

## Optimisation on the robot

| Problem | Variables | Loss / cost | Typical solver and time |
|---|---|---|---|
| Train a policy | Network weights | Imitation or RL objective | AdamW, hours to weeks on GPUs |
| System identification | Masses, friction, motor constants | Mismatch between predicted and measured motion | Least squares, gradient-based, offline |
| Inverse kinematics | Joint angles | Distance from target pose | Damped least squares, microseconds |
| MPC / trajectory optimisation | Future controls and states | Tracking error + effort, subject to constraints | QP or nonlinear solvers, 1–20 ms, every control cycle |

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Optimization_and_Loss_Functions]])
```
