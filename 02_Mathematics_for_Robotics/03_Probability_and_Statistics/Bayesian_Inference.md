---
title: Bayesian Inference
description: The mathematical procedure for updating beliefs in light of evidence. The engine underneath every probabilistic robot — localization, SLAM, sensor fusion, tracking, and modern learning.
tags:
  - mathematics
  - probability
  - bayesian-inference
  - estimation
  - robotics
  - sensor-fusion
layout: default
category: mathematics
author: Jordan_Smith
date: 2025-05-02
permalink: /bayesian_inference/
related:
  - "[[Probability_and_Statistics_for_Robotics]]"
  - "[[Kalman_Filter]]"
  - "[[Sensor_Fusion]]"
  - "[[SLAM]]"
  - "[[Machine_Learning]]"
---

# Bayesian Inference

**Bayesian inference** is the procedure for updating a probability distribution over some quantity as new evidence arrives. Named after Reverend Thomas Bayes (1701-1761), whose posthumously published theorem underpins the entire framework.

> **Etymology.** *Bayesian* is the adjective form of Bayes. *Inference* comes from Latin *inferre*, "to carry in" or "to bring in" — to carry conclusions in from evidence.

---

## Bayes' theorem

$$
\underbrace{P(\mathbf{x} \mid \mathbf{z})}_{\text{posterior}} = \frac{\overbrace{P(\mathbf{z} \mid \mathbf{x})}^{\text{likelihood}} \, \overbrace{P(\mathbf{x})}^{\text{prior}}}{\underbrace{P(\mathbf{z})}_{\text{evidence}}}
$$

- **Prior** $P(\mathbf{x})$: what you believed about $\mathbf{x}$ before seeing data.
- **Likelihood** $P(\mathbf{z} \mid \mathbf{x})$: how consistent the data $\mathbf{z}$ are with each value of $\mathbf{x}$.
- **Posterior** $P(\mathbf{x} \mid \mathbf{z})$: updated belief after seeing the data.
- **Evidence** $P(\mathbf{z}) = \int P(\mathbf{z} \mid \mathbf{x}) P(\mathbf{x})\, d\mathbf{x}$: a normalizing constant.

For sequential data (which robotics almost always has), apply Bayes recursively:

$$
P(\mathbf{x} \mid \mathbf{z}_1, \mathbf{z}_2) \propto P(\mathbf{z}_2 \mid \mathbf{x}) \cdot P(\mathbf{x} \mid \mathbf{z}_1)
$$

Yesterday's posterior becomes today's prior.

---

## The recursive Bayes filter

The master equation of [[Probabilistic_Robotics]]:

$$
\text{bel}(\mathbf{x}_t) = \eta \, P(\mathbf{z}_t \mid \mathbf{x}_t) \int P(\mathbf{x}_t \mid \mathbf{u}_t, \mathbf{x}_{t-1}) \, \text{bel}(\mathbf{x}_{t-1}) \, d\mathbf{x}_{t-1}
$$

Two steps at each time $t$:

1. **Prediction** (motion update): propagate the belief forward using the motion model $P(\mathbf{x}_t \mid \mathbf{u}_t, \mathbf{x}_{t-1})$.
2. **Correction** (measurement update): multiply by the likelihood $P(\mathbf{z}_t \mid \mathbf{x}_t)$ and normalize.

Every practical filter is a computational approximation of this integral:

| Approximation | Name |
|---|---|
| Belief as Gaussian; linear motion/measurement | [[Kalman_Filter]] |
| Belief as Gaussian; linearize nonlinearities | [[Extended_Kalman_Filter]] |
| Belief as Gaussian; sigma-point propagation | Unscented Kalman filter |
| Belief as weighted samples | [[Particle_Filter]] |
| Belief as discrete grid | Histogram filter |
| Belief as mixture of Gaussians | Multi-hypothesis tracker |

---

## Why Bayesian reasoning fits robotics so naturally

1. **Robots must act under uncertainty.** Sensors are noisy; models are wrong. Bayesian posteriors give calibrated uncertainty, not just point estimates.
2. **Fusion is free.** Incorporating a new sensor reduces to multiplying in its likelihood. This is why adding a GPS measurement to a wheel-odometry filter is one line of math, not a redesign.
3. **Priors encode engineering knowledge.** A robot's map, joint-limit constraints, and physics priors all slot naturally into $P(\mathbf{x})$.
4. **Composability.** Markov chains, HMMs, Kalman filters, SLAM, and dynamic Bayes nets are all built from the same primitives.

---

## Maximum likelihood vs. maximum a posteriori (MAP)

Two common point-estimate extractions from a Bayesian framework:

$$
\hat{\mathbf{x}}_{\text{MLE}} = \arg\max_{\mathbf{x}} P(\mathbf{z} \mid \mathbf{x}) \qquad \text{(likelihood only)}
$$

$$
\hat{\mathbf{x}}_{\text{MAP}} = \arg\max_{\mathbf{x}} P(\mathbf{z} \mid \mathbf{x}) P(\mathbf{x}) \qquad \text{(posterior mode)}
$$

MAP reduces to MLE under a flat prior. MAP with a Gaussian prior is equivalent to L2-regularized (ridge) regression. Most SLAM back-ends solve a nonlinear-least-squares problem that is really MAP estimation with Gaussian residuals.

---

## Conjugate priors — closed-form Bayes

A prior is *conjugate* to a likelihood if the posterior lies in the same family as the prior. Crucial conjugate pairs for robotics:

| Prior | Likelihood | Posterior |
|---|---|---|
| Gaussian $\mathcal{N}(\mu_0, \sigma_0^2)$ | Gaussian (known variance) | Gaussian |
| Beta $\text{Beta}(\alpha, \beta)$ | Bernoulli / binomial | Beta |
| Dirichlet | Categorical / multinomial | Dirichlet |
| Gamma | Poisson | Gamma |
| Normal-inverse-Wishart | Multivariate Gaussian (unknown $\mu, \Sigma$) | Normal-inverse-Wishart |

The **Gaussian-Gaussian conjugacy** is the algebraic reason the Kalman filter is closed-form and so fast: multiplying two Gaussians gives another Gaussian, and propagating a Gaussian through a linear map gives another Gaussian.

---

## Gaussian fusion — the workhorse identity

Two independent Gaussian measurements $\mathcal{N}(\mu_1, \sigma_1^2)$ and $\mathcal{N}(\mu_2, \sigma_2^2)$ of the same scalar combine to:

$$
\mu_{\text{fused}} = \frac{\sigma_2^2 \mu_1 + \sigma_1^2 \mu_2}{\sigma_1^2 + \sigma_2^2}, \quad \sigma_{\text{fused}}^2 = \frac{\sigma_1^2 \sigma_2^2}{\sigma_1^2 + \sigma_2^2}
$$

The fused variance is smaller than either input — this is why multi-sensor fusion consistently beats single-sensor estimates. The multivariate version extends to covariance matrices via the **information filter** formulation:

$$
\Sigma_{\text{fused}}^{-1} = \Sigma_1^{-1} + \Sigma_2^{-1}, \quad \Sigma_{\text{fused}}^{-1} \boldsymbol{\mu}_{\text{fused}} = \Sigma_1^{-1} \boldsymbol{\mu}_1 + \Sigma_2^{-1} \boldsymbol{\mu}_2
$$

Inverse covariance (the **information matrix**) is the natural parameterization for fusion because it simply adds.

---

## Approximate inference when exact is impossible

When the posterior has no closed form, approximate:

| Method | Idea | Robotics use |
|---|---|---|
| **Laplace approximation** | Fit a Gaussian at the MAP | Bundle adjustment, pose-graph optimization |
| **Variational inference** | Minimize KL to a tractable family | Variational SLAM, variational autoencoders |
| **MCMC** (Metropolis-Hastings, HMC) | Draw correlated samples from posterior | Bayesian deep learning |
| **Sequential Monte Carlo** (particle filter) | Weighted samples, evolved in time | Nonlinear/non-Gaussian localization |
| **Expectation-Maximization** | Alternate posterior + parameter updates | Calibration, system identification |

---

## Bayesian perspectives on modern problems

- **Bayesian occupancy grids** turn a LiDAR stream into a probabilistic map where each cell stores $P(\text{occupied})$.
- **GraphSLAM** formulates simultaneous localization and mapping as MAP estimation on a factor graph with Gaussian factors.
- **POMDPs** (partially observable Markov decision processes) lift Bayes to include decision-making: the belief state is the posterior, and planning runs on that belief.
- **Bayesian deep learning** places priors over neural-network weights to produce uncertainty estimates — useful for safe robotics.

---

## Recommended reading

- Thrun, Burgard, Fox — *Probabilistic Robotics* (canonical)
- Barfoot — *State Estimation for Robotics* (modern, matrix-Lie-group aware)
- Murphy — *Probabilistic Machine Learning* (comprehensive reference)
- Dellaert & Kaess — *Factor Graphs for Robot Perception*

---

## Dataview

```dataview
LIST FROM #bayesian-inference OR #probability WHERE contains(file.outlinks, [[Bayesian_Inference]])
```
