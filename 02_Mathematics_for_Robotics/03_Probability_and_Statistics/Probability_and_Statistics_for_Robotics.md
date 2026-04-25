---
title: Probability and Statistics for Robotics
description: The mathematics of uncertainty. Every sensor is noisy, every model is approximate, and every action has unpredictable consequences. Probability gives a robot a principled way to reason under those conditions.
tags:
  - mathematics
  - probability
  - statistics
  - bayesian-inference
  - estimation
  - robotics
  - foundations
layout: default
category: mathematics
author: Jordan_Smith
date: 2025-05-02
permalink: /probability_and_statistics_for_robotics/
related:
  - "[[Bayesian_Inference]]"
  - "[[Kalman_Filter]]"
  - "[[Sensor_Fusion]]"
  - "[[SLAM]]"
  - "[[Machine_Learning]]"
---

# Probability and Statistics for Robotics

**Probability** is the branch of mathematics that quantifies *uncertainty*. **Statistics** is the applied discipline that uses probability to reason from data back to the mechanisms that produced it. In robotics these are inseparable: the robot collects noisy data (statistics) in order to update its beliefs about the world (probability), and then acts.

> **Etymology.** *Probability* comes from Latin *probabilis*, "worthy of approval, likely" — from *probare*, "to test, to prove". *Statistics* comes from Italian *statistica*, via German *Statistik*, originally referring to "the science of the state" — numerical facts kept by governments. Today both words point at the same underlying object: the mathematics of reasoning under incomplete information.

---

## Why probability owns robot autonomy

A deterministic robot can't exist in the real world. Consider the layers of uncertainty a mobile robot confronts at every control cycle:

- **Sensor noise.** A LiDAR range reading has millimeter-to-centimeter standard deviation. A wheel encoder accumulates error with every rotation.
- **Model error.** No robot matches its URDF (Unified Robot Description Format) exactly — link masses are slightly wrong, joint friction is nonlinear, tires flex.
- **Environment stochasticity.** A human walks into the lab. A door closes. Lighting changes.
- **Action noise.** A commanded torque of 2.3 Nm produces something close to, but not exactly, 2.3 Nm of actual torque.

Probability gives the robot a single currency — a distribution — to represent all of these together, and Bayesian updating gives it a single mathematical operation for fusing them.

---

## The core axioms (Kolmogorov)

Probability is built on three axioms over a sample space $\Omega$ and a collection of events:

1. $P(A) \geq 0$ for every event $A$
2. $P(\Omega) = 1$
3. For disjoint events $A_1, A_2, \ldots$: $P(\cup_i A_i) = \sum_i P(A_i)$

Every theorem — Bayes, law of large numbers, central limit theorem — is derived from these three lines and the machinery of measure theory.

---

## Random variables and distributions

A **random variable** $X$ is a measurable function from the sample space to $\mathbb{R}$ (or $\mathbb{R}^n$). Instead of describing outcomes directly, we describe $X$'s **distribution**.

### Discrete distributions

Characterized by a probability mass function $p(x) = P(X = x)$.

| Distribution | Models | Robotics use |
|---|---|---|
| Bernoulli | Single coin flip | Grasp success / failure |
| Binomial | $k$ successes in $n$ trials | Sample efficiency analysis |
| Categorical | One of $K$ classes | Object classification |
| Poisson | Count of rare events | Sensor drop-out events |

### Continuous distributions

Characterized by a probability density function $f(x)$ with $P(a \leq X \leq b) = \int_a^b f(x)\,dx$.

| Distribution | Density | Robotics use |
|---|---|---|
| Uniform | Constant on $[a,b]$ | Particle filter initialization |
| Gaussian (normal) | $\mathcal{N}(\mu, \sigma^2)$ | Sensor noise, Kalman filters |
| Multivariate Gaussian | $\mathcal{N}(\boldsymbol{\mu}, \Sigma)$ | State estimates, covariance |
| Exponential | $\lambda e^{-\lambda x}$ | Time between events |
| Beta | $\text{Beta}(\alpha, \beta)$ on $[0,1]$ | Bayesian prior on success rates |
| Dirichlet | Multivariate Beta on simplex | Bayesian prior on categorical |

**The Gaussian dominates robotics** for two reasons: (1) it is the maximum-entropy distribution with a given mean and variance (so picking Gaussian is the least-committal choice), and (2) linear functions of Gaussians are Gaussian, which gives closed-form Bayesian updates — the mathematical foundation of the Kalman filter.

### Multivariate Gaussian

For $\mathbf{x} \in \mathbb{R}^n$ with mean $\boldsymbol{\mu} \in \mathbb{R}^n$ and covariance $\Sigma \in \mathbb{R}^{n \times n}$ (symmetric, positive definite):

$$
\mathcal{N}(\mathbf{x}; \boldsymbol{\mu}, \Sigma) = \frac{1}{\sqrt{(2\pi)^n |\Sigma|}} \exp\left( -\tfrac{1}{2}(\mathbf{x}-\boldsymbol{\mu})^T \Sigma^{-1} (\mathbf{x}-\boldsymbol{\mu}) \right)
$$

Contours of constant density are ellipsoids whose axes are eigenvectors of $\Sigma$. This is how a robot's state uncertainty is usually visualized — the famous "uncertainty ellipse" around an estimated pose.

---

## Bayes' theorem

The mathematical heart of probabilistic robotics:

$$
\underbrace{P(\mathbf{x} \mid \mathbf{z})}_{\text{posterior}} = \frac{\overbrace{P(\mathbf{z} \mid \mathbf{x})}^{\text{likelihood}} \, \overbrace{P(\mathbf{x})}^{\text{prior}}}{\underbrace{P(\mathbf{z})}_{\text{evidence}}}
$$

Read it this way: **what you believed before** ($P(\mathbf{x})$), **multiplied by** **how consistent the data are with each hypothesis** ($P(\mathbf{z} \mid \mathbf{x})$), **normalized**, gives you **what you should believe now** ($P(\mathbf{x} \mid \mathbf{z})$).

**Recursive Bayesian filtering.** For time-indexed state $\mathbf{x}_t$ and measurement $\mathbf{z}_t$:

$$
\underbrace{\text{bel}(\mathbf{x}_t)}_{\text{posterior}} = \eta \, P(\mathbf{z}_t \mid \mathbf{x}_t) \int P(\mathbf{x}_t \mid \mathbf{x}_{t-1}, \mathbf{u}_t) \, \text{bel}(\mathbf{x}_{t-1}) \, d\mathbf{x}_{t-1}
$$

This single equation is the Bayes filter. Every localization, tracking, and SLAM algorithm is a special case:

| Algorithm | Special case of Bayes filter |
|---|---|
| Kalman filter | Linear Gaussian dynamics + Gaussian measurements |
| Extended Kalman filter | Linearized nonlinear dynamics + Gaussian |
| Unscented Kalman filter | Sigma-point linearization |
| Particle filter | Nonparametric (samples) — handles any distribution |
| Histogram filter | Discretized grid |

---

## Gaussian fusion — the workhorse

Two independent Gaussian measurements of the same quantity combine optimally:

$$
\mu_{\text{fused}} = \frac{\sigma_2^2 \mu_1 + \sigma_1^2 \mu_2}{\sigma_1^2 + \sigma_2^2}, \quad \sigma_{\text{fused}}^2 = \frac{\sigma_1^2 \sigma_2^2}{\sigma_1^2 + \sigma_2^2}
$$

The fused variance is smaller than either input variance — two noisy sensors together beat either alone, *provided the noise sources are independent*. This is the mathematical basis for multi-sensor fusion: IMU + wheel odometry + LiDAR + GPS all streaming into a single Kalman filter.

---

## Information theory in brief

Tightly related to probability, frequently used in robotics estimation and decision making:

- **Entropy** $H(X) = -\sum p(x) \log p(x)$ — information content / uncertainty
- **Mutual information** $I(X; Y) = H(X) - H(X \mid Y)$ — how much $Y$ tells you about $X$
- **KL divergence** $D_{\mathrm{KL}}(p \Vert q) = \sum p(x) \log \frac{p(x)}{q(x)}$ — asymmetric distance between distributions

**Robotics use:**
- Active SLAM picks sensing actions that maximize mutual information between the map and future observations.
- Variational inference minimizes KL divergence between tractable and true posteriors.
- Information-gain exploration drives a robot toward regions of map uncertainty.

---

## Statistics: reasoning from data

The inverse problem: given a sample $\mathbf{x}_1, \ldots, \mathbf{x}_N$ drawn from an unknown distribution, infer that distribution.

### Point estimators

- **Sample mean** $\bar{\mathbf{x}} = \tfrac{1}{N}\sum_i \mathbf{x}_i$ — unbiased estimator of $\boldsymbol{\mu}$
- **Sample covariance** $\hat{\Sigma} = \tfrac{1}{N-1} \sum_i (\mathbf{x}_i - \bar{\mathbf{x}})(\mathbf{x}_i - \bar{\mathbf{x}})^T$

### Fundamental laws

- **Law of large numbers:** sample averages converge to the true mean
- **Central limit theorem:** sums of independent random variables tend to Gaussian, regardless of the underlying distribution

The CLT is why Gaussian noise models are so often justified even when the micro-physics of a sensor is non-Gaussian — many independent error sources add up, so the aggregate looks normal.

### Estimation frameworks

| Framework | Estimator |
|---|---|
| Maximum likelihood (MLE) | $\hat{\theta} = \arg\max_\theta P(\mathbf{data} \mid \theta)$ |
| Maximum a posteriori (MAP) | $\hat{\theta} = \arg\max_\theta P(\theta \mid \mathbf{data})$ |
| Minimum mean squared error (MMSE) | $\hat{\theta} = \mathbb{E}[\theta \mid \mathbf{data}]$ |

MLE ignores priors. MAP incorporates them. MMSE gives the Bayes-optimal point estimate under squared loss.

---

## Probabilistic algorithms in the robotics stack

| Layer | Probabilistic method |
|---|---|
| Localization | Particle filter (MCL), EKF-localization |
| SLAM | GraphSLAM, EKF-SLAM, FastSLAM (particle filter over poses + Gaussian map) |
| Sensor fusion | Kalman filter, UKF, factor graphs with iSAM2 |
| Object tracking | Multi-hypothesis tracking, JPDA |
| Grasping | Uncertainty-aware grasp planning, GraspNet |
| Learning | Gaussian processes, Bayesian neural networks |
| Decision making | POMDPs (partially observable Markov decision processes) |
| Exploration | Information-gain-maximizing policies |

---

## Recommended references

- Thrun, Burgard & Fox, *Probabilistic Robotics* — the canonical text for this topic
- Timothy Barfoot, *State Estimation for Robotics* — modern, matrix-Lie-group aware
- Christopher Bishop, *Pattern Recognition and Machine Learning* — strong Bayesian foundations
- Kevin Murphy, *Probabilistic Machine Learning* — comprehensive and current

---

## Dataview Plugin Features

### List of related concepts

```dataview
LIST FROM #probability OR #bayesian-inference WHERE contains(file.outlinks, [[Probability_and_Statistics_for_Robotics]])
```
