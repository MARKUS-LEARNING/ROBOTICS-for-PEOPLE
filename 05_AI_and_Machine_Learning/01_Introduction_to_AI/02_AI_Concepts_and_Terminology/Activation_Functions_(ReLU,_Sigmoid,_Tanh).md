---
title: Activation Functions (ReLU, Sigmoid, Tanh)
description: "Activation functions are the nonlinearities between a neural network's layers. Compares sigmoid, tanh, ReLU and the GELU and SiLU functions used in today's transformers, with a computed table of values, the derivative that causes vanishing gradients, and practical guidance for robot policies, including bounded outputs for motor commands."
aliases:
  - Activation Functions
  - ReLU
  - Sigmoid
  - GELU
tags:
  - neural-networks
  - deep-learning
  - machine-learning
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
  - "[[Neural_Networks]]"
related:
  - "[[Neural_Networks_and_Backpropagation]]"
  - "[[Deep_Learning]]"
  - "[[Transformers_and_Attention]]"
key_takeaways:
  - "Without a nonlinear activation, a deep network is just one linear map; the activation is what lets layers build complex functions."
  - "Sigmoid and tanh saturate (their gradients vanish for large inputs); ReLU fixed this for hidden layers, and smooth variants (GELU, SiLU) are standard in transformers."
  - "Choose output activations by what the output means: tanh to bound motor commands to [−1, 1], softmax for classes, none for unbounded regression."
sources:
  - "Glorot, X., Bordes, A. and Bengio, Y. (2011). Deep Sparse Rectifier Neural Networks. https://proceedings.mlr.press/v15/glorot11a.html"
  - "Hendrycks, D. and Gimpel, K. (2016). Gaussian Error Linear Units (GELUs). https://arxiv.org/abs/1606.08415"
  - "Ramachandran, P., Zoph, B. and Le, Q. V. (2017). Searching for Activation Functions. https://arxiv.org/abs/1710.05941"
  - "Goodfellow, I., Bengio, Y. and Courville, A. (2016). Deep Learning, chapter 6.3: Hidden Units. https://www.deeplearningbook.org/"
---

# Activation Functions (ReLU, Sigmoid, Tanh)

An **activation function** is the nonlinear function a neuron applies to its weighted sum. It is what makes a [[Neural_Networks|neural network]] more than a linear equation: stack linear layers without activations and you get one linear layer.

> **Etymology.** From the neuroscience picture of a neuron *activating* (firing) once its input crosses a threshold. The earliest artificial neurons used a hard step function; differentiable activations replaced it so networks could be trained by gradient descent.

## The common functions

| Function | Formula | Range | Where used |
|---|---|---|---|
| **Sigmoid** | $\sigma(z) = 1/(1+e^{-z})$ | (0, 1) | Probabilities of yes/no outputs; gates in LSTMs |
| **Tanh** | $\tanh z$ | (−1, 1) | Bounded outputs; older hidden layers |
| **ReLU** | $\max(0, z)$ | [0, ∞) | Default hidden activation for MLPs and CNNs since ~2011 |
| **GELU** | $z\,\Phi(z)$ ($\Phi$ = Gaussian CDF) | ≈ (−0.17, ∞) | Transformers (BERT, GPT, ViT) |
| **SiLU / Swish** | $z\,\sigma(z)$ | ≈ (−0.28, ∞) | Many vision and language models; inside SwiGLU layers |

### Computed values

| $z$ | ReLU | Sigmoid | Tanh | GELU | SiLU |
|---|---|---|---|---|---|
| −2 | 0 | 0.1192 | −0.9640 | −0.0455 | −0.2384 |
| 0 | 0 | 0.5 | 0 | 0 | 0 |
| 2 | 2 | 0.8808 | 0.9640 | 1.9545 | 1.7616 |

GELU and SiLU behave like ReLU for large positive inputs, but they are smooth near zero and let a little negative signal through. That tends to train slightly better in large models.

## Why sigmoid fell out of hidden layers

The sigmoid's derivative $\sigma'(z) = \sigma(z)(1-\sigma(z))$ is at most 0.25 (at $z = 0$) and nearly zero for $|z| > 4$. Backpropagation multiplies one such factor per layer, so after 10 sigmoid layers the gradient is scaled by at most $0.25^{10} \approx 9.5 \times 10^{-7}$. Early layers barely learn: the **vanishing gradient** problem. ReLU's derivative is exactly 1 for positive inputs, which keeps gradients alive. This is one reason deep networks became trainable around 2011–2012 (see [[Neural_Networks_and_Backpropagation]]).

ReLU's own failure mode is the **dying ReLU**: a neuron whose input is always negative outputs zero, gets zero gradient, and never recovers. Leaky ReLU (a small slope for $z < 0$), GELU and SiLU avoid it.

## Choosing activations for robot networks

- **Hidden layers.** ReLU, ELU or SiLU for small policy MLPs; GELU or SwiGLU inside transformers (the backbone usually decides for you).
- **Outputs that are motor commands.** `tanh` bounds actions to [−1, 1], which is then scaled to joint or velocity limits. Many RL policies (for example SAC) squash a Gaussian sample through tanh for exactly this reason.
- **Probabilities.** Sigmoid for one yes/no (grasp success), softmax for one-of-many (object class, discretised action bins in token-based VLAs).
- **Unbounded regression.** No activation on the output (depth, force, position error).
- **Smoothness matters for control.** Policies with smooth activations (ELU, SiLU, tanh) give smoother action changes than ReLU, which some locomotion work prefers to reduce motor jitter.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Activation_Functions_(ReLU,_Sigmoid,_Tanh)]])
```
