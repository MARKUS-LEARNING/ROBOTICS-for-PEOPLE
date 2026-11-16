---
title: Autoencoders
description: "An autoencoder learns to compress its input into a small code and reconstruct it, discovering structure without labels. Covers the plain, variational (VAE), vector-quantised (VQ-VAE) and masked (MAE) variants, a worked compression example, and their roles in 2026 robotics: visual pretraining for policies, latent world models, action tokenizers, and anomaly detection."
aliases:
  - Autoencoder
  - VAE
  - Variational Autoencoder
  - VQ-VAE
  - Masked Autoencoder
tags:
  - neural-networks
  - deep-learning
  - generative-models
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
  - "[[Neural_Networks]]"
  - "[[Unsupervised_Learning]]"
related:
  - "[[Generative_Models]]"
  - "[[World_Models]]"
  - "[[Imitation_Learning]]"
  - "[[Transformers_and_Attention]]"
key_takeaways:
  - "An autoencoder squeezes data through a small bottleneck and learns to rebuild it, so the bottleneck must capture what matters; no labels are needed."
  - "Variants matter: VAEs give a smooth latent space you can sample, VQ-VAEs give discrete tokens, masked autoencoders learn strong visual features by filling in hidden patches."
  - "In robotics they compress camera images for policies and world models, tokenise actions, structure demonstrations (ACT's CVAE), and flag anomalies by high reconstruction error."
sources:
  - "Hinton, G. E. and Salakhutdinov, R. R. (2006). Reducing the Dimensionality of Data with Neural Networks. https://www.science.org/doi/10.1126/science.1127647"
  - "Kingma, D. P. and Welling, M. (2013). Auto-Encoding Variational Bayes. https://arxiv.org/abs/1312.6114"
  - "van den Oord, A., Vinyals, O. and Kavukcuoglu, K. (2017). Neural Discrete Representation Learning. https://arxiv.org/abs/1711.00937"
  - "He, K. et al. (2021). Masked Autoencoders Are Scalable Vision Learners. https://arxiv.org/abs/2111.06377"
  - "Xiao, T. et al. (2022). Masked Visual Pre-training for Motor Control. https://arxiv.org/abs/2203.06173"
  - "Zhao, T. et al. (2023). Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware. https://arxiv.org/abs/2304.13705"
  - "Lee, S. et al. (2024). Behavior Generation with Latent Actions. https://arxiv.org/abs/2403.03181"
---

# Autoencoders

An **autoencoder** is a neural network trained to output its own input. That sounds pointless until you force the data through a narrow middle layer, the **bottleneck** or **latent code**. To rebuild a 27,648-number camera image from 64 numbers, the network must discover what actually varies in the images: object positions, the arm's pose, the lighting. No labels are needed.

> **Etymology.** Greek *autos*, "self": it *encodes itself*. The idea dates to the 1980s; Hinton and Salakhutdinov's 2006 *Science* paper showed deep autoencoders beating PCA at compressing images and helped launch deep learning.

## Structure

$$\mathbf z = E_\phi(\mathbf x), \qquad \hat{\mathbf x} = D_\theta(\mathbf z), \qquad L = \lVert \mathbf x - \hat{\mathbf x}\rVert^2$$

The **encoder** $E$ compresses and the **decoder** $D$ reconstructs. After training you often keep only the encoder, as a feature extractor.

### Worked example: how much compression?

A 96×96 RGB image has $96 \times 96 \times 3 = 27{,}648$ values. With a 64-dimensional latent, the autoencoder compresses by $27{,}648 / 64 = 432\times$. A linear autoencoder with squared error learns the same subspace as PCA; the nonlinear one can follow curved structure (an arm's joint angles map to images along a curved surface), so it reconstructs better at the same size. Make the bottleneck too small and details that matter for control, such as a thin cable or a gripper's opening, are lost. Always check reconstructions of task-critical details.

## The variants

| Variant | Twist | What it is good for |
|---|---|---|
| **Plain / denoising** | Reconstruct a clean input from a corrupted one | Robust features, denoising sensor data |
| **Variational (VAE)** | Encoder outputs a distribution; a KL penalty keeps the latent space smooth and Gaussian | Sampling new data, smooth latent dynamics; a component of many world models |
| **Conditional VAE** | Encoder and decoder also see a condition (the observation) | Capturing the "style" of demonstrations: ACT's action chunks (2023) |
| **VQ-VAE** | Latents snapped to a learned codebook of discrete vectors | Turning images, video or actions into **tokens** for transformers |
| **Masked autoencoder (MAE)** | Hide most image patches (e.g. 75%) and reconstruct them | Self-supervised visual pretraining; strong features for policies |

The VAE trains on the **evidence lower bound**:

$$\mathcal L = \underbrace{\mathbb E_{q_\phi(\mathbf z|\mathbf x)}\big[-\log p_\theta(\mathbf x|\mathbf z)\big]}_{\text{reconstruction}} + \beta\,\underbrace{\mathrm{KL}\big(q_\phi(\mathbf z|\mathbf x)\,\Vert\,\mathcal N(0, I)\big)}_{\text{keep latents regular}}$$

## Autoencoders in robotics (2026)

- **Visual pretraining for control.** Masked visual pretraining (MVP, 2022) trained an MAE on egocentric and manipulation video, then froze it as the image encoder for robot policies. It outperformed ImageNet-trained encoders on motor-control tasks.
- **Latent world models.** World models such as Dreamer, and video world models, predict the future in a learned latent space rather than in pixels, using a VAE-style encoder (see [[World_Models]]).
- **Action tokenizers.** VQ-BeT (2024) uses a VQ-VAE to turn continuous action chunks into discrete tokens, so a transformer can predict multimodal behaviour; several VLAs tokenise actions in related ways.
- **Demonstration structure.** ACT (ALOHA, 2023) wraps its transformer policy in a conditional VAE, so it can represent different ways a human performed the same task (see [[Imitation_Learning]]).
- **Anomaly detection.** Train on normal operation; at run time, a high reconstruction error flags something new: a fault, an unexpected object, a sensor failure.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Autoencoders]])
```
