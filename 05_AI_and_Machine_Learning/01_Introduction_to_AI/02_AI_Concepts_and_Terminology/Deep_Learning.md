---
title: Deep Learning
description: "Deep learning trains many-layered neural networks that learn their own features from raw data. Covers why depth helps, the main architectures (MLPs, CNNs, recurrent networks, transformers, diffusion models), how training works, pretrained foundation models (CLIP, DINOv2, SAM), and a worked compute example contrasting a 1 kHz locomotion policy with a robot foundation model."
aliases:
  - DL
  - Deep Neural Networks
tags:
  - deep-learning
  - neural-networks
  - machine-learning
  - foundation-models
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
  - "[[Neural_Networks]]"
related:
  - "[[Transformers_and_Attention]]"
  - "[[Convolutional_Neural_Networks]]"
  - "[[Deep_Learning_in_Robotics]]"
  - "[[Generative_Models]]"
  - "[[Vision-Language-Action_Models]]"
key_takeaways:
  - "Deep networks stack many simple layers; each layer re-represents its input, so the network learns features (edges, parts, objects, affordances) instead of being given them."
  - "Transformers became the common architecture for language, vision and robot actions; pretrained foundation models (CLIP, DINOv2, SAM, VLMs) are the starting point for most robot perception and many policies."
  - "Model size sets where a network can run: a small locomotion policy runs at 1 kHz on a CPU, while a billions-of-parameters VLA needs a GPU and runs at a few to tens of hertz."
sources:
  - "LeCun, Y., Bengio, Y. and Hinton, G. (2015). Deep learning. https://www.nature.com/articles/nature14539"
  - "Goodfellow, I., Bengio, Y. and Courville, A. (2016). Deep Learning. https://www.deeplearningbook.org/"
  - "Krizhevsky, A., Sutskever, I. and Hinton, G. (2012). ImageNet Classification with Deep Convolutional Neural Networks. https://papers.nips.cc/paper/4824-imagenet-classification-with-deep-convolutional-neural-networks"
  - "He, K. et al. (2015). Deep Residual Learning for Image Recognition. https://arxiv.org/abs/1512.03385"
  - "Vaswani, A. et al. (2017). Attention Is All You Need. https://arxiv.org/abs/1706.03762"
  - "Radford, A. et al. (2021). Learning Transferable Visual Models From Natural Language Supervision. https://arxiv.org/abs/2103.00020"
  - "Oquab, M. et al. (2023). DINOv2: Learning Robust Visual Features without Supervision. https://arxiv.org/abs/2304.07193"
---

# Deep Learning

**Deep learning** is [[Machine_Learning|machine learning]] with neural networks that have many layers. Each layer transforms the output of the one before, so the network builds up its own hierarchy of features: from pixels to edges, to parts, to objects, to "graspable handle". Since 2012 it has transformed perception, and since the early 2020s, robot control as well.

> **Etymology.** *Deep* refers to the number of layers between input and output. The phrase "deep learning" entered machine learning around 2006, from work by Geoffrey Hinton and colleagues on training many-layered networks one layer at a time. Before then, most practical networks had one or two hidden layers.

## Why depth helps

A network with one hidden layer can approximate any reasonable function in principle (the universal approximation theorem), but it may need astronomically many units. Stacking layers lets a network **reuse** features: a "wheel" detector built from edge detectors is shared by every vehicle class. In practice, deeper networks learn better representations from the same data, as long as they can be trained. **Residual connections** (He et al., 2015), which add each block's input to its output, made networks of 100+ layers trainable and are now in almost every architecture.

## The architectures you will meet

| Architecture | Built for | Robotics use |
|---|---|---|
| **MLP** (multilayer perceptron) | Fixed-size vectors | Locomotion policies from joint states and IMU |
| **[[Convolutional_Neural_Networks\|CNN]]** | Images: local patterns, shift-invariance | Detection, segmentation, depth; ResNet image encoders |
| **[[RNN_and_LSTM\|RNN / LSTM]]** | Sequences, with memory | Older sequence models; partly replaced by transformers and state-space models |
| **[[Transformers_and_Attention\|Transformer]]** | Sets and sequences of tokens, with attention | Language, vision (ViT), and robot action models (VLAs) |
| **Diffusion and flow models** | Generating samples from a distribution | Action generation ([[Diffusion_Policy]]), video world models |

## How training works

1. **Forward pass**: compute predictions for a mini-batch of examples.
2. **Loss**: measure the error (cross-entropy for classes, squared error for continuous values).
3. **Backward pass**: backpropagation computes the gradient of the loss with respect to every parameter (see [[Neural_Networks_and_Backpropagation]]).
4. **Update**: an optimiser (usually Adam or AdamW) nudges the parameters against the gradient.

Repeat for millions of steps on GPUs. The practical craft lies in data quality, normalisation, learning-rate schedules, regularisation and evaluation on held-out data.

## Foundation models: start from someone else's training

Training from scratch is now rare. Robotics teams start from **pretrained** models and adapt them:

| Model | Learned from | What it gives a robot |
|---|---|---|
| **CLIP** (2021) | 400 million image–text pairs | Recognise objects named in words ("the blue mug") without task-specific training |
| **DINOv2** (2023) | 142 million images, self-supervised | General visual features for policies and correspondence |
| **SAM / SAM 2** (2023–24) | Over a billion masks | Segment any object in images and video |
| **Vision-language models** | Web images, video and text | Scene understanding and reasoning; the backbone of [[Vision-Language-Action_Models\|VLAs]] |

## Worked example: how big a network can a robot run?

**A locomotion policy** is an MLP with inputs of 50 values (joint positions and velocities, IMU, command), two hidden layers of 256 units, and 12 outputs (joint targets):

$$\underbrace{50\cdot256+256}_{13{,}056} + \underbrace{256\cdot256+256}_{65{,}792} + \underbrace{256\cdot12+12}_{3{,}084} = 81{,}932 \text{ parameters}$$

One forward pass costs about 2 floating-point operations per parameter, so about 164,000 FLOPs. At 1 kHz that is 0.16 GFLOP/s, trivial for any onboard CPU.

**A 3-billion-parameter VLA** processing about 800 image and text tokens per step needs roughly $2 \times 3\times10^9 \times 800 = 4.8\times10^{12}$ FLOPs per inference. A GPU sustaining 100 TFLOP/s would take about 48 ms, so a few to tens of decisions per second.

Both numbers are rough (attention, memory bandwidth and batching change them), but the 30-million-fold gap explains the 2026 architecture. Big models think at 1–10 Hz and output chunks of actions; small policies and classical controllers track those actions at 100–1,000 Hz.

## Limits

- **Data hunger.** Performance grows with data, and robot data is costly.
- **Brittleness.** Accuracy can collapse under distribution shift; adversarial inputs exist.
- **Opacity.** It is hard to explain or certify why a network acts as it does.
- **Compute and energy.** Large models need GPUs and power, which is difficult on battery-powered robots.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Deep_Learning]])
```
