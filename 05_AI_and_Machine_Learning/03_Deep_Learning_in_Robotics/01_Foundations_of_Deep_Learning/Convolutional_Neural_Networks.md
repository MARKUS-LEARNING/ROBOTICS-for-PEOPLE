---
title: Convolutional Neural Networks
description: "Convolutional neural networks (CNNs) process images by sliding small learned filters across them, which shares weights and respects locality. Covers convolution with a computed edge-detection example, output sizes and parameter counts, pooling and strides, the architectures that matter (LeNet, AlexNet, ResNet, ConvNeXt), and CNNs versus vision transformers in 2026."
aliases:
  - CNN
  - ConvNet
  - CNNs
tags:
  - neural-networks
  - deep-learning
  - computer-vision
type: concept
level: intermediate
subfield:
  - robot-learning
  - perception
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Neural_Networks]]"
  - "[[Linear_Algebra_for_Robotics]]"
related:
  - "[[CNN_in_Robotics]]"
  - "[[Transformers_and_Attention]]"
  - "[[Deep_Learning]]"
  - "[[Computer_Vision_in_Robotics]]"
key_takeaways:
  - "A convolution layer slides the same small filter over the whole image, so it needs few weights and detects a pattern wherever it appears."
  - "Stacked convolutions see progressively larger regions: early layers find edges, later layers find parts and objects."
  - "In 2026 CNNs remain the efficient choice on embedded robot hardware (detectors such as YOLO, ResNet encoders in policies), while vision transformers dominate large pretrained models."
sources:
  - "LeCun, Y., Bottou, L., Bengio, Y. and Haffner, P. (1998). Gradient-based learning applied to document recognition. https://ieeexplore.ieee.org/document/726791"
  - "Krizhevsky, A., Sutskever, I. and Hinton, G. (2012). ImageNet Classification with Deep Convolutional Neural Networks. https://papers.nips.cc/paper/4824-imagenet-classification-with-deep-convolutional-neural-networks"
  - "He, K. et al. (2015). Deep Residual Learning for Image Recognition. https://arxiv.org/abs/1512.03385"
  - "Liu, Z. et al. (2022). A ConvNet for the 2020s. https://arxiv.org/abs/2201.03545"
  - "Dosovitskiy, A. et al. (2020). An Image is Worth 16x16 Words: Transformers for Image Recognition at Scale. https://arxiv.org/abs/2010.11929"
---

# Convolutional Neural Networks

A **convolutional neural network (CNN)** is a [[Neural_Networks|neural network]] designed for images and other grid-shaped data. Instead of connecting every pixel to every neuron, it slides small learned **filters** across the image, which matches two facts about images: nearby pixels belong together, and an object looks the same wherever it appears.

> **Etymology.** *Convolution* is from Latin *convolvere*, "to roll together": the filter is rolled across the input, combining it with each patch. Kunihiko Fukushima's *Neocognitron* (1980) introduced the layered idea, and Yann LeCun's LeNet (1989–98) made CNNs trainable with backpropagation, reading cheque digits for US banks.

## Convolution, computed

A 3×3 filter is multiplied element by element with each 3×3 patch of the image and summed, giving one output value per position. The **Sobel** filter below responds to left-to-right brightness changes (vertical edges):

$$K = \begin{bmatrix}-1&0&1\\-2&0&2\\-1&0&1\end{bmatrix}$$

Apply it to a 5×5 patch whose left three columns are dark (10) and right two bright (80):

| Patch rows (all equal) | → | Output (3×3, "valid" convolution) |
|---|---|---|
| 10 10 10 80 80 | | 0 280 280 |

The leftmost window covers only dark pixels: $(-1-2-1)\cdot10 + (1+2+1)\cdot10 = 0$. The next window straddles the edge: $(-1-2-1)\cdot 10 + (1+2+1)\cdot 80 = 280$. The filter "lights up" exactly where the edge is. A CNN *learns* thousands of such filters from data, rather than having them designed by hand.

## Sizes and parameters

For input width $W$, filter size $k$, padding $p$ and stride $s$, the output width is $\lfloor (W + 2p - k)/s \rfloor + 1$.

**Example (ResNet's first layer).** A 224×224×3 image with 64 filters of 7×7, stride 2, padding 3 gives $\lfloor(224 + 6 - 7)/2\rfloor + 1 = 112$, so a 112×112×64 output. It needs $7\cdot7\cdot3\cdot64 + 64 = 9{,}472$ parameters. A fully connected layer producing the same output from the same input would need $150{,}528 \times 802{,}816 \approx 1.2\times10^{11}$ weights, about 13 million times more. Weight sharing is why CNNs made image learning practical.

## Building blocks

- **Convolution + activation** (usually ReLU): detects local patterns.
- **Stride or pooling**: downsamples, so later layers see a wider area (larger *receptive field*) at lower cost.
- **Normalisation** (BatchNorm) and **residual connections**: make deep stacks trainable.
- **Head**: global pooling plus a small classifier, or task-specific heads for boxes, masks or depth.

## Architectures worth knowing

| Network | Year | Contribution |
|---|---|---|
| LeNet-5 | 1998 | Convolution + pooling + backprop, digits |
| AlexNet | 2012 | GPU training, ReLU, dropout; won ImageNet by a wide margin |
| VGG | 2014 | Deep stacks of 3×3 filters |
| ResNet | 2015 | Residual connections; 50–152 layers; still a standard robot-vision encoder |
| EfficientNet, MobileNet | 2017–19 | Accuracy per FLOP for phones and robots |
| ConvNeXt | 2022 | Modernised CNN matching vision transformers at similar scale |

## CNNs and vision transformers in 2026

Vision transformers (ViT, 2020) treat image patches as tokens. With enough pretraining data they match or beat CNNs, and they underlie most large foundation models (CLIP, DINOv2, the vision encoders inside VLAs). CNNs keep three advantages that matter on robots: efficiency on embedded accelerators, good performance from modest data thanks to their built-in assumptions, and predictable latency. Real-time detectors (the YOLO family, YOLO26 being current in late 2026), many depth and segmentation networks, and the ResNet encoders in policies such as [[Diffusion_Policy]] are convolutional. See [[CNN_in_Robotics]].

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Convolutional_Neural_Networks]])
```
