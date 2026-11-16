---
title: Neural Networks
description: "A neural network is a function built from many simple units, each computing a weighted sum followed by a nonlinearity, arranged in layers and trained from data. Covers the artificial neuron, layers and what they compute, a hand-built network that solves XOR, why nonlinearity matters, training in outline, and where neural networks sit on a robot in 2026."
aliases:
  - Artificial Neural Networks
  - Neural Networks Basics
  - ANN
tags:
  - neural-networks
  - deep-learning
  - machine-learning
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
  - "[[Linear_Algebra_for_Robotics]]"
related:
  - "[[Neural_Networks_and_Backpropagation]]"
  - "[[Activation_Functions_(ReLU,_Sigmoid,_Tanh)]]"
  - "[[Deep_Learning]]"
  - "[[Neural_Networks_in_Control]]"
  - "[[Transformers_and_Attention]]"
key_takeaways:
  - "A neuron computes a weighted sum of its inputs plus a bias, then applies a nonlinear activation; a layer is many neurons in parallel; a network is layers in sequence."
  - "Without nonlinear activations, any stack of layers collapses into one linear map; with them, networks can approximate essentially any function."
  - "Training adjusts the weights by gradient descent on a loss, with gradients computed by backpropagation."
sources:
  - "McCulloch, W. S. and Pitts, W. (1943). A logical calculus of the ideas immanent in nervous activity. https://link.springer.com/article/10.1007/BF02478259"
  - "Rosenblatt, F. (1958). The perceptron: a probabilistic model for information storage and organization in the brain. https://psycnet.apa.org/record/1959-09865-001"
  - "Cybenko, G. (1989). Approximation by superpositions of a sigmoidal function. https://link.springer.com/article/10.1007/BF02551274"
  - "Rumelhart, D. E., Hinton, G. E. and Williams, R. J. (1986). Learning representations by back-propagating errors. https://www.nature.com/articles/323533a0"
  - "Goodfellow, I., Bengio, Y. and Courville, A. (2016). Deep Learning. https://www.deeplearningbook.org/"
---

# Neural Networks

A **neural network** is a mathematical function assembled from many small, identical pieces called neurons. Each piece is simple; arranged in layers and tuned on data, together they can recognise objects in camera images, estimate depth, or map a robot's sensor readings to motor commands.

> **Etymology.** Greek *neuron*, "sinew, nerve". Warren McCulloch and Walter Pitts modelled a neuron as a logic unit in 1943, and Frank Rosenblatt built the trainable *perceptron* in 1958. Artificial neurons are inspired by biological ones but are far simpler; the analogy is historical, not literal.

## The artificial neuron

A neuron takes inputs $x_1, \dots, x_n$, multiplies each by a weight, adds a bias, and passes the sum through an **activation function** $\sigma$:

$$y = \sigma\Big(\sum_{i=1}^{n} w_i x_i + b\Big) = \sigma(\mathbf w^\top \mathbf x + b)$$

The weights say how much each input matters; the bias shifts the threshold; the activation adds the nonlinearity (see [[Activation_Functions_(ReLU,_Sigmoid,_Tanh)]]).

## Layers and networks

A **layer** applies many neurons to the same inputs at once, which is one matrix multiplication: $\mathbf h = \sigma(W\mathbf x + \mathbf b)$. A **network** chains layers: input layer → hidden layers → output layer. Each hidden layer re-describes the data in terms that make the next step easier.

**Why the nonlinearity is essential.** Without $\sigma$, two layers give $W_2(W_1\mathbf x) = (W_2W_1)\mathbf x$: still one linear map, however many layers you stack. With it, a network with even one hidden layer can approximate any continuous function on a bounded region to any accuracy, given enough neurons (Cybenko, 1989).

## Worked example: a network that computes XOR

XOR (output 1 when exactly one input is 1) cannot be computed by a single neuron, because no straight line separates its 1s from its 0s. Two hidden ReLU neurons, $\text{ReLU}(z) = \max(0, z)$, suffice:

$$h_1 = \text{ReLU}(x_1 + x_2), \qquad h_2 = \text{ReLU}(x_1 + x_2 - 1), \qquad y = h_1 - 2h_2$$

| $x_1, x_2$ | $h_1$ | $h_2$ | $y$ |
|---|---|---|---|
| 0, 0 | 0 | 0 | 0 |
| 1, 0 | 1 | 0 | 1 |
| 0, 1 | 1 | 0 | 1 |
| 1, 1 | 2 | 1 | 0 |

The hidden layer re-described the input as "how many inputs are on" ($h_1$) and "are both on" ($h_2$). In that new description the problem is linear. Training discovers such re-descriptions automatically, rather than by hand.

## Training in outline

1. Pick a **loss**, for example the squared difference between the output and the target.
2. Compute the **gradient** of the loss with respect to every weight using **backpropagation** (see [[Neural_Networks_and_Backpropagation]]).
3. Move every weight a small step downhill: $w \leftarrow w - \eta\,\partial L/\partial w$.
4. Repeat over many mini-batches of data.

## Kinds of networks

| Network | Input it suits | See |
|---|---|---|
| MLP (fully connected) | Vectors: joint states, IMU | this note |
| Convolutional | Images, grids | [[Convolutional_Neural_Networks]] |
| Recurrent | Time series | [[RNN_and_LSTM]] |
| Transformer | Sequences and sets of tokens | [[Transformers_and_Attention]] |
| Autoencoder | Compression, representation | [[Autoencoders]] |

## Neural networks on a robot (2026)

- **Perception**: detectors, segmenters and depth estimators in camera pipelines.
- **Policies**: small MLPs trained with RL run locomotion at hundreds of hertz; large transformer-based VLAs map images and language to actions (see [[Deep_Learning]]).
- **Models**: learned dynamics and residual models correct physics models for control (see [[Neural_Networks_in_Control]]).
- **Estimation**: learned odometry and contact detection alongside classical filters.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Neural_Networks]])
```
