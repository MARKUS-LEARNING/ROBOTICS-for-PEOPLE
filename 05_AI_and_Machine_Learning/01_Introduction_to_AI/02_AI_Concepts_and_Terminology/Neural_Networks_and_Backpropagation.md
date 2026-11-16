---
title: Neural Networks and Backpropagation
description: "Backpropagation is the algorithm that trains neural networks: it applies the chain rule backwards through the network to get the gradient of the loss with respect to every weight in one pass. Covers the idea, the equations for a layer, a fully computed two-weight example with one update step, automatic differentiation in PyTorch and JAX, and the practical problems (vanishing gradients, initialisation, normalisation)."
aliases:
  - Backpropagation
  - Backprop
tags:
  - neural-networks
  - deep-learning
  - optimization
  - machine-learning
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
  - "[[Calculus_for_Robotics]]"
related:
  - "[[Optimization_and_Loss_Functions]]"
  - "[[Activation_Functions_(ReLU,_Sigmoid,_Tanh)]]"
  - "[[Deep_Learning]]"
key_takeaways:
  - "Backpropagation is the chain rule applied from the loss back to the inputs, reusing each layer's intermediate results; it costs about as much as two forward passes."
  - "Modern frameworks (PyTorch, JAX) do it automatically (reverse-mode automatic differentiation); you write the forward computation, they produce the gradients."
  - "Deep networks train well only with care: sensible initialisation, non-saturating activations, residual connections, normalisation and adaptive optimisers such as Adam."
sources:
  - "Rumelhart, D. E., Hinton, G. E. and Williams, R. J. (1986). Learning representations by back-propagating errors. https://www.nature.com/articles/323533a0"
  - "Glorot, X., Bordes, A. and Bengio, Y. (2011). Deep Sparse Rectifier Neural Networks. https://proceedings.mlr.press/v15/glorot11a.html"
  - "Kingma, D. P. and Ba, J. (2014). Adam: A Method for Stochastic Optimization. https://arxiv.org/abs/1412.6980"
  - "Baydin, A. G. et al. (2015). Automatic differentiation in machine learning: a survey. https://arxiv.org/abs/1502.05767"
  - "Goodfellow, I., Bengio, Y. and Courville, A. (2016). Deep Learning, chapter 6.5: Back-Propagation. https://www.deeplearningbook.org/"
---

# Neural Networks and Backpropagation

Training a [[Neural_Networks|neural network]] means adjusting millions or billions of weights so that a loss goes down. To know which way to move each weight, you need the **gradient**: how much the loss changes when that weight changes. **Backpropagation** computes all of those gradients in one backward sweep through the network.

> **Etymology.** *Back-propagation of errors*: the error signal is propagated backwards from the output. The method has several inventors: Seppo Linnainmaa described reverse-mode differentiation in 1970 and Paul Werbos applied it to networks in 1974. Rumelhart, Hinton and Williams's 1986 *Nature* paper made it the standard way to train networks.

## The idea: the chain rule, run backwards

A network is a chain of functions: input → layer 1 → layer 2 → … → output → loss. The chain rule says the sensitivity of the loss to an early quantity equals the product of the sensitivities along the chain. Going **backwards** means each layer's gradient is computed once and reused by every layer before it. The whole backward pass therefore costs about the same as two forward passes, however many weights there are. Computing each weight's effect separately would cost a full forward pass *per weight*.

For a layer $\mathbf h = \sigma(\mathbf z)$ with $\mathbf z = W\mathbf x + \mathbf b$, given the upstream gradient $\boldsymbol\delta_h = \partial L/\partial \mathbf h$:

$$\boldsymbol\delta_z = \boldsymbol\delta_h \odot \sigma'(\mathbf z), \qquad \frac{\partial L}{\partial W} = \boldsymbol\delta_z\,\mathbf x^\top, \qquad \frac{\partial L}{\partial \mathbf b} = \boldsymbol\delta_z, \qquad \frac{\partial L}{\partial \mathbf x} = W^\top \boldsymbol\delta_z$$

The last quantity is passed back to the previous layer, and so on.

## Worked example: one training step by hand

A tiny network: $h = \text{ReLU}(w_1 x)$, $y = w_2 h$, loss $L = \tfrac12 (y - t)^2$. Take $x = 1$, target $t = 2$, and start from $w_1 = 0.5$, $w_2 = 2$.

**Forward pass.** $h = \text{ReLU}(0.5) = 0.5$, $y = 2 \times 0.5 = 1.0$, $L = \tfrac12(1-2)^2 = 0.5$.

**Backward pass.**

| Quantity | Chain rule | Value |
|---|---|---|
| $\partial L/\partial y$ | $y - t$ | −1 |
| $\partial L/\partial w_2$ | $\partial L/\partial y \cdot h$ | −0.5 |
| $\partial L/\partial h$ | $\partial L/\partial y \cdot w_2$ | −2 |
| $\partial L/\partial w_1$ | $\partial L/\partial h \cdot \text{ReLU}'(0.5) \cdot x$ | −2 |

**Update** with learning rate $\eta = 0.1$: $w_2 \leftarrow 2 - 0.1(-0.5) = 2.05$ and $w_1 \leftarrow 0.5 - 0.1(-2) = 0.7$.

**Check.** Now $h = 0.7$, $y = 2.05 \times 0.7 = 1.435$, and $L = \tfrac12(1.435 - 2)^2 = 0.160$. One step cut the loss from 0.5 to 0.16. Notice that $w_1$ moved more than $w_2$: the gradient automatically sends more of the blame to the weight whose change helps most.

## You rarely write it yourself

Frameworks implement backpropagation as **reverse-mode automatic differentiation**. You write the forward computation and they record it and produce exact gradients:

```python
import torch
w1 = torch.tensor(0.5, requires_grad=True); w2 = torch.tensor(2.0, requires_grad=True)
y = w2 * torch.relu(w1 * 1.0)
loss = 0.5 * (y - 2.0) ** 2
loss.backward()
print(w1.grad, w2.grad)   # tensor(-2.) tensor(-0.5)
```

The same machinery differentiates through anything written in the framework, including physics: **differentiable simulators** (MJX, Warp-based engines) let gradients flow through rigid-body dynamics into a controller.

## Making deep networks trainable

| Problem | What happens | Remedy |
|---|---|---|
| **Vanishing gradients** | Repeated multiplication by small derivatives (sigmoid's is at most 0.25) shrinks gradients toward zero in early layers | ReLU-family activations, residual connections |
| **Exploding gradients** | Products grow without bound, especially in recurrent nets | Gradient clipping, careful initialisation, LSTMs or transformers |
| **Bad scale** | Activations drift in size layer to layer | Normalisation (LayerNorm, BatchNorm), Xavier or He initialisation |
| **Slow or unstable descent** | One learning rate fits no weight well | Adaptive optimisers (Adam, AdamW), learning-rate warm-up and decay |

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Neural_Networks_and_Backpropagation]])
```
