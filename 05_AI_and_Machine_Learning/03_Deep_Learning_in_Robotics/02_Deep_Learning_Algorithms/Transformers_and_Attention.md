---
title: Transformers and Attention
description: "The transformer is the neural-network architecture behind language models, vision transformers and today's robot foundation models. Explains attention from first principles with a hand-computed example, tokens for images and actions, why transformers scale, their cost, and how robot policies (RT-2, OpenVLA, π0, GR00T) use them in 2026."
aliases:
  - Transformer
  - Attention
  - Self-Attention
  - Vision Transformer
tags:
  - transformers
  - deep-learning
  - neural-networks
  - foundation-models
  - vision-language-action
type: concept
level: intermediate
subfield:
  - robot-learning
status: keystone
author: Jordan_Smith
date: 2026-11-15
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Deep_Learning]]"
  - "[[Neural_Networks]]"
related:
  - "[[Vision-Language-Action_Models]]"
  - "[[Diffusion_Policy]]"
  - "[[World_Models]]"
  - "[[RNN_and_LSTM]]"
  - "[[Generative_Models]]"
key_takeaways:
  - "Attention lets every token look at every other token and take a weighted average of what it finds; the weights come from how well a query matches each key."
  - "Anything can be tokens: words, image patches, joint states, actions. That is why one architecture now spans language, vision and robot control."
  - "Attention costs grow with the square of the number of tokens, which is why robot models compress images into few tokens and predict chunks of actions at once."
sources:
  - "Vaswani, A. et al. (2017). Attention Is All You Need. https://arxiv.org/abs/1706.03762"
  - "Bahdanau, D., Cho, K. and Bengio, Y. (2014). Neural Machine Translation by Jointly Learning to Align and Translate. https://arxiv.org/abs/1409.0473"
  - "Dosovitskiy, A. et al. (2020). An Image is Worth 16x16 Words: Transformers for Image Recognition at Scale. https://arxiv.org/abs/2010.11929"
  - "Brohan, A. et al. (2023). RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control. https://arxiv.org/abs/2307.15818"
  - "Kim, M. J. et al. (2024). OpenVLA: An Open-Source Vision-Language-Action Model. https://arxiv.org/abs/2406.09246"
  - "Black, K. et al. (2024). π0: A Vision-Language-Action Flow Model for General Robot Control. https://arxiv.org/abs/2410.24164"
---

# Transformers and Attention

The **transformer** is a neural-network architecture built around one operation, **attention**, that lets every element of an input look at every other element and decide what matters. Introduced in 2017 for machine translation, it now underlies large language models, vision models and the [[Vision-Language-Action_Models|vision-language-action models]] that control robots.

> **Etymology.** *Attention* is from Latin *attendere*, "to stretch toward": the network stretches toward the parts of the input that are relevant. *Transformer* simply names what the network does, transforming one sequence of representations into another. The 2017 paper's title, *Attention Is All You Need*, signalled that attention could replace recurrence entirely.

## The problem attention solves

Before transformers, sequence models ([[RNN_and_LSTM|RNNs and LSTMs]]) read inputs one step at a time and squeezed everything seen so far into a fixed-size memory. Long-range connections faded, and training could not be parallelised across time. Attention, first added to RNNs by Bahdanau et al. (2014), lets the model reach back directly to any earlier element. The transformer kept only the attention.

## Attention from first principles

Each **token** (a word, an image patch, a joint reading) is a vector. From it the network computes three vectors with learned matrices:
- a **query** $q$: what am I looking for?
- a **key** $k$: what do I contain?
- a **value** $v$: what do I pass on if selected?

For one query, attention scores every key, turns the scores into weights with a softmax, and averages the values:

$$\text{Attention}(q, K, V) = \sum_i \text{softmax}_i\!\left(\frac{q \cdot k_i}{\sqrt{d}}\right) v_i$$

Dividing by $\sqrt{d}$ (the vector size) keeps the scores from growing with dimension and saturating the softmax.

### Worked example

Let $d = 2$, a query $q = (1, 0)$, and three tokens:

| Token | Key $k_i$ | Value $v_i$ | Score $q\cdot k_i/\sqrt2$ | Weight |
|---|---|---|---|---|
| A | (1, 0) | (10, 0) | 0.7071 | 0.4011 |
| B | (0, 1) | (0, 10) | 0 | 0.1978 |
| C | (1, 1) | (5, 5) | 0.7071 | 0.4011 |

The weights are $e^{0.7071} / (2e^{0.7071} + e^0) = 2.028 / 5.056 = 0.4011$ for A and C, and $1/5.056 = 0.1978$ for B. The output is

$$0.4011(10,0) + 0.1978(0,10) + 0.4011(5,5) = (6.017,\ 3.983).$$

The query "matched" A and C, so the output leans toward their values. B still contributes a little: attention is soft.

In a real transformer every token issues its own query (**self-attention**), several attention "heads" run in parallel to capture different relations, and each attention layer is followed by a small MLP. Residual connections and normalisation wrap every block, and **positional encodings** tell the model where each token sits in the sequence (or in the image).

## Everything is tokens

| Modality | How it becomes tokens |
|---|---|
| Text | Sub-word pieces from a tokenizer |
| Images | 16×16-pixel patches, each projected to a vector (Vision Transformer, 2020) |
| Proprioception | Joint positions and velocities projected to one or a few tokens |
| Actions | Discretised into bins and treated like words (RT-2, OpenVLA), or generated as continuous chunks by a diffusion or flow "action expert" (π0, GR00T) |

Because every input becomes the same kind of object, one network can take a camera image, a sentence ("put the cup in the sink") and the robot's joint state, and output motor commands. That is the core of a VLA.

## Why transformers scale, and what they cost

- **Parallel training.** All tokens are processed at once, which suits GPUs.
- **Predictable scaling.** Bigger models trained on more data improve smoothly, which justified building foundation models.
- **Quadratic cost.** Self-attention compares every token with every other: 256 tokens make 65,536 pairs, while 1,024 tokens make 1,048,576. That is 16 times the work for 4 times the tokens.

For robots that quadratic cost matters. A 224×224 image cut into 16×16 patches gives 196 tokens; three cameras and a short history quickly reach thousands. Robot models therefore:
- compress images into fewer tokens;
- cache the vision-language part and run only a small action module often;
- predict **chunks** of 10–50 future actions per inference (see [[Diffusion_Policy]]).

Efficient-attention kernels, mixture-of-experts layers and state-space models are active ways to push these limits further.

## Transformers in robotics, 2023–2026

| Model | Year | How it uses the transformer |
|---|---|---|
| RT-1 | 2022 | Transformer policy over image tokens, discretised actions |
| RT-2 | 2023 | Vision-language model fine-tuned to output action tokens; first widely cited "VLA" |
| OpenVLA | 2024 | Open 7B VLA built on a Llama 2 backbone |
| π0 / π0.5 / π0.7 | 2024–26 | VLM backbone plus a flow-matching action expert generating action chunks |
| GR00T N1.x | 2025–26 | Vision-language "System 2" with a diffusion-transformer action "System 1" for humanoids |
| Video world models | 2025–26 | Diffusion transformers predicting future video (and actions); see [[World_Models]] |

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Transformers_and_Attention]])
```
