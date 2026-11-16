---
title: RNN and LSTM
description: "Recurrent neural networks carry a hidden state from one time step to the next, which gives them memory for sequences such as sensor streams. Covers the simple RNN, why its gradients vanish (computed), how LSTM and GRU gates fix it, where recurrent policies still matter in robotics (memory of terrain, contact and history), and how transformers and state-space models compare in 2026."
aliases:
  - RNN
  - LSTM
  - GRU
  - Recurrent Neural Networks
tags:
  - neural-networks
  - deep-learning
  - locomotion
  - robot-learning
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
  - "[[Neural_Networks_and_Backpropagation]]"
related:
  - "[[Transformers_and_Attention]]"
  - "[[Reinforcement_Learning_for_Robots]]"
  - "[[Legged_Robots]]"
  - "[[Deep_Learning]]"
key_takeaways:
  - "An RNN updates a hidden state h_t = f(h_{t-1}, x_t) every step, so it can remember what it has seen; it is trained by backpropagation through time."
  - "Simple RNNs forget quickly because gradients shrink geometrically with time; LSTM and GRU gates let the network decide what to keep, preserving information over hundreds of steps."
  - "Recurrent policies (often GRU or LSTM) remain common in legged locomotion, where the robot must infer terrain, friction and its own state from recent history; transformers and state-space models are the alternatives."
sources:
  - "Hochreiter, S. and Schmidhuber, J. (1997). Long Short-Term Memory. https://direct.mit.edu/neco/article/9/8/1735/6109/Long-Short-Term-Memory"
  - "Cho, K. et al. (2014). Learning Phrase Representations using RNN Encoder-Decoder for Statistical Machine Translation. https://arxiv.org/abs/1406.1078"
  - "Miki, T. et al. (2022). Learning robust perceptive locomotion for quadrupedal robots in the wild. https://arxiv.org/abs/2201.08117"
  - "Radosavovic, I. et al. (2023). Real-World Humanoid Locomotion with Reinforcement Learning. https://arxiv.org/abs/2303.03381"
  - "Gu, A. and Dao, T. (2023). Mamba: Linear-Time Sequence Modeling with Selective State Spaces. https://arxiv.org/abs/2312.00752"
---

# RNN and LSTM

A **recurrent neural network (RNN)** processes a sequence one step at a time and carries a **hidden state** forward, a running summary of everything it has seen. A robot needs exactly this when one sensor reading is not enough: a single IMU sample cannot say whether the ground is slippery, but the last second of joint and IMU readings can.

> **Etymology.** *Recurrent* is from Latin *recurrere*, "to run back": the network's output runs back in as its own input at the next step. *LSTM* stands for *long short-term memory*, a short-term memory (the hidden state) that lasts a long time. Sepp Hochreiter and Jürgen Schmidhuber introduced it in 1997.

## The simple RNN

$$\mathbf h_t = \tanh(W\mathbf h_{t-1} + U\mathbf x_t + \mathbf b), \qquad \mathbf y_t = V\mathbf h_t$$

The same weights $W, U, V$ are reused at every step. Training "unrolls" the network over time and applies backpropagation through the unrolled chain (**backpropagation through time**).

## Why simple RNNs forget: a computed example

The influence of an input $k$ steps back is multiplied by roughly the recurrent weight (times the activation's derivative) once per step. With an effective factor of 0.5 per step, an input 10 steps back contributes $0.5^{10} \approx 0.001$ of its original effect, so the network effectively cannot learn from it. At 100 Hz, 10 steps is just 0.1 s of memory. Factors above 1 cause the opposite problem: exploding gradients.

## LSTM and GRU: learned memory

The **LSTM** adds a separate *cell state* $\mathbf c_t$ and three gates (sigmoids between 0 and 1) that decide what to forget, what to write and what to output:

$$\mathbf c_t = \mathbf f_t \odot \mathbf c_{t-1} + \mathbf i_t \odot \tilde{\mathbf c}_t, \qquad \mathbf h_t = \mathbf o_t \odot \tanh(\mathbf c_t)$$

Because the cell state is updated by addition, scaled by the forget gate $\mathbf f_t$, the network can hold information as long as it chooses. With a forget gate of 0.95, an input still carries $0.95^{10} = 0.60$ of its weight after 10 steps and $0.95^{50} = 0.077$ after 50. At 0.99 it keeps $0.99^{100} = 0.37$ after 100 steps, a full second at 100 Hz. The network *learns* these gate values for each situation.

The **GRU** (Cho et al., 2014) merges the gates into two and drops the separate cell state. It is slightly cheaper and often performs as well, and it is a frequent choice in robot policies.

## Where recurrence matters in robotics

| Use | Why memory is needed |
|---|---|
| **Legged locomotion** | Terrain, friction and payload are not directly observed; a GRU or LSTM infers them from recent proprioception (e.g. ANYmal's perceptive locomotion, 2022) |
| **State estimation** | Learned odometry and contact estimation from IMU and joint histories |
| **Partial observability in RL** | Occluded objects, hidden velocities; recurrent policies (PPO + LSTM) |
| **Time-series monitoring** | Fault and anomaly detection on motor currents and vibration |

Alternatives that keep a fixed **window of history** are also common: stack the last N observations into an MLP, or use a 1D convolution over time. RMA (2021) used a temporal convolution to estimate environment properties.

## RNNs, transformers and state-space models (2026)

| Model | Memory | Cost per step at run time | Training | Robotics use |
|---|---|---|---|---|
| **LSTM / GRU** | Compressed into a fixed-size state | Constant, tiny | Sequential, slower | Onboard locomotion policies at 50–1,000 Hz |
| **[[Transformers_and_Attention\|Transformer]]** | Exact access to a window of past tokens | Grows with the window | Parallel, fast | Humanoid locomotion (2023), VLAs, world models |
| **State-space models** (S4, Mamba) | Fixed-size state with structured, selective updates | Constant | Parallel | Long-horizon sequence modelling; emerging in robot policies |

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[RNN_and_LSTM]])
```
