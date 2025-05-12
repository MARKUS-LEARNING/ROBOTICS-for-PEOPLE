---
title: RNN and LSTM
description: Recurrent Neural Networks (RNNs) and Long Short-Term Memory networks (LSTMs) are types of neural networks designed for processing sequential data, widely used in robotics for tasks such as time series prediction and motion planning.
tags:
  - robotics
  - recurrent-neural-networks
  - long-short-term-memory
  - machine-learning
  - neural-networks
  - engineering
  - glossary-term
  - manipulator-arm
  - mobile-robot
layout: default
category: robotics
author: Jordan_Smith_and_le_Chat
date: 2025-05-02
permalink: /rnn_and_lstm/
related:
  - "[[Neural_Networks]]"
  - "[[Machine_Learning]]"
  - "[[Deep_Learning]]"
  - "[[Robot_Control]]"
  - "[[Time_Series_Prediction]]"
---

# RNN and LSTM

**Recurrent Neural Networks (RNNs)** and **Long Short-Term Memory networks (LSTMs)** are types of neural networks designed for processing sequential data. They are widely used in robotics for tasks such as time series prediction, motion planning, and natural language processing. RNNs and LSTMs are particularly effective for tasks that involve temporal dependencies, as they can capture patterns and relationships in sequential data.

---

## Key Concepts

### Recurrent Neural Networks (RNNs)

RNNs are neural networks that include loops in their architecture, allowing information to persist and be used in subsequent steps. This enables them to process sequential data, where the output at each step depends on the previous computations.

### Long Short-Term Memory Networks (LSTMs)

LSTMs are a type of RNN that include a memory cell, which can maintain information over long periods of time. This enables them to capture long-term dependencies in sequential data, improving their ability to learn and predict patterns.

### Hidden State

The hidden state in an RNN or LSTM is the internal representation of the network at each time step, capturing the information from the previous steps and using it to influence the current computation.

### Backpropagation Through Time (BPTT)

Backpropagation Through Time (BPTT) is the algorithm used to train RNNs and LSTMs, involving the unfolding of the network in time and the computation of the gradient of the loss function with respect to the weights.

---

## Mathematical Formulation

### RNN Hidden State Update

The hidden state update in an RNN can be represented as:

$$
h_t = \sigma(W_{xh} \cdot x_t + W_{hh} \cdot h_{t-1} + b)
$$

where:
- $h_t$ is the hidden state at time $t$.
- $x_t$ is the input at time $t$.
- $W_{xh}$ and $W_{hh}$ are the weight matrices.
- $b$ is the bias term.
- $\sigma$ is the activation function.

### LSTM Cell State Update

The cell state update in an LSTM can be represented as:

$$
C_t = f_t \cdot C_{t-1} + i_t \cdot \tilde{C}_t
$$

where:
- $C_t$ is the cell state at time $t$.
- $f_t$ is the forget gate.
- $i_t$ is the input gate.
- $\tilde{C}_t$ is the candidate cell state.

### Example: Motion Planning

Consider a robotic system using an LSTM for motion planning. The LSTM processes sequential data from the robot's sensors, capturing the temporal dependencies in the data. The hidden state of the LSTM represents the robot's internal state, capturing the information from the previous steps and using it to plan the current motion. This enables the robot to perform tasks such as navigation and manipulation, adapting to changing conditions and performing tasks effectively.

---

## Applications in Robotics

- **Time Series Prediction**: RNNs and LSTMs are used to predict future values in time series data, enabling tasks such as trajectory planning and control.
- **Motion Planning**: Enables robots to plan and execute motions, capturing the temporal dependencies in the data and adapting to changing conditions.
- **Natural Language Processing**: RNNs and LSTMs are used to process and generate natural language, facilitating tasks such as instruction interpretation and communication.
- **Control Systems**: RNNs and LSTMs are used to design control algorithms that regulate the behavior of robotic systems, enabling precise and adaptive control.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #robotics OR #machine-learning WHERE contains(file.outlinks, [[RNN_and_LSTM]])
