---
title: Activation Functions (ReLU, Sigmoid, Tanh)
description: Activation Functions are mathematical functions used in neural networks to introduce non-linearity, enabling models to learn complex patterns and make decisions, with ReLU, Sigmoid, and Tanh being among the most commonly used.
tags:
  - robotics
  - activation-functions
  - neural-networks
  - machine-learning
  - engineering
  - glossary-term
  - manipulator-arm
  - mobile-robot
layout: default
category: robotics
author: Jordan_Smith_and_le_Chat
date: 2025-05-02
permalink: /activation_functions/
related:
  - "[[Neural_Networks_Basics]]"
  - "[[Neural_Networks_in_Control]]"
  - "[[Neural_Networks_and_Backpropagation]]"
  - "[[Machine_Learning]]"
  - "[[Deep_Learning]]"
  - "[[Robot_Control]]"
  - "[[Model_Training]]"
---

# Activation Functions (ReLU, Sigmoid, Tanh)

**Activation Functions** are mathematical functions used in neural networks to introduce non-linearity, enabling models to learn complex patterns and make decisions. They are fundamental components of neural networks, determining the output of a node given an input or set of inputs. Among the most commonly used activation functions are ReLU (Rectified Linear Unit), Sigmoid, and Tanh, each with unique properties and applications in robotics and machine learning.

---

## Key Concepts

### ReLU (Rectified Linear Unit)

ReLU is an activation function defined as $f(x) = \max(0, x)$. It outputs the input directly if it is positive, otherwise, it outputs zero. ReLU is widely used in deep learning due to its simplicity and effectiveness in mitigating the vanishing gradient problem.

### Sigmoid

The sigmoid function is defined as $f(x) = \frac{1}{1 + e^{-x}}$. It outputs values between 0 and 1, making it suitable for models where the output needs to be interpreted as a probability or a binary classification.

### Tanh (Hyperbolic Tangent)

The tanh function is defined as $f(x) = \frac{e^x - e^{-x}}{e^x + e^{-x}}$. It outputs values between -1 and 1, providing a zero-centered output that can be beneficial for certain types of models and data.

### Non-linearity

Non-linearity is a crucial property introduced by activation functions, allowing neural networks to model complex relationships and patterns in the data. Without non-linearity, neural networks would be limited to representing linear functions.

---

## Mathematical Formulation

### ReLU Function

The ReLU function is defined as:

$$
f(x) = \max(0, x)
$$

### Sigmoid Function

The sigmoid function is defined as:

$$
f(x) = \frac{1}{1 + e^{-x}}
$$

### Tanh Function

The tanh function is defined as:

$$
f(x) = \frac{e^x - e^{-x}}{e^x + e^{-x}}
$$

### Example: Neural Network Training

Consider a robotic system using a neural network for decision-making. The network uses ReLU activation functions in its hidden layers to introduce non-linearity, enabling the model to learn complex patterns in the data. The output layer uses a sigmoid activation function to produce a probability output, facilitating tasks such as binary classification and decision-making.

---

## Applications in Robotics

- **Decision Making**: Activation functions are used in neural networks to enable robots to make decisions based on complex patterns and relationships in the data.
- **Control Systems**: Enables the design of control algorithms that regulate the behavior of robotic systems, introducing non-linearity to model and adapt to dynamic environments.
- **Object Recognition**: Activation functions are used in models for recognizing and classifying objects in the environment, facilitating tasks such as grasping and manipulation.
- **Model Training**: Activation functions are fundamental in training neural networks, enabling them to learn and adapt to new tasks and environments.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #deep-learning OR #AI WHERE contains(file.outlinks, [[Activation_Functions_(ReLU,_Sigmoid,_Tanh)]])
