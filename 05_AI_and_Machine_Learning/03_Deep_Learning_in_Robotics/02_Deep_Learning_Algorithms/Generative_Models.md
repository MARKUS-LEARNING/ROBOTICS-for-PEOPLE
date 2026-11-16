---
title: Generative Models
description: "Models that learn the distribution of their data so they can produce new samples: VAEs, GANs, autoregressive transformers, diffusion and flow matching. In robotics they generate actions, predict future video, synthesise training data and power world models."
tags:
  - generative-models
  - diffusion-models
  - deep-learning
  - machine-learning
  - robot-learning
type: concept
level: intermediate
subfield:
  - robot-learning
status: keystone
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Deep_Learning]]"
  - "[[Probability_and_Statistics_for_Robotics]]"
related:
  - "[[Diffusion_Policy]]"
  - "[[World_Models]]"
  - "[[Vision-Language-Action_Models]]"
  - "[[Autoencoders]]"
  - "[[Neural_Networks]]"
key_takeaways:
  - "A generative model learns p(x) (or p(x | condition)) and can sample new x: images, video, text, or robot actions."
  - "Diffusion and flow matching generate by gradually turning noise into data; they handle multimodal distributions, which is why they now dominate robot action generation."
  - "In robotics, generative models produce actions (diffusion policies, VLA action heads), predict the future (world models) and create synthetic data."
sources:
  - "Kingma, D. and Welling, M. (2013). Auto-Encoding Variational Bayes. https://arxiv.org/abs/1312.6114"
  - "Goodfellow, I. et al. (2014). Generative Adversarial Networks. https://arxiv.org/abs/1406.2661"
  - "Vaswani, A. et al. (2017). Attention Is All You Need. https://arxiv.org/abs/1706.03762"
  - "Ho, J., Jain, A. and Abbeel, P. (2020). Denoising Diffusion Probabilistic Models. https://arxiv.org/abs/2006.11239"
  - "Lipman, Y. et al. (2022). Flow Matching for Generative Modeling. https://arxiv.org/abs/2210.02747"
  - "Chi, C. et al. (2023). Diffusion Policy: Visuomotor Policy Learning via Action Diffusion. https://arxiv.org/abs/2303.04137"
  - "NVIDIA (2025). Cosmos World Foundation Model Platform for Physical AI. https://arxiv.org/abs/2501.03575"
---

# Generative Models

A **generative model** learns what its training data looks like well enough to produce new examples of it. A *discriminative* model answers "is this a cup?"; a generative model can draw a new cup, predict what the next second of video will show, or propose the next motion of a robot arm. Formally it learns a probability distribution $p(x)$, or a conditional one $p(x \mid c)$ given some context $c$, and can **sample** from it.

> **Etymology.** *Generative* comes from Latin *generare*, "to beget, produce", from *genus*, "birth, kind". In linguistics, Noam Chomsky's *generative grammar* (1957) described rules that produce every valid sentence of a language; statistics later used "generative model" for a model of how the data itself was produced, as opposed to one that only separates classes.

## Why robotics cares

| Use | What is generated | Notes |
|---|---|---|
| **Actions** | The next chunk of robot motion, given images and an instruction | [[Diffusion_Policy]]; the action heads of [[Vision-Language-Action_Models]] |
| **The future** | Video and states that follow an action | [[World_Models]], used for planning, evaluation and training |
| **Data** | Synthetic images, scenes, objects or whole demonstrations | Augmenting scarce robot data; generating simulation assets |
| **Goals** | An image of what success should look like | Subgoal images steer some 2026 policies |

Robot actions are a natural fit: the same task can be done many ways, so the right output is a *distribution* over good actions, not one average action.

## The main families

| Family | Idea | Strength | Weakness |
|---|---|---|---|
| **VAE** (Kingma and Welling 2013) | Encode data to a small latent code, decode back; train with a likelihood bound | Stable, gives a latent space | Blurry samples |
| **GAN** (Goodfellow et al. 2014) | A generator tries to fool a discriminator | Sharp samples, fast sampling | Unstable training, mode collapse |
| **Autoregressive** (transformers; Vaswani et al. 2017) | Predict one token at a time, conditioned on the previous ones | Scales extremely well; language models | Slow for long outputs; continuous data must be tokenised |
| **Diffusion** (Ho et al. 2020) | Learn to remove noise step by step | High quality, captures every mode, stable | Many steps to sample (mitigated by distillation) |
| **Flow matching** (Lipman et al. 2022) | Learn a velocity field that carries noise to data along straight paths | Fewer sampling steps, simple training | Newer; same family as diffusion |

## Diffusion and flow matching, from first principles

Take a data point $x_0$ (an image, or a chunk of robot actions). **Forward process**: add Gaussian noise over many steps until nothing but noise is left:

$$x_t = \sqrt{\bar\alpha_t}\, x_0 + \sqrt{1-\bar\alpha_t}\, \epsilon, \qquad \epsilon \sim \mathcal{N}(0, I)$$

**Learned reverse process**: a network $\epsilon_\theta(x_t, t, c)$ is trained to predict the noise that was added,

$$\mathcal{L} = \mathbb{E}_{x_0, \epsilon, t}\big\lVert \epsilon - \epsilon_\theta(x_t, t, c) \big\rVert^2$$

and generation runs the process backwards: start from pure noise and repeatedly subtract the predicted noise until a clean sample remains. Because each step only has to make the sample *a little* more plausible, the model can represent distributions with many separate modes.

**Flow matching** replaces the noise-prediction target with a velocity: along a straight path $x_t = (1-t)\,x_0 + t\,\epsilon$, the network learns $v_\theta(x_t, t) \approx \epsilon - x_0$. Sampling integrates this velocity from noise to data, typically in about 10 steps. Physical Intelligence's π0 family and NVIDIA's GR00T action heads generate robot actions this way.

## Worked example: why averaging fails

Two demonstrations reach around an obstacle: one to the left ($a = -1$), one to the right ($a = +1$). A regression model trained with mean-squared error predicts their average, $a = 0$: straight into the obstacle. A generative model learns the two-peaked distribution and samples either $-1$ or $+1$, both of which work. This single picture explains much of the shift from regression to generative policies in robot learning.

## State of the art (as of November 2026)

- **Actions.** Flow-matching and diffusion action heads are the default in generalist robot policies (π0.x, GR00T N1.x); see [[Vision-Language-Action_Models]].
- **Video as a world model.** Large video diffusion models trained on physical scenes, such as NVIDIA's Cosmos family, are used to generate training data and to predict outcomes. "World action models" generate video and actions together (DreamZero, GR00T 2); see [[World_Models]].
- **Speed.** Distillation and consistency methods cut sampling to a few steps, which matters for robots that must act at 10–50 Hz.

## Reading list

- Ho, Jain and Abbeel 2020, *Denoising Diffusion Probabilistic Models*.
- Lipman et al. 2022, *Flow Matching for Generative Modeling*.
- Chi et al. 2023, *Diffusion Policy*: diffusion applied to robot actions.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Generative_Models]])
```
