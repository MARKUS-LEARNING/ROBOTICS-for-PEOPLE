---
title: Transfer Learning
description: "Reusing what a model learned on one task or dataset to learn another faster: pretrained vision encoders, fine-tuning and parameter-efficient adaptation (LoRA), cross-embodiment transfer between robots, and sim-to-real transfer."
tags:
  - transfer-learning
  - deep-learning
  - robot-learning
  - foundation-models
  - sim-to-real
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
  - "[[Deep_Learning]]"
related:
  - "[[Vision-Language-Action_Models]]"
  - "[[Imitation_Learning]]"
  - "[[Reinforcement_Learning_for_Robots]]"
  - "[[Convolutional_Neural_Networks]]"
key_takeaways:
  - "Transfer learning starts from a model trained on lots of data and adapts it to a task with little data."
  - "In robotics it shows up three ways: pretrained perception, cross-embodiment transfer between robots, and sim-to-real transfer."
  - "Parameter-efficient fine-tuning (LoRA) lets a multi-billion-parameter robot model be adapted on one GPU."
sources:
  - "Yosinski, J. et al. (2014). How transferable are features in deep neural networks? https://arxiv.org/abs/1411.1792"
  - "Radford, A. et al. (2021). Learning Transferable Visual Models From Natural Language Supervision. https://arxiv.org/abs/2103.00020"
  - "Nair, S. et al. (2022). R3M: A Universal Visual Representation for Robot Manipulation. https://arxiv.org/abs/2203.12601"
  - "Hu, E. et al. (2021). LoRA: Low-Rank Adaptation of Large Language Models. https://arxiv.org/abs/2106.09685"
  - "Open X-Embodiment Collaboration (2023). Open X-Embodiment: Robotic Learning Datasets and RT-X Models. https://arxiv.org/abs/2310.08864"
---

# Transfer Learning

**Transfer learning** means starting from a model that already learned something useful and adapting it to a new problem, instead of learning from scratch. A network trained on millions of internet images already knows edges, textures and objects; a robot that must recognise a few hundred parts in a factory can reuse that knowledge and learn from far fewer examples.

For robots, where every training example is expensive to collect, transfer learning is not an optimisation. It is how almost every learned robot system is built.

> **Etymology.** *Transfer* is from Latin *transferre*, "to carry across" (*trans* + *ferre*, "to bear"). Educational psychologists studied *transfer of training* from the early 1900s (Thorndike and Woodworth, 1901): does learning Latin make you better at geometry? Machine learning revived the question in the 1990s, asking when knowledge from one task helps another.

## Why it works

Early layers of deep networks learn general features (edges, colours, simple shapes); later layers become task-specific. Yosinski et al. (2014) measured this directly: copying early layers between tasks helped almost always, while copying later layers helped only between similar tasks. Large models trained on broad data push the general part much further.

## The three kinds of transfer in robotics

| Kind | From → to | Example |
|---|---|---|
| **Perception transfer** | Web images and text → robot vision | Pretrained encoders such as CLIP (Radford et al. 2021) or R3M (Nair et al. 2022, trained on human video) as a policy's visual front end |
| **Cross-embodiment transfer** | Many robots → a new robot | Open X-Embodiment's RT-X models: training on data from many robots improved performance on each; generalist [[Vision-Language-Action_Models]] fine-tuned to a new arm |
| **Sim-to-real transfer** | Simulation → hardware | Locomotion policies trained in simulation; see [[Reinforcement_Learning_for_Robots]] |

## Ways to adapt a pretrained model

| Method | What is trained | When to use |
|---|---|---|
| **Feature extraction** | Only a new head; the pretrained model is frozen | Very little data |
| **Full fine-tuning** | All weights | Plenty of data and compute |
| **Parameter-efficient fine-tuning** | Small added modules (LoRA, adapters) | Large models, limited compute |
| **Co-training** | New task data mixed with the original data | Keeping old abilities while adding new ones |

**LoRA** (Hu et al. 2021) freezes a weight matrix $W \in \mathbb{R}^{d \times k}$ and learns a low-rank update $\Delta W = BA$ with $B \in \mathbb{R}^{d \times r}$, $A \in \mathbb{R}^{r \times k}$ and $r \ll \min(d, k)$.

## Worked example: LoRA's saving

One 4096 × 4096 projection matrix has about 16.8 million weights. A LoRA update with rank $r = 16$ trains $4096 \times 16 + 16 \times 4096 = 131{,}072$ weights: under 1% as many. Across a multi-billion-parameter VLA, LoRA fine-tuning can fit on a single GPU, which is why it is the common way to adapt open robot foundation models to a new robot or task.

## Risks

- **Negative transfer**: knowledge from a dissimilar source can hurt.
- **Catastrophic forgetting**: fine-tuning on a narrow task can erase general abilities; co-training on the original data reduces it.
- **Embodiment mismatch**: a policy pretrained on parallel grippers transfers poorly to a five-fingered hand without enough new data.

## State of the art (as of November 2026)

Generalist robot models are explicitly designed for transfer: pretrained once on many robots and tasks, then post-trained per robot. Gemini Robotics On-Device 2 is described by Google DeepMind as adapting to new embodiments with a few hours of data, and NVIDIA reports fine-tuning GR00T 2 to new tasks from five demonstrations (vendor figures).

## Reading list

- Yosinski et al. 2014, *How transferable are features in deep neural networks?*
- Hu et al. 2021, *LoRA*.
- Open X-Embodiment Collaboration 2023: cross-embodiment transfer.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Transfer_Learning]])
```
