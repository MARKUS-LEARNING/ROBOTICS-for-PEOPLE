---
title: Vision-Language-Action Models (VLAs)
description: "Robot foundation models that take camera images and a language instruction and output actions. Built on pretrained vision-language models and trained on large multi-robot datasets, they are the main route to generalist robots in 2026: π0.x, GR00T, Gemini Robotics, OpenVLA and others."
aliases:
  - VLA
  - VLAs
  - Vision-Language-Action Model
  - Foundation Models
  - Foundation_Models
  - Robot Foundation Models
tags:
  - vision-language-action
  - foundation-models
  - robot-learning
  - imitation-learning
  - manipulation
  - humanoid
type: concept
level: advanced
subfield:
  - robot-learning
  - manipulation
  - legged-humanoid
status: keystone
author: Jordan_Smith
date: 2026-11-15
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Imitation_Learning]]"
  - "[[Diffusion_Policy]]"
  - "[[Transfer_Learning]]"
related:
  - "[[World_Models]]"
  - "[[Robot_Learning_Datasets_and_Benchmarks]]"
  - "[[Teleoperation_and_Data_Collection]]"
  - "[[Embodied_AI]]"
  - "[[Humanoid_Robots]]"
key_takeaways:
  - "A VLA reuses a vision-language model's knowledge of the world and adds a way to output robot actions: discrete tokens, compressed tokens, or a diffusion / flow-matching action expert."
  - "Training is staged: web-scale VLM pretraining, robot-data pretraining across many embodiments, then task or robot fine-tuning, increasingly followed by reinforcement learning."
  - "2026 models generalise to new objects, scenes and some new tasks, but long multi-step autonomy, fine multi-finger dexterity, speed and rigorous evaluation remain open problems."
sources:
  - "Brohan, A. et al. (2022). RT-1: Robotics Transformer for Real-World Control at Scale. https://arxiv.org/abs/2212.06817"
  - "Brohan, A. et al. (2023). RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control. https://arxiv.org/abs/2307.15818"
  - "Octo Model Team (2024). Octo: An Open-Source Generalist Robot Policy. https://arxiv.org/abs/2405.12213"
  - "Kim, M. J. et al. (2024). OpenVLA: An Open-Source Vision-Language-Action Model. https://arxiv.org/abs/2406.09246"
  - "Black, K. et al. (2024). π0: A Vision-Language-Action Flow Model for General Robot Control. https://arxiv.org/abs/2410.24164"
  - "Pertsch, K. et al. (2025). FAST: Efficient Action Tokenization for Vision-Language-Action Models. https://arxiv.org/abs/2501.09747"
  - "NVIDIA (2025). GR00T N1: An Open Foundation Model for Generalist Humanoid Robots. https://arxiv.org/abs/2503.14734"
  - "Gemini Robotics Team (2025). Gemini Robotics: Bringing AI into the Physical World. https://arxiv.org/abs/2503.20020"
  - "Physical Intelligence (2026). π0.7: a Steerable Generalist Robotic Foundation Model with Emergent Capabilities. https://arxiv.org/abs/2604.15483"
  - "NVIDIA (2026). NVIDIA Isaac GR00T N1.7: Open Reasoning VLA Model for Humanoid Robots. https://huggingface.co/blog/nvidia/gr00t-n1-7"
  - "Google DeepMind (2026). Gemini Robotics 2 brings whole body intelligence to robots. https://deepmind.google/blog/gemini-robotics-2-brings-whole-body-intelligence-to-robots/"
  - "Google DeepMind (2026). Gemini Robotics 2.5 accelerates real-world general dexterity. https://deepmind.google/blog/gemini-robotics-2-5-accelerates-real-world-physical-intelligence/"
---

# Vision-Language-Action Models (VLAs)

A **vision-language-action model (VLA)** is a robot [[Policy]] built on top of a large vision-language model. It takes camera images and an instruction such as "put the dishes in the rack" and outputs robot actions. The idea is to reuse what a model trained on billions of web images and sentences already knows (what a mug is, where dishes go, what "gently" means) and connect it to a robot's motors.

VLAs are the robotics counterpart of large language models: pretrained once on broad data, then adapted to many robots and tasks. They are also called **robot foundation models** or **generalist robot policies**.

> **Etymology.** The name lists the three modalities the model connects: *vision* (Latin *visio*, "sight"), *language* (Latin *lingua*, "tongue"), and *action* (Latin *agere*, "to do"). Google DeepMind's RT-2 paper (2023) introduced the term "vision-language-action model". *Foundation model* was coined by Stanford's Center for Research on Foundation Models in 2021 for models trained on broad data and adapted to many downstream tasks.

## How a VLA is built

```
camera images ─┐
               ├─► vision-language backbone (pretrained on web data) ─► action decoder ─► action chunk ─► low-level controller
instruction  ──┤                                                           ▲
robot state  ──┘                                                           └── tokens  |  FAST tokens  |  diffusion / flow head
```

The main design choice is **how actions come out**:

| Action decoding | How | Used by | Trade-off |
|---|---|---|---|
| **Discrete tokens** | Each action dimension binned (e.g. 256 bins) and emitted as text-like tokens | RT-2, OpenVLA | Reuses the language model as-is; slow and coarse for high-frequency control |
| **Compressed tokens** | Action chunks compressed (discrete cosine transform) into few tokens | FAST (Pertsch et al. 2025), π0-FAST | Faster autoregressive training on dexterous data |
| **Flow / diffusion action expert** | A separate network generates continuous action chunks, attending to the VLM's features | π0 family, GR00T N1.x | Smooth, precise, multimodal; see [[Diffusion_Policy]] |
| **Hierarchy** | A slow reasoning model plans; a fast VLA executes | Gemini Robotics ER + VLA, GR00T N1's "System 2 / System 1" design | Long-horizon tasks; two models to train and coordinate |

## How a VLA is trained

1. **Web pretraining.** The vision-language backbone learns from image-text data (captioning, visual question answering).
2. **Robot pretraining.** Train on large teleoperation datasets across many robots, often co-trained with web data so the model keeps its general knowledge. See [[Robot_Learning_Datasets_and_Benchmarks]].
3. **Post-training / fine-tuning.** Adapt to a specific robot or task with a few to tens of hours of data, often with parameter-efficient methods such as LoRA.
4. **Reinforcement learning (increasingly).** Improve from the robot's own successes and failures; see [[Reinforcement_Learning]].

## A short history

| Year | Model | What changed |
|---|---|---|
| 2022 | RT-1 (Google) | A transformer trained on 130,000 real episodes from 13 robots; showed data scale works |
| 2023 | RT-2 (Google DeepMind) | Fine-tuned a web VLM to output action tokens: web knowledge transferred to robot control |
| 2024 | Octo, OpenVLA | Open-source generalist policies trained on Open X-Embodiment |
| 2024 | π0 (Physical Intelligence) | VLM plus flow-matching action expert; dexterous tasks like laundry folding |
| 2025 | GR00T N1 (NVIDIA), Gemini Robotics (Google DeepMind), π0.5 | Humanoid-focused open model; VLA built on Gemini; open-world generalisation in homes |
| 2026 | π0.7, GR00T N1.7, Gemini Robotics 2 and 2.5, GR00T 2 | Steerable prompting, whole-body humanoid control, world-action models (below) |

## State of the art (as of November 2026)

- **π0.7 (Physical Intelligence, 16 April 2026).** Prompted not only with the task but with step-by-step language, metadata such as speed and quality, control-mode labels and subgoal images, which a world model can generate at test time. This lets suboptimal autonomous data be labelled rather than discarded. The company reports early compositional generalisation, such as operating kitchen appliances it was never shown.
- **GR00T N1.7 (NVIDIA, April 2026, early access).** Open-weight humanoid VLA: a vision-language backbone feeding a diffusion-transformer action head, available on Hugging Face and integrated with LeRobot.
- **Gemini Robotics 2 (Google DeepMind, 30 July 2026).** Three models: Gemini Robotics 2 (the VLA), Gemini Robotics ER 2 (an embodied-reasoning model that plans multi-minute tasks and coordinates several robots) and Gemini Robotics On-Device 2. The VLA controls full humanoids, including Apptronik's Apollo 2 with a 22-degree-of-freedom hand. ER 2 is available through Google AI Studio; the VLAs are limited to early-access partners. DeepMind notes that multi-finger dexterous manipulation "remains challenging".
- **Gemini Robotics 2.5 (10 November 2026).** Evaluated on 200 tasks; DeepMind reports 30–100% higher success than Gemini Robotics 2 depending on task type, and a 96% success rate in an automotive kitting pilot with Apptronik and Mercedes-Benz. These are vendor-reported figures.
- **From VLAs to world action models.** NVIDIA's GR00T 2 (November 2026) predicts future video and actions together rather than actions alone. See [[World_Models]].

## Worked example: the resolution of action tokens

OpenVLA discretises each of 7 action dimensions into 256 bins. Suppose the end-effector may move at most ±2 cm per step in $x$. The bin width is $4\,\text{cm} / 256 \approx 0.16$ mm. That is fine for most pick-and-place, but each action needs 7 sequential tokens, and a 7-billion-parameter model generates them at only a few actions per second. A flow-matching head instead outputs a whole chunk of continuous actions (π0 uses 50 steps) in roughly 10 integration steps. This is why most 2025–26 VLAs for dexterous work moved to continuous action heads.

## Open problems

| Problem | Why it is hard |
|---|---|
| Long multi-step autonomy | Errors accumulate over minutes; reports note π0.7 cannot yet complete complex multi-step tasks from one high-level command |
| Fine dexterity | Multi-finger contact is hard to demonstrate and to learn |
| Speed | Large models are slow; robots must react in tens of milliseconds |
| Evaluation | Real-robot tests are slow, noisy and easy to cherry-pick; see [[Robot_Learning_Datasets_and_Benchmarks]] |
| Safety | A model that follows open-ended instructions must refuse unsafe ones and respect physical limits |
| Openness | Some leading models are closed or partner-only; open weights (π0, π0.5, GR00T N1.x, OpenVLA, SmolVLA) drive research |

## Tooling

- **LeRobot** (Hugging Face): open implementations and fine-tuning for several VLAs (including π0-family and GR00T policies), datasets and real-robot drivers.
- **openpi** (Physical Intelligence): open weights and code for π0 and π0.5.
- **Isaac GR00T** (NVIDIA): open GR00T N1.x weights, fine-tuning and deployment code.

## Reading list

- Brohan et al. 2023, *RT-2*.
- Kim et al. 2024, *OpenVLA*.
- Black et al. 2024, *π0*.
- Physical Intelligence 2026, *π0.7*.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Vision-Language-Action_Models]])
```
