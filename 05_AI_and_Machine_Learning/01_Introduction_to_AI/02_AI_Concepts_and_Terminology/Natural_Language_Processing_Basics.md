---
title: Natural Language Processing Basics
description: "Natural language processing (NLP) lets machines read, interpret and produce human language. Covers tokens, embeddings and language models, the shift from rules and statistics to large language models, and how robots use language in 2026: instructions grounded in what the robot can do (with a computed SayCan-style example), dialogue and feedback, and language inside vision-language-action models."
aliases:
  - NLP
  - Natural Language Processing
  - Large Language Models
  - LLM
tags:
  - natural-language-processing
  - artificial-intelligence
  - human-robot-interaction
  - foundation-models
type: concept
level: beginner
subfield:
  - robot-learning
  - human-robot-interaction
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Machine_Learning]]"
related:
  - "[[Transformers_and_Attention]]"
  - "[[Vision-Language-Action_Models]]"
  - "[[Knowledge_Representation_and_Reasoning]]"
  - "[[Agents_and_Environments]]"
key_takeaways:
  - "Modern NLP turns text into tokens, maps tokens to vectors (embeddings), and uses transformer language models trained to predict the next token on enormous text collections."
  - "Language models know a lot about the world but nothing about this robot's body or surroundings; grounding means combining what the model says is useful with what the robot can actually do."
  - "In 2026 language reaches robots three ways: high-level planners that call skills, conversational interfaces for people, and the language backbone inside vision-language-action models."
sources:
  - "Jurafsky, D. and Martin, J. H. (2026). Speech and Language Processing, 3rd edition draft. https://web.stanford.edu/~jurafsky/slp3/"
  - "Vaswani, A. et al. (2017). Attention Is All You Need. https://arxiv.org/abs/1706.03762"
  - "Brown, T. et al. (2020). Language Models are Few-Shot Learners. https://arxiv.org/abs/2005.14165"
  - "Ahn, M. et al. (2022). Do As I Can, Not As I Say: Grounding Language in Robotic Affordances. https://arxiv.org/abs/2204.01691"
  - "Huang, W. et al. (2022). Inner Monologue: Embodied Reasoning through Planning with Language Models. https://arxiv.org/abs/2207.05608"
  - "Brohan, A. et al. (2023). RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control. https://arxiv.org/abs/2307.15818"
---

# Natural Language Processing Basics

**Natural language processing (NLP)** is the part of AI concerned with human language: understanding it, answering in it, translating it, and acting on it. For robots, language is the most natural interface. People would rather say "please clear the table" than program a trajectory.

> **Etymology.** *Natural* language, as opposed to *formal* languages such as programming languages and logic. *Language* is from Latin *lingua*, "tongue". The field began with machine-translation experiments in the 1950s.

## From rules to large language models

| Era | Approach | Strength | Weakness |
|---|---|---|---|
| 1950s–80s | Hand-written grammars and rules | Precise on narrow inputs | Brittle; endless exceptions |
| 1990s–2010s | Statistical models (n-grams, hidden Markov models), then word embeddings (word2vec, 2013) | Learned from data | Shallow understanding |
| 2017– | **Transformer language models** (see [[Transformers_and_Attention]]) | Fluent, broad knowledge, few-shot learning | Can be confidently wrong; no built-in grounding in the physical world |

## The core ideas

1. **Tokens.** Text is split into sub-word pieces ("un", "screw", "ing"). A typical English word is about 1.3 tokens.
2. **Embeddings.** Each token becomes a vector; similar meanings end up near each other.
3. **Language model.** A network trained to predict the next token given the previous ones. Trained on trillions of tokens, this simple objective produces models that summarise, translate, answer questions, write code and plan.
4. **Instruction tuning and feedback.** Further training on instructions and human preferences turns a next-token predictor into an assistant that follows requests.

## Grounding: language meets the robot's body

A language model asked "I spilled my drink, can you help?" might suggest "use a vacuum". That is useless if the robot has no vacuum. **Grounding** connects what the model says to what the robot can perceive and do.

### Worked example: SayCan-style skill selection

SayCan (2022) scores each skill by two numbers: how useful the language model thinks it is for the instruction, and how likely the robot is to succeed at it from the current state (a learned **affordance** value). It picks the skill with the highest product.

Instruction: "I spilled my drink, can you help?"

| Skill | Language-model usefulness | Affordance (can do it now) | Product |
|---|---|---|---|
| find a sponge | 0.40 | 0.90 | **0.360** |
| go to the kitchen | 0.50 | 0.20 | 0.100 |
| pick up the apple | 0.10 | 0.95 | 0.095 |

The robot chooses "find a sponge". The language model alone preferred the kitchen; the affordance alone preferred the apple; together they choose something both sensible and feasible. The process repeats after each step, with the completed steps added to the prompt.

## Language on robots in 2026

| Role | How | See |
|---|---|---|
| **Task planning** | A language or vision-language model breaks an instruction into skill calls or code; feedback (success detection, scene descriptions) loops back, as in Inner Monologue (2022) | [[Knowledge_Representation_and_Reasoning]] |
| **Conversation** | Speech recognition + language model + speech synthesis; asking clarifying questions | [[Agents_and_Environments]] |
| **Inside the policy** | Vision-language-action models take the instruction as tokens alongside images, and output actions directly; RT-2 (2023) showed web knowledge transferring to new objects and concepts | [[Vision-Language-Action_Models]] |

**Caution.** Language models can hallucinate facts and produce unsafe plans with complete confidence. Robot systems therefore constrain them:
- to a fixed set of skills;
- with checks on preconditions;
- by asking for confirmation before irreversible actions;
- with independent safety layers.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Natural_Language_Processing_Basics]])
```
