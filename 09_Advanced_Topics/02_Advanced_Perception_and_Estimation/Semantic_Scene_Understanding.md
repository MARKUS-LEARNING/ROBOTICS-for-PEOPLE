---
title: Semantic Scene Understanding
description: "Semantic scene understanding gives a robot meaning, not just geometry: what objects are present, what they are for, and how they relate. Covers detection and segmentation (with a computed IoU example), open-vocabulary models (CLIP, Grounding DINO, SAM 2), 3D semantic mapping and scene graphs, affordances, and how vision-language models make scene understanding queryable in plain language in 2026."
aliases:
  - Scene Understanding
  - Semantic Mapping
  - Semantic Segmentation
  - Open-Vocabulary Perception
tags:
  - computer-vision
  - perception
  - object-detection
  - foundation-models
type: concept
level: advanced
subfield:
  - perception
  - robot-learning
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Computer_Vision_in_Robotics]]"
  - "[[CNN_in_Robotics]]"
related:
  - "[[Knowledge_Representation_and_Reasoning]]"
  - "[[Active_Perception_and_Sensing]]"
  - "[[Vision-Language-Action_Models]]"
  - "[[SLAM]]"
key_takeaways:
  - "Geometry says where surfaces are; semantics says what they are (cup, door, person), what they afford (graspable, openable) and how they relate (cup on table in kitchen)."
  - "Open-vocabulary models removed the fixed list of classes: a robot can be asked for 'the blue mug with a chipped handle' and find it without task-specific training."
  - "3D scene graphs anchor that meaning in a map that language models can query, which is how 2026 robots connect instructions to places and objects."
sources:
  - "Radford, A. et al. (2021). Learning Transferable Visual Models From Natural Language Supervision. https://arxiv.org/abs/2103.00020"
  - "Liu, S. et al. (2023). Grounding DINO: Marrying DINO with Grounded Pre-Training for Open-Set Object Detection. https://arxiv.org/abs/2303.05499"
  - "Ravi, N. et al. (2024). SAM 2: Segment Anything in Images and Videos. https://arxiv.org/abs/2408.00714"
  - "Gu, Q. et al. (2023). ConceptGraphs: Open-Vocabulary 3D Scene Graphs for Perception and Planning. https://arxiv.org/abs/2309.16650"
  - "Hughes, N., Chang, Y. and Carlone, L. (2022). Hydra: A Real-time Spatial Perception System for 3D Scene Graph Construction and Optimization. https://arxiv.org/abs/2201.13360"
---

# Semantic Scene Understanding

A map made only of geometry tells a robot where it can drive. To fetch a mug, clear a table or open a door, it needs **semantics**: what each thing is, what it can be used for, and how things relate. **Semantic scene understanding** adds that meaning to perception.

> **Etymology.** Greek *sēmantikos*, "significant", from *sēma*, "sign". In robotics, "semantic mapping" became a research area in the 2000s; open-vocabulary, language-queryable versions arrived with foundation models around 2022–23.

## Levels of understanding

| Level | Question | Methods |
|---|---|---|
| **Classification** | What is in this image? | CNNs, ViTs |
| **Detection** | Where are the objects (boxes)? | YOLO family, DETR-style detectors |
| **Semantic segmentation** | Which class is each pixel? | Segmentation networks |
| **Instance segmentation** | Which pixels belong to which object? | Mask-based networks, SAM 2 |
| **3D semantic mapping** | Where are the objects in the world, over time? | Fusing 2D results into 3D maps (voxels, point clouds, Gaussian splats) |
| **Scene graphs** | How are objects, places and rooms related? | Hydra, ConceptGraphs |
| **Affordances** | What can I do with it, and where do I grasp or press it? | Learned affordance models, VLMs |

## Worked example: measuring a detection

Detections are scored by **intersection over union (IoU)**. A predicted box spans (0, 0)–(4, 4) and the true box (1, 1)–(5, 5):
- the intersection is the square (1, 1)–(4, 4), area 9;
- the union is $16 + 16 - 9 = 23$;
- IoU $= 9/23 = 0.39$.

Under the common threshold of 0.5 this counts as a **miss**, even though the boxes overlap substantially. For a grasping robot, a 1-unit offset at the scale of a mug can mean grasping air, so robots often need tighter localisation than detection benchmarks reward, and refine with depth and segmentation.

## Open-vocabulary perception

Models trained on image–text pairs connect pixels to language:
- **CLIP** (2021) scores how well an image matches a phrase;
- **Grounding DINO** (2023) finds boxes for any text query;
- **SAM 2** (2024) segments any object in images and video from a prompt.

Chained together, a robot can find "the charger cable behind the monitor" without having been trained on that class (see [[Deep_Learning]]).

## Scene graphs: semantics you can query

A **3D scene graph** organises a building into layers (building → rooms → places → objects), with relations such as *on*, *in* and *next to*, anchored in a metric map. Hydra builds them in real time; ConceptGraphs labels nodes with open-vocabulary features so a language model can answer "where could I find something to cut paper?" by searching the graph. This is how instructions are grounded in specific places and objects (see [[Knowledge_Representation_and_Reasoning]]).

## Semantics inside policies

[[Vision-Language-Action_Models]] carry semantic understanding inside the policy, inherited from their vision-language backbones; RT-2 showed web knowledge transferring to robot actions on new objects. Explicit semantic maps remain valuable for memory, long-range navigation and inspection by people.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Semantic_Scene_Understanding]])
```
