---
title: CNN in Robotics
description: "How convolutional networks are used on real robots: real-time object detection (YOLO), instance segmentation, monocular depth, grasp prediction from depth images, and image encoders inside learned policies. Includes a worked latency budget for a 30 Hz camera pipeline on embedded hardware and guidance on CNNs versus vision transformers in 2026."
aliases:
  - CNNs in Robotics
tags:
  - neural-networks
  - computer-vision
  - object-detection
  - grasping
  - robot-learning
type: method
level: intermediate
subfield:
  - perception
  - robot-learning
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Convolutional_Neural_Networks]]"
related:
  - "[[Computer_Vision_in_Robotics]]"
  - "[[Diffusion_Policy]]"
  - "[[Grasping]]"
  - "[[Transformers_and_Attention]]"
  - "[[Camera_Systems]]"
key_takeaways:
  - "CNNs do most of a robot's real-time image work: detection, segmentation, depth and keypoints, at camera frame rates on embedded GPUs."
  - "Grasping from depth images with CNNs (Dex-Net, 2017) was an early large-scale success of learning in manipulation; today's policies use CNN or ViT encoders end to end."
  - "Budget latency end to end: capture, preprocessing, network and postprocessing must fit inside the frame period, or the robot acts on stale images."
sources:
  - "Redmon, J. et al. (2015). You Only Look Once: Unified, Real-Time Object Detection. https://arxiv.org/abs/1506.02640"
  - "He, K., Gkioxari, G., Dollár, P. and Girshick, R. (2017). Mask R-CNN. https://arxiv.org/abs/1703.06870"
  - "Mahler, J. et al. (2017). Dex-Net 2.0: Deep Learning to Plan Robust Grasps with Synthetic Point Clouds and Analytic Grasp Metrics. https://arxiv.org/abs/1703.09312"
  - "Yang, L. et al. (2024). Depth Anything V2. https://arxiv.org/abs/2406.09414"
  - "Ultralytics (2026). Ultralytics YOLO (YOLO26 models). https://github.com/ultralytics/ultralytics"
  - "Chi, C. et al. (2023). Diffusion Policy: Visuomotor Policy Learning via Action Diffusion. https://arxiv.org/abs/2303.04137"
---

# CNN in Robotics

[[Convolutional_Neural_Networks|Convolutional neural networks]] are the workhorses of robot vision. Wherever a robot needs to know what is in a camera image, where it is and how far away it is, within the time of one camera frame, there is usually a CNN.

> **Etymology.** See [[Convolutional_Neural_Networks]]. *CNN* became robotics vocabulary after 2012, when AlexNet's ImageNet result made learned vision clearly better than hand-designed features.

## Main uses

| Task | Output | Typical models (2026) | Robot use |
|---|---|---|---|
| **Object detection** | Boxes + class labels | YOLO family (YOLO26), RT-DETR | Find people, parts, obstacles |
| **Instance segmentation** | Pixel masks per object | Mask R-CNN, YOLO-seg; SAM 2 (transformer) for any object | Pick a specific item from clutter |
| **Monocular depth** | Depth per pixel from one image | Depth Anything V2 (ViT encoder + convolutional decoder) | Obstacle sensing without lidar |
| **Keypoints and pose** | Joints of people, 6-DoF object pose | Pose networks, FoundationPose | Human-robot interaction, part insertion |
| **Grasp prediction** | Grasp quality per candidate or pixel | Dex-Net, GG-CNN | Bin picking |
| **Policy encoders** | Feature vector per image | ResNet-18/34 (often with spatial softmax), or ViT | Inputs to [[Diffusion_Policy]] and other visuomotor policies |

## Two landmark examples

**Grasping from depth (Dex-Net 2.0, 2017).** A CNN trained on 6.7 million *synthetic* depth images, labelled with physics-based grasp robustness, predicted grasp success on real objects with high reliability. It showed that simulation-generated data could train real-robot vision, an early sim-to-real success (see [[Sim-to-Real_Transfer]]).

**Visuomotor policies (2016–2026).** Instead of detecting objects and then planning, the CNN's features feed directly into a network that outputs actions, trained end to end from demonstrations. The ResNet-18 image encoder in Diffusion Policy (2023) is typical: small enough to run at 10+ Hz on a laptop GPU, rich enough for contact-rich tasks.

## Worked example: a 30 Hz latency budget

A mobile robot's camera runs at 30 Hz, so a new frame arrives every 1000/30 = 33.3 ms. On an embedded GPU the pipeline costs:

| Stage | Time |
|---|---|
| Capture and transfer | 5 ms |
| Resize and normalise | 1.7 ms |
| Detector inference (small YOLO, FP16) | 8 ms |
| Postprocessing (non-maximum suppression, tracking) | 2 ms |
| **Total** | **16.7 ms** |

That leaves about 17 ms of slack per frame. Add a 20 ms depth network on the same GPU and the total reaches 36.7 ms, more than one frame. Frames start queuing, and the robot reacts to images that are older every second. Fixes: run depth at 15 Hz, use a smaller model, quantise to INT8 with TensorRT, or move a stage to another accelerator. Always measure on the target hardware; desktop benchmarks mislead.

## CNN or vision transformer?

| Choose a CNN when… | Choose a ViT when… |
|---|---|
| Latency and power are tight (embedded, battery) | You use a large pretrained foundation model (CLIP, DINOv2, a VLA backbone) |
| Training data is modest | Data or pretraining is abundant |
| The task is well defined (detect these 20 classes) | The task is open-ended (any object named in language) |

Hybrids are common: Depth Anything V2 pairs a transformer encoder with a convolutional decoder, and many detectors mix both.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[CNN_in_Robotics]])
```
