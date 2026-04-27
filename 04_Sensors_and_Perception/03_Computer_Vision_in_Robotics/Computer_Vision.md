---
title: Computer Vision
description: The field that builds machines capable of interpreting images and video — from raw pixels to scene-level understanding. Covers the classical era (geometry + hand-engineered features, 1970–2010) and the deep era (end-to-end learning, 2012–present), with emphasis on how each is used inside a robotics stack.
tags:
  - computer-vision
  - perception
  - robotics
  - image-processing
  - deep-learning
  - feature-detection
layout: default
category: robotics
author: Jordan_Smith
date: 2025-05-02
permalink: /computer_vision/
related:
  - "[[Perception]]"
  - "[[Camera_Systems]]"
  - "[[Sensors]]"
  - "[[Computer_Vision_in_Robotics]]"
  - "[[Sensor_Fusion]]"
  - "[[Deep_Learning]]"
  - "[[SLAM]]"
  - "[[Object_Recognition]]"
  - "[[Image_Processing]]"
---

# Computer Vision

**Computer vision (CV)** is the field of building machines that interpret images and video — turning a 2D array of pixel intensities into descriptions of objects, geometry, motion, and actions in the 3D world. It is the *interpretive* counterpart to [[Camera_Systems]] (the sensor itself) — what the sensor produces is light measurements; what the application needs is statements like "there is a chair at $(x, y, z)$, oriented $\theta$." Computer vision is the machinery in between.

> **Etymology.** *Vision* — Latin *visio*, from *videre*, "to see." Same root as *vista*, *visa*, *evidence* (literally, "out of sight"), and *video* ("I see"). The phrase *computer vision* dates to **1966**, when Marvin Minsky and Seymour Papert at MIT assigned the legendary "Summer Vision Project" to undergraduate Gerald Sussman as a *summer-long* task: *"connect a camera to a computer and have the computer describe what it sees."* Sixty years later the project is still ongoing — the original underestimate is the most-cited gaffe in AI history. The field formally crystallized in the early 1970s with David Marr's group at MIT and Hans-Hellmut Nagel's at Karlsruhe.

---

## What computer vision tries to do

A useful taxonomy of CV tasks, from low-level to high-level:

| Level | Task | Example |
|---|---|---|
| **Low-level** | Image processing | Denoise, sharpen, demosaic Bayer raw |
| | Edge / corner detection | Canny, Harris, FAST |
| | Optical flow | Lucas-Kanade, Farneback, RAFT |
| **Mid-level** | Feature description | SIFT, ORB, SuperPoint |
| | Stereo matching | Block matching, SGM, RAFT-Stereo |
| | Visual odometry | DSO, ORB-SLAM3 front-end |
| **High-level** | Image classification | ResNet, ViT |
| | Object detection | YOLO, DETR |
| | Semantic / instance segmentation | Mask R-CNN, SAM |
| | Pose estimation | OpenPose, 6-DoF object pose |
| | Depth estimation | MiDaS, Depth Anything |
| | Action / activity recognition | I3D, SlowFast |
| **Scene-level** | Scene reconstruction | SfM, NeRF, Gaussian splatting |
| | Visual question answering | LLaVA, GPT-4V |

A robot's perception stack typically uses a small slice from each level — corner detection for visual odometry, segmentation for object isolation, depth for grasp planning, scene reconstruction for navigation.

---

## The two eras

### Classical era — 1970–2012

Hand-engineered features + geometric models + statistical inference.

**The story:** humans designed feature detectors that were *invariant* to lighting, viewpoint, and scale. The detector finds keypoints; descriptors summarize the local image patch as a vector; matching across images plus a geometric model (epipolar geometry, homography, fundamental matrix) recovers structure and motion.

Key milestones:

- **1962** — Hubel & Wiesel discover edge-detecting cells in cat visual cortex (motivation for filter-based vision).
- **1980** — Marr's *Vision* book lays out the "primal sketch → 2½D sketch → 3D model" hierarchy.
- **1986** — Canny edge detector (J. Canny, MIT).
- **1988** — Harris corner detector.
- **1999** — Lowe's SIFT (Scale-Invariant Feature Transform). The most-cited CV paper of the classical era.
- **2003** — Davison's MonoSLAM — single camera + EKF + Shi-Tomasi features → real-time SLAM.
- **2011** — ORB (Rublee et al.) — fast, license-friendly SIFT replacement.

The classical pipeline shines on **geometric** problems: SLAM, structure-from-motion, calibration, image stitching. Even today, the metric layer of every visual SLAM system is classical bundle adjustment.

### Deep era — 2012–present

End-to-end learning replaces hand-engineered features.

**The shock:** in 2012, Krizhevsky, Sutskever & Hinton's *AlexNet* (a deep CNN) won the ImageNet challenge by 10+ percentage points over the classical entry. Within five years, every CV benchmark was dominated by deep learning. By 2020 the very *features* in a SLAM front-end (SuperPoint, R2D2, DISK) became learned.

Key milestones:

- **2012** — AlexNet (8 layers, 60M parameters) wins ImageNet.
- **2014** — VGG-16, GoogLeNet, R-CNN.
- **2015** — ResNet (152 layers, residual connections).
- **2016** — YOLO real-time object detection.
- **2017** — Mask R-CNN, end-to-end instance segmentation.
- **2020** — Vision Transformer (ViT) — transformers begin replacing CNNs.
- **2020** — NeRF — neural radiance fields render photorealistic novel views.
- **2021** — CLIP — text-image contrastive learning, zero-shot classification.
- **2023** — Segment Anything (SAM) — prompted segmentation works on arbitrary images.
- **2023** — Gaussian splatting — real-time photoreal scene reconstruction.
- **2024** — Vision-Language-Action (VLA) models for robotics (RT-2, OpenVLA, π₀).

---

## Classical primitives every roboticist should still know

### Image filtering — convolution

The basic operation. A small kernel slides over the image, computing a weighted sum:

$$
(I * K)(x, y) = \sum_{i, j} K(i, j) \cdot I(x - i, y - j)
$$

Different kernels do different things: Gaussian blur, derivative kernels (Sobel, Prewitt) for edges, sharpening kernels for crispness. CNNs scale this idea up by *learning* the kernel weights.

### Edge detection — Canny

1. Gaussian blur to suppress noise.
2. Compute gradient magnitude and direction (Sobel).
3. Non-maximum suppression: keep only local maxima along the gradient direction.
4. Hysteresis thresholding: track strong edges, extend through weak edges.

Output: a binary edge map. The most-implemented CV algorithm of all time. Used as a front-end for line detection (Hough transform), structure-from-shading, and many feature detectors.

### Corner detection — Harris / Shi-Tomasi

A *corner* is a point where the local image patch changes in two perpendicular directions. Harris computes a structure tensor $M$ and reports the corner score $R = \det(M) - k \cdot \text{trace}(M)^2$. Shi-Tomasi uses $\min(\lambda_1, \lambda_2)$ instead. The corners are stable across viewpoint and ideal for tracking.

### Feature description — SIFT / ORB

Given a keypoint, build a vector summarizing the local image patch — invariant to scale, rotation, illumination. SIFT uses gradient orientation histograms (128-d float vector); ORB uses binary patterns from intensity comparisons (256-bit string). Match by nearest neighbor in descriptor space.

### Geometric models — epipolar geometry

Two views of the same scene satisfy the **epipolar constraint** $\mathbf{x}'^T F \mathbf{x} = 0$, where $F$ is the **fundamental matrix**. Recovering $F$ from feature correspondences (8-point algorithm, RANSAC outlier rejection) lets us:

- Triangulate matched feature points → 3D structure.
- Recover camera relative pose (essential matrix).
- Reject outlier matches (anything that doesn't satisfy the constraint).

Visual SLAM and structure-from-motion are essentially the iterative refinement of this two-view problem to many views.

---

## Deep-learning primitives for robotics

### Convolutional networks (CNNs)

Stack convolutions, non-linearities, and pooling. Modern variants (ResNet, EfficientNet, ConvNeXt) keep this structure with residual connections. CNNs remain the workhorse for embedded perception (low compute, low latency).

### Vision transformers (ViTs)

Treat the image as a sequence of patches and apply attention. Better at large-scale tasks, expensive on small devices. Hybrid CNN-transformer designs (Swin, MViT) bridge the two.

### Object detection — YOLO and DETR

**YOLO** (You Only Look Once, Redmon 2016) — single network produces bounding boxes and classes in one pass. YOLOv8 / v10 are real-time on CPU. **DETR** (Carion 2020) — transformer-based, more elegant but slower.

For robot manipulation: detect, segment, get a 3D pose (via depth from RGB-D), plan a grasp. The whole loop runs at 30+ Hz on a Jetson Orin.

### Segment Anything (SAM)

A *promptable* segmentation model — you click a point, draw a box, or even type text, and SAM returns a mask. Released 2023 by Meta AI. For robotics it eliminates the need to train a custom segmentation network for each new object.

### Vision-language models

CLIP (Radford 2021) learns a joint embedding of image and text, enabling zero-shot classification. LLaVA, GPT-4V, and Gemini extend this to question-answering and multi-step reasoning. The robotics impact is **Vision-Language-Action (VLA) models** — RT-2 (DeepMind 2023), OpenVLA (2024), π₀ (Physical Intelligence 2024) — that consume an image plus a natural-language goal and emit motor actions directly.

---

## Computer vision *in robotics* — what's different

A research-grade CV paper optimizes for accuracy on a static benchmark. A robotics CV system has constraints the benchmark doesn't measure:

1. **Latency.** A controller running at 100 Hz cannot wait 200 ms for object detection. Real-time inference (or at minimum, low-latency feature extraction with downstream temporal smoothing) is mandatory.
2. **Onboard compute.** A 50 W Jetson NX is the typical compute budget. A 350 W RTX 4090 is not.
3. **Distribution shift.** The robot sees a never-before-seen lighting / viewpoint / object. The model must degrade gracefully, not silently fail.
4. **Calibration.** Pixels alone are useless without a 3D ray. The CV system depends on accurate intrinsic and extrinsic calibration. See [[Camera_Systems]].
5. **Uncertainty.** A robot needs a *confidence* on every detection so the planner knows when to act and when to wait. Most deep-learning models don't expose this directly.
6. **Safety.** A misclassification has consequences in physical space. Conservative fallbacks (uncertainty-aware control, safety envelopes) are part of the system, not the model.

See [[Computer_Vision_in_Robotics]] for the detailed integration story.

---

## Worked example — visual servoing to a coffee mug

A simple but complete CV pipeline:

1. **Capture** — RGB-D frame from a wrist camera (640×480, 30 Hz).
2. **Detect** — YOLO11n outputs `cup, bbox, conf=0.94`.
3. **Segment** — SAM (or a smaller distilled model) refines bbox → per-pixel mask.
4. **3D point** — masked depth pixels become a partial point cloud of the mug; cluster + centroid → 3D position in camera frame.
5. **Coordinate transform** — apply `tf2` lookup `camera → base_link` → mug position in robot frame.
6. **Track** — Kalman filter on 3D position smooths jitter and predicts motion.
7. **Plan** — inverse-kinematics solve to a pre-grasp pose 5 cm above the mug.

Steps 2-4 are deep learning; steps 5-7 are classical robotics. The pipeline runs at camera rate (30 Hz) on a Jetson Orin. Take any one piece out and the robot can't reach the mug.

---

## Tooling

| Tool | Use |
|---|---|
| **OpenCV** | Classical CV — calibration, features, stereo, optical flow |
| **PyTorch / JAX** | Deep CV training and inference |
| **Ultralytics YOLO** | Pre-trained real-time detection |
| **Detectron2 / MMDetection** | Research-grade detection / segmentation |
| **Open3D, PCL** | Point-cloud processing |
| **Kalibr, ROS `camera_calibration`** | Camera + IMU calibration |
| **TensorRT, ONNX Runtime, OpenVINO** | Edge inference acceleration |
| **`vision_msgs` (ROS 2)** | Standard detection/pose-output messages |
| **Foxglove Studio, RViz** | Visualization with overlays |

---

## Recommended reading

- Hartley & Zisserman, *Multiple View Geometry in Computer Vision* (2nd ed., 2003) — *the* classical-CV reference
- Szeliski, *Computer Vision: Algorithms and Applications* (2nd ed., 2022) — the modern textbook (free PDF)
- Goodfellow, Bengio, Courville, *Deep Learning* (2016) — the deep-learning foundation
- Marr, *Vision* (1982) — the philosophical foundation
- Lowe (2004), *Distinctive Image Features from Scale-Invariant Keypoints* — the SIFT paper
- Krizhevsky, Sutskever, Hinton (2012), *ImageNet Classification with Deep Convolutional Neural Networks* — AlexNet
- Mildenhall et al. (2020), *NeRF: Representing Scenes as Neural Radiance Fields*
- Brohan et al. (2023), *RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control*

---

## Dataview

```dataview
LIST FROM #computer-vision OR #perception WHERE contains(file.outlinks, [[Computer_Vision]])
```
