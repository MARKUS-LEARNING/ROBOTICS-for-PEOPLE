---
title: Event-Based Sensing and Processing
description: "Event cameras report per-pixel brightness changes asynchronously with microsecond timing instead of full frames. Covers how they work, their advantages (latency, dynamic range, no motion blur) with a computed blur comparison, processing methods (event frames, filters, spiking and learned networks), robotics uses such as fast obstacle dodging, and limits as of 2026."
aliases:
  - Event Cameras
  - Event Camera
  - Neuromorphic Vision
  - Dynamic Vision Sensor
tags:
  - computer-vision
  - sensors
  - perception
type: concept
level: advanced
subfield:
  - perception
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Camera_Systems]]"
related:
  - "[[Computer_Vision_in_Robotics]]"
  - "[[Drones]]"
  - "[[Advanced_Sensor_Fusion]]"
key_takeaways:
  - "Each pixel of an event camera fires independently when its log-brightness changes by a threshold, producing a stream of (x, y, time, polarity) events with microsecond resolution."
  - "No exposure means no motion blur: an object at 10 m/s smears 10 cm across a 10 ms frame exposure, while events track it continuously; dynamic range exceeds 120 dB."
  - "They shine for fast motion, high-dynamic-range scenes and low power; algorithms and datasets lag frame-based vision, so most robots in 2026 pair them with ordinary cameras."
sources:
  - "Gallego, G. et al. (2019). Event-based Vision: A Survey. https://arxiv.org/abs/1904.08405"
  - "Falanga, D., Kleber, K. and Scaramuzza, D. (2020). Dynamic obstacle avoidance for quadrotors with event cameras. https://www.science.org/doi/10.1126/scirobotics.aaz9712"
  - "Siciliano, B. and Khatib, O., eds. (2016). Springer Handbook of Robotics, 2nd edition (vision sensors). https://link.springer.com/referencework/10.1007/978-3-319-32552-1"
---

# Event-Based Sensing and Processing

An **event camera** does not take pictures. Each of its pixels independently watches the brightness it sees and emits an **event** the moment that brightness changes by more than a threshold: position, timestamp (microsecond resolution) and polarity (brighter or darker). A static scene produces almost no data; a moving edge produces a dense stream exactly where and when it moves.

> **Etymology.** *Event* from Latin *evenire*, "to come out, happen". The sensors descend from Carver Mead's *neuromorphic* (brain-shaped) engineering at Caltech in the late 1980s; the Dynamic Vision Sensor (DVS) from ETH Zurich and the University of Zurich (2008) made them practical.

## Why they are different

| Property | Frame camera | Event camera |
|---|---|---|
| Output | Full image every 1/30 s or so | Asynchronous events per pixel |
| Latency | Up to one frame period (33 ms at 30 Hz) | Microseconds |
| Motion blur | Yes, during exposure | None (no exposure) |
| Dynamic range | ~60–70 dB | >120 dB |
| Data when nothing moves | Full frames anyway | Almost none |
| Absolute brightness and texture | Yes | Only changes; static scenes are invisible |

## Worked example: motion blur

A ball thrown at 10 m/s passes a camera. In a frame with a 10 ms exposure, the ball moves $10 \times 0.01 = 0.1$ m, a 10 cm smear. An event camera reports its edges continuously as they move, with no exposure-time smear, so its position can be estimated at any instant. Event-based drones have used this to detect and dodge objects thrown at them with reaction times of a few milliseconds (Falanga et al., 2020).

## Processing events

- **Event frames and voxel grids**: accumulate events over short windows into image-like tensors, so standard CNNs and transformers can be used.
- **Filters and model fitting**: update estimates (feature tracks, optical flow, pose) event by event.
- **Spiking neural networks** on neuromorphic chips: process events natively, at very low power.
- **Learned networks** trained on event data or on simulated events (generated from video).
- **Fusion with frames and IMUs** for visual-inertial odometry robust to fast motion and harsh lighting (see [[Advanced_Sensor_Fusion]]).

## Uses and limits (2026)

Fast drone navigation and obstacle avoidance, high-speed tracking, tactile sensing (event-based touch), automotive night and glare conditions, and always-on low-power monitoring. Commercial sensors exist from several vendors. Limits: noisy output in low light, no information in static scenes, fewer large datasets and pretrained models than for frames, and processing pipelines that are less mature. For now event cameras are mostly a specialist addition rather than a replacement.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Event_Based_Sensing_and_Processing]])
```
