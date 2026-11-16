---
title: Machine Vision in Manufacturing
description: "Machine vision gives factory robots and machines sight: to locate parts, guide motion, inspect quality, measure and identify. Covers the four jobs of industrial vision, lighting, a computed example of resolution and motion blur on a conveyor, hand–eye calibration, bin picking, AI-based inspection trained on good parts only, and why false rejects dominate when defects are rare."
aliases:
  - Industrial Machine Vision
  - Vision-Guided Robotics
  - Automated Optical Inspection
  - Visual Inspection
tags:
  - computer-vision
  - manufacturing
  - calibration
  - automation
type: method
level: intermediate
subfield:
  - industrial-automation
  - perception
status: reviewed
author: Jordan_Smith
date: 2026-11-16
last_reviewed: 2026-11-16
as_of: 2026-11
prerequisites:
  - "[[Manufacturing]]"
  - "[[Camera_Systems]]"
related:
  - "[[CNN_in_Robotics]]"
  - "[[Robot_Work_Cell_Design]]"
  - "[[Computer_Vision_in_Robotics]]"
  - "[[Semantic_Scene_Understanding]]"
  - "[[Supervised_Learning]]"
key_takeaways:
  - "Industrial vision does four jobs: locate parts for a robot, inspect for defects, measure dimensions, and identify parts by codes or text."
  - "Resolution and exposure are arithmetic: a 200 mm field on 2,448 pixels gives 0.082 mm per pixel, and a 0.5 m/s conveyor needs an exposure under about 160 µs to keep blur below one pixel."
  - "When defects are rare, false alarms dominate: at a 1% defect rate, a detector with 99% recall and a 2% false-positive rate rejects two good parts for every bad one."
sources:
  - "Müller, C. (2026). World Robotics 2026 – Industrial Robots, executive summary. https://ifr.org/img/worldrobotics/Executive_Summary_WR_2026_-_Industrial_Robots.pdf"
  - "Tsai, R. Y. and Lenz, R. K. (1989). A new technique for fully autonomous and efficient 3D robotics hand/eye calibration. https://ieeexplore.ieee.org/document/34770"
  - "Bergmann, P., Fauser, M., Sattlegger, D. and Steger, C. (2019). MVTec AD — A Comprehensive Real-World Dataset for Unsupervised Anomaly Detection. https://www.mvtec.com/company/research/datasets/mvtec-ad"
  - "Roth, K. et al. (2021). Towards Total Recall in Industrial Anomaly Detection. https://arxiv.org/abs/2106.08265"
  - "International Federation of Robotics (2026). History of industrial robots. https://ifr.org/robot-history"
  - "Mahler, J. et al. (2017). Dex-Net 2.0: Deep Learning to Plan Robust Grasps with Synthetic Point Clouds and Analytic Grasp Metrics. https://arxiv.org/abs/1703.09312"
---

# Machine Vision in Manufacturing

**Machine vision** is the use of cameras and image processing to automate tasks that would otherwise need a person's eyes. In a factory it tells a robot where a part is, checks that every product is right, measures without touching, and reads codes. It is the form of AI that factories adopted first and trust most: the IFR's 2026 report describes machine vision for detection and robot guidance as established and industry-grade.

> **Etymology.** *Machine vision* is the industrial branch of [[Computer_Vision_in_Robotics|computer vision]]; the term spread in the 1980s. In 1981 General Motors' CONSIGHT system used vision and three robots to sort up to six kinds of castings from a moving conveyor, at up to 1,400 an hour.

## The four jobs

| Job | Question | Examples |
|---|---|---|
| **Locate** (guide) | Where is the part, exactly? | Pick from a conveyor or bin; align a part before assembly; find a weld seam |
| **Inspect** | Is it good? | Scratches, missing components, wrong assembly, solder joints |
| **Measure** (gauge) | How big is it? | Dimensions and gaps, without contact |
| **Identify** | Which part is it? | Barcodes, data-matrix codes, printed text |

## The imaging chain

**Lighting → lens → sensor → processing.** Lighting matters most: it decides whether a defect is visible at all. Backlights give sharp silhouettes for measuring; low-angle light makes scratches glow; diffuse domes tame shiny metal. A factory can control its lighting, and should. That control is the main reason industrial vision is more reliable than vision in the open world.

### Worked example: can the camera see the defect, on a moving belt?

A camera with a sensor 2,448 pixels wide views a field 200 mm wide.

- **Resolution.** Each pixel covers $200 / 2448 = 0.082$ mm. A defect should span at least 3–4 pixels to be detected reliably, so the smallest reliable defect is about 0.25–0.33 mm. To see 0.1 mm scratches, halve the field of view or use a higher-resolution sensor.
- **Motion blur.** Parts pass on a conveyor at 0.5 m/s (500 mm/s). During a 1 ms exposure a part moves 0.5 mm, which is 6 pixels of smear, enough to hide the defect. Keeping blur under one pixel needs an exposure below $0.082 / 500 = 163$ µs.

Short exposures need a lot of light, which is why inspection stations use strobe lights fired in step with a part-present sensor.

## Guiding robots

**2D guidance.** A camera above a conveyor finds each part's position and rotation, and the robot picks it without precise fixtures.

**3D guidance and bin picking.** Structured-light, stereo or time-of-flight sensors produce depth images of randomly piled parts. The system finds a part and a collision-free grasp. Learned grasp planners trained on simulated data, such as Dex-Net (2017), helped make bin picking of varied parts practical (see [[CNN_in_Robotics]]).

**Hand–eye calibration.** The robot needs the fixed transform between the camera and the robot. Moving the robot through several poses while viewing a calibration target gives equations of the form

$$A X = X B,$$

where $A$ is a robot motion, $B$ the matching camera motion, and $X$ the unknown camera-to-robot transform. Tsai and Lenz gave an efficient solution in 1989. A 1 mm calibration error becomes a 1 mm picking error, so calibration is checked whenever a camera is bumped or replaced.

## AI-based inspection

Classic inspection uses hand-set rules: thresholds, edges, template matching. Rules work for well-defined features and struggle with natural variation such as wood grain, castings or textiles.

**Learned inspection** has two forms:
- **Supervised**: train a classifier or detector on labelled images of each defect type (see [[Supervised_Learning]]). The catch is that defects are rare, so examples are scarce.
- **Anomaly detection**: train only on images of *good* parts and flag anything that looks different. The MVTec AD benchmark (2019), with over 5,000 images in 15 object and texture categories, made this a standard research problem. Methods such as PatchCore (2021), which compares image patches against a memory of normal features from a pretrained network, score very high on it.

### Worked example: why false rejects dominate

A line makes 10,000 parts a day and 1% are defective: 100 bad parts. The inspection system catches 99% of defects (recall) and wrongly flags 2% of good parts (false-positive rate).

| | Count per day |
|---|---|
| Bad parts caught | $100 \times 0.99 = 99$ |
| Bad parts missed | 1 |
| Good parts wrongly rejected | $9{,}900 \times 0.02 = 198$ |

Of the 297 parts rejected, only 99 are bad: one in three. Two good parts are thrown out, or sent for manual re-inspection, for every real defect. Cutting the false-positive rate to 0.2% leaves about 20 false rejects a day and makes five out of six rejects genuine. When defects are rare, the false-positive rate, not recall, usually decides whether an inspection system is economical.

## Status in 2026

- **Established**: vision-guided picking and placing, code reading, measurement, and rule-based inspection.
- **Increasingly used**: AI-based inspection, which the IFR lists alongside predictive maintenance and process optimisation.
- **Emerging**: open-vocabulary and foundation models that find or segment parts from a text prompt and label training data automatically (see [[Semantic_Scene_Understanding]]). They cut the cost of setting up a new product, but production lines still demand measured, validated accuracy before they are trusted.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Machine_Vision_in_Manufacturing]])
```
