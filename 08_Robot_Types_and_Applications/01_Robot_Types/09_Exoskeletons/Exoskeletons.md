---
title: Exoskeletons
description: "Exoskeletons are wearable robots that add strength, support or assistance to a person's own movement: medical devices for walking after spinal cord injury or stroke, industrial suits that reduce strain, and soft exosuits that cut the energy cost of walking. Covers types, how assistance is controlled, a computed assistance-power example, the learning-based advances of 2022–2024 (real-world personalisation, deep-network joint-moment estimation), and open problems."
aliases:
  - Exoskeleton
  - Exosuit
  - Wearable Robots
tags:
  - exoskeleton
  - human-robot-interaction
  - medical-robots
  - control
type: robot-type
level: intermediate
subfield:
  - assistive-rehabilitation
  - human-robot-interaction
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Robots]]"
  - "[[Control_Systems]]"
related:
  - "[[Medical_Robotics]]"
  - "[[Soft_Robotics]]"
  - "[[Bio-inspired_Robotics]]"
  - "[[Neural_Networks]]"
key_takeaways:
  - "An exoskeleton works in parallel with the body: it must add the right torque at the right moment of each movement, or it gets in the way."
  - "Assistance is modest in torque but well timed: about 0.2 N·m per kg at the hip is typical; personalised ankle assistance cut the energy of walking a given distance by 17% in real-world tests (Nature, 2022)."
  - "Learning is changing control: a deep network estimating the wearer's own joint moments let one controller assist across 28 activities (Nature, 2024), instead of one tuned mode per task."
sources:
  - "Slade, P. et al. (2022). Personalizing exoskeleton assistance while walking in the real world. https://www.nature.com/articles/s41586-022-05191-1"
  - "Molinaro, D. D. et al. (2024). Task-agnostic exoskeleton control via biological joint moment estimation. https://www.nature.com/articles/s41586-024-08157-7"
  - "Kim, J. et al. (2019). Reducing the metabolic rate of walking and running with a versatile, portable exosuit. https://www.science.org/doi/10.1126/science.aav7536"
---

# Exoskeletons

An **exoskeleton** is a robot you wear. Motors or springs act alongside your joints to add strength, reduce effort, support posture, or help you walk again after injury. Unlike other robots, it shares control with a human body that is already moving, so timing and comfort matter as much as power.

> **Etymology.** Greek *exo*, "outside", + *skeleton*: a skeleton on the outside, like an insect's or crab's shell. Early powered attempts include General Electric's Hardiman (1965), which was too heavy to be practical. In 2014 the FDA cleared the first powered exoskeleton for people with spinal cord injury (ReWalk).

## Types

| Type | Purpose | Examples of use |
|---|---|---|
| **Medical / rehabilitation** | Help people with paralysis, stroke or weakness stand, walk and train | Clinic-based gait training; personal devices for spinal cord injury |
| **Industrial (passive)** | Springs and counterweights support arms or back; no motors | Overhead work, lifting in logistics and manufacturing |
| **Industrial (powered)** | Motors assist lifting and carrying | Heavy-goods handling |
| **Soft exosuits** | Textile suits with cables driven by small motors; light and comfortable | Reducing the energy cost of walking and running |
| **Military and field** | Load carriage | Mostly research and trials |

## Control: assist the person, not fight them

The controller must decide, many times per second, how much torque to add at each joint. Common strategies:
- **Gait-phase-based**: detect heel strike and toe-off with IMUs or pressure sensors, then apply a torque profile at the right percentage of the stride.
- **Impedance / admittance**: behave like a spring and damper around the wearer's motion.
- **Human-in-the-loop optimisation**: adjust the assistance profile to minimise the wearer's measured effort.
- **Learned models**: estimate the wearer's intended joint moments from sensors with a neural network, and supply a fraction of them.

## Worked example: how much help is enough?

Hip assistance is often specified per kilogram of body mass. At 0.2 N·m/kg, an 80 kg person receives $0.2 \times 80 = 16$ N·m at the hip. At a hip angular velocity of about 3 rad/s during brisk walking, the peak mechanical power delivered is $16 \times 3 = 48$ W, roughly the output of a small drone motor. The effect is far larger than the numbers suggest because the help arrives exactly when the muscles would otherwise work hardest. Mistimed, the same torque *increases* effort.

## Recent advances (2022–2026)

- **Real-world personalisation (2022).** A portable ankle exoskeleton tuned its assistance during an hour of everyday walking using wearable sensors and a data-driven model. Compared with normal shoes, it increased self-selected walking speed by 9 ± 4% and reduced the energy used to travel a given distance by 17 ± 5%. On a treadmill at 1.5 m/s, metabolic energy fell by 23 ± 8%.
- **Task-agnostic control (2024).** A deep network estimated hip and knee biological joint moments in real time (average R² 0.83) across 28 activities, from walking and running to lifting and lunging. The exoskeleton then assisted in proportion, outperforming a task-classifier approach.
- **Soft exosuits** have shown reductions in the metabolic cost of both walking and running from one portable device.

## Open problems

- **Comfort and fit**: misalignment between device and body joints causes pressure and chafing.
- **Weight and battery**: every added kilogram costs energy to carry.
- **Safety and certification**: falls, especially for medical users.
- **Evidence and cost**: long-term clinical benefit and insurance coverage.
- **Intent detection**: knowing what the wearer is about to do (EMG, IMUs, vision, learned models).

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Exoskeletons]])
```
