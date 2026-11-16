---
title: Medical Robotics
description: "Robots in medicine: surgical systems that extend a surgeon's hands, rehabilitation and assistive devices, hospital logistics and laboratory automation. Covers how teleoperated surgical robots work (motion scaling and tremor filtering, computed), the 2025 market, levels of autonomy in surgery including the 2025 autonomous gallbladder-removal study, regulation, and open problems."
aliases:
  - Medical Robots
  - Surgical Robots
  - Surgical Robotics
tags:
  - medical-robots
  - surgery
  - teleoperation
  - human-robot-interaction
  - safety
type: robot-type
level: beginner
subfield:
  - medical-robotics
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Robots]]"
  - "[[Manipulator_Arm]]"
related:
  - "[[Exoskeletons]]"
  - "[[Autonomous_Robots]]"
  - "[[Imitation_Learning]]"
  - "[[Ethical_and_Social_Implications]]"
key_takeaways:
  - "Most surgical robots are teleoperated: the surgeon controls every motion, and the robot adds precision through motion scaling, tremor filtering, wristed instruments and 3D vision."
  - "Robotic surgery is mainstream: Intuitive reported 11,106 da Vinci systems installed at the end of 2025 and about 18% growth in da Vinci procedures that year."
  - "Autonomy is arriving step by step: in 2025 a learned system (SRT-H) performed the clipping-and-cutting phase of gallbladder removal autonomously on ex vivo pig tissue; clinical autonomy remains years away and heavily regulated."
sources:
  - "Intuitive Surgical (2026). Intuitive Announces Preliminary Fourth Quarter and Full Year 2025 Results. https://www.biospace.com/press-releases/intuitive-announces-preliminary-fourth-quarter-and-full-year-2025-results"
  - "Kim, J. W. et al. (2025). SRT-H: A hierarchical framework for autonomous surgery via language-conditioned imitation learning. https://www.science.org/doi/10.1126/scirobotics.adt5254"
  - "Yang, G.-Z. et al. (2017). Medical robotics—Regulatory, ethical, and legal considerations for increasing levels of autonomy. https://www.science.org/doi/10.1126/scirobotics.aam8638"
  - "US Food and Drug Administration (2026). Computer-Assisted Surgical Systems. https://www.fda.gov/medical-devices/surgery-devices/computer-assisted-surgical-systems"
  - "Siciliano, B. and Khatib, O., eds. (2016). Springer Handbook of Robotics, 2nd edition (chapters on medical robotics). https://link.springer.com/referencework/10.1007/978-3-319-32552-1"
  - "International Federation of Robotics (2026). Global Sales of Professional Service Robots Surge 24%. https://ifr.org/ifr-press-releases/news/global-sales-of-professional-service-robots-surge-24-percent"
---

# Medical Robotics

**Medical robots** help diagnose, treat and care for people: surgical systems that let a surgeon operate through small incisions with greater precision; rehabilitation and assistive devices; robots that deliver medicines and supplies inside hospitals; and laboratory robots that run tests around the clock.

> **Etymology.** Latin *medicus*, "physician". The first robot-assisted surgical procedure is usually dated to 1985, when a PUMA 560 industrial arm positioned a needle for a brain biopsy. Intuitive's da Vinci system received US clearance for general laparoscopic surgery in 2000.

## Kinds of medical robot

| Kind | What it does | Examples |
|---|---|---|
| **Surgical (soft tissue)** | Surgeon-controlled instruments through small incisions | da Vinci (Intuitive), Hugo (Medtronic), Versius (CMR), Toumai and other Chinese systems |
| **Surgical (orthopaedic, neuro)** | Robot positions tools to a pre-operative plan | Knee and hip replacement and spine systems |
| **Endoluminal and catheter** | Steer flexible instruments inside airways or vessels | Ion (lung biopsy) |
| **Rehabilitation** | Guide and assist movement therapy | Upper-limb rehab robots, gait trainers |
| **Assistive** | Support daily living | Powered [[Exoskeletons]], feeding and wheelchair-mounted arms |
| **Hospital logistics and disinfection** | Deliver supplies; UV disinfection | Autonomous mobile robots in hospitals |
| **Laboratory automation** | Pipetting, sample handling | Automated clinical labs |

## How a teleoperated surgical robot helps

The surgeon sits at a console, looks into a 3D high-definition view, and moves hand controllers. The robot reproduces those motions with instruments inside the patient. Two features show why it helps:

**Motion scaling, computed.** At a 5:1 scale, a 15 mm hand movement becomes a 15/5 = **3 mm** instrument movement, so fine dissection uses comfortable, larger hand motions.

**Tremor filtering.** Physiological hand tremor is a small oscillation at about 8–12 Hz. Low-pass filtering removes most of it, and scaling shrinks what remains: a 0.1 mm tremor becomes 0.1/5 = **0.02 mm** at the instrument.

Wristed instruments add degrees of freedom that straight laparoscopic tools lack. Because the surgeon controls every motion, these systems are regulated as surgeon-controlled tools, not autonomous devices.

## The market (end of 2025)

From Intuitive's preliminary full-year 2025 results:
- **11,106** da Vinci systems installed worldwide, up 12% from 9,902;
- da Vinci procedures up about **18%**;
- 870 of the 1,721 systems placed in 2025 were the newest da Vinci 5.

Competitors from Medtronic, CMR, J&J and several Chinese companies are expanding the market. Across all makers, the International Federation of Robotics counts 8,400 surgical robots sold in 2025, up 57%, the main driver of growth in medical robotics.

## Autonomy in surgery

Researchers grade surgical autonomy from 0 (no autonomy) to 5 (full autonomy). Clinical systems today sit at 1–2: assistance and task-level automation such as camera control or fixed tool paths in orthopaedics.

In 2025 the **SRT-H** system (Johns Hopkins and Stanford, *Science Robotics*) used a hierarchical, language-conditioned policy trained by [[Imitation_Learning|imitation learning]] on surgical video and kinematics. It autonomously performed the clipping-and-cutting phase of a cholecystectomy (gallbladder removal), 17 steps in all, on ex vivo pig tissue, succeeding in all 8 trials. It was slower than a human surgeon and accepted spoken corrections. This is a milestone for learned surgical autonomy, but it is not yet clinical: tissue was not living, and the trials were few.

## Rules and safety

Surgical robots are regulated medical devices (in the US, FDA clearance; in the EU, the Medical Device Regulation). Higher autonomy raises open questions of validation, liability and informed consent. Under the EU AI Act, AI that is a safety component of a medical device is "high-risk" (see [[Ethical_and_Social_Implications]]).

## Open problems

- Haptic (force) feedback to the surgeon, which most systems still lack.
- Cost and access outside wealthy hospitals.
- Training data and evaluation for autonomous surgical skills.
- Soft-tissue modelling for planning and simulation.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Medical_Robotics]])
```
