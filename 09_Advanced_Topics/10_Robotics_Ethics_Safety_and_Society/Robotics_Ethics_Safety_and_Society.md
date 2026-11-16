---
title: Robotics Ethics, Safety and Society
description: "Section overview for the social side of robotics: safety engineering and risk assessment (with a computed risk-matrix example), semantic and AI safety for robots that follow language instructions, labour and economic effects, privacy, accountability and regulation in 2026. Points to the detailed notes in this textbook."
aliases:
  - Robot Ethics
  - Robotics and Society
  - Robot Safety
tags:
  - ethics
  - safety
  - regulation
type: overview
level: intermediate
subfield:
  - ethics-society
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Ethical_and_Social_Implications]]"
related:
  - "[[Collaborative_Robots]]"
  - "[[Formal_Methods_in_Control]]"
  - "[[Humanoid_Robots]]"
  - "[[Autonomous_Robots]]"
key_takeaways:
  - "Safety engineering starts with risk assessment: identify hazards, estimate severity and likelihood, reduce risk by design first, then by safeguards, then by information."
  - "Robots that take language instructions add semantic safety: knowing that 'put the knife in the toddler's hand' is unsafe even if physically possible; 2025 work generated robot 'constitutions' and benchmarks for it."
  - "Society-level questions (jobs, privacy, accountability, access) are shaped by regulation that in 2026 is still being phased in; builders should design for it now."
sources:
  - "Sermanet, P. et al. (2025). Generating Robot Constitutions & Benchmarks for Semantic Safety. https://arxiv.org/abs/2503.08663"
  - "European Union (2024). Regulation (EU) 2024/1689: the Artificial Intelligence Act. https://eur-lex.europa.eu/eli/reg/2024/1689/oj"
  - "European Union (2023). Regulation (EU) 2023/1230 on machinery products. https://eur-lex.europa.eu/eli/reg/2023/1230/oj"
  - "International Federation of Robotics (2026). Five Million Robots now Operate in Factories Globally (World Robotics 2026). https://ifr.org/ifr-press-releases/news/five-million-robots-now-operate-in-factories-globally"
---

# Robotics Ethics, Safety and Society

This section gathers the human questions about robots: how to make them safe, how to decide what they should and should not do, and how they change work and life. The core discussion, including 2026 regulation, is in [[Ethical_and_Social_Implications]]; this overview adds safety-engineering method and the newest concerns.

> **Etymology.** See [[Ethical_and_Social_Implications]].

## Safety engineering: risk assessment

Machinery safety follows a standard sequence:
1. identify hazards (crushing, impact, falling objects, electrical, software errors);
2. estimate the **risk** of each, typically as severity × probability;
3. reduce it in priority order: **inherently safe design**, then **safeguards** (sensing, stops, guards), then **information** (warnings, training);
4. check what risk remains and document it.

**Worked example.** On a 1–5 scale for severity and probability, a mobile robot's hazards might score:
- running over a foot: severity 2 × probability 3 = **6**;
- collision with a person at full speed: 4 × 2 = **8**;
- tipping over on a ramp while carrying a load: 4 × 1 = **4**.

Reducing speed near people lowers the collision probability to 1 (score 4), and a lower centre of mass lowers tipping probability further. Real assessments follow standards (ISO 12100, and robot-specific ones such as ISO 10218 and ISO 13482) with more detailed scoring, but the logic is the same.

## Semantic safety for instruction-following robots

Physical safety layers prevent dangerous *forces*. Robots driven by language and vision-language models also need **semantic** safety: understanding that some requests are dangerous or inappropriate even when physically feasible, such as handing a knife blade-first or putting metal in a microwave. In 2025, researchers generated robot "constitutions" (rules derived from data and refined by language models) and a benchmark (ASIMOV) for this kind of judgement. Semantic safety is now an active research area alongside classical functional safety.

## Society

- **Work**: around 600,000 industrial robots were installed in 2025 alone (IFR); automation is spreading to logistics, services and, through humanoids, potentially to many manual jobs. The effects depend heavily on policy, training and how robots are deployed.
- **Privacy**: home and workplace robots are mobile sensor platforms (see [[Ethical_and_Social_Implications]]).
- **Accountability and liability** for learned behaviour.
- **Access and fairness**: who benefits, and who bears the risks.
- **Military use**: autonomous weapons and dual-use concerns.

## Regulation (2026)

EU AI Act obligations for high-risk AI phase in during 2027–2028; the EU Machinery Regulation applies from January 2027; the robot-safety standards were revised in 2025 (see [[Collaborative_Robots]]). Designing for these now is cheaper than retrofitting.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Robotics_Ethics_Safety_and_Society]])
```
