---
title: Ethical and Social Implications
description: "The ethical and social questions raised by AI-driven robots: physical safety of learned behaviour, privacy of always-on sensors, bias, work and jobs, accountability, dual use and autonomous weapons, and environmental cost. Covers the 2025–2026 rules that apply (EU AI Act and its 2026 amendments, EU Machinery Regulation, ISO 10218:2025, NIST AI RMF) and practical habits for people who build robots."
aliases:
  - Robot Ethics
  - AI Ethics
  - Roboethics
tags:
  - ethics
  - safety
  - regulation
  - artificial-intelligence
type: overview
level: beginner
subfield:
  - ethics-society
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Artificial_Intelligence]]"
related:
  - "[[Embodied_AI]]"
  - "[[Vision-Language-Action_Models]]"
  - "[[Industrial_Arms]]"
  - "[[Humanoid_Robots]]"
key_takeaways:
  - "Robots turn AI errors into physical events, so safety (and proving it) is the first ethical obligation; learned policies make that harder because their failures are hard to predict."
  - "Rules are arriving: the EU AI Act (amended in 2026 to delay high-risk obligations to December 2027 and, for AI inside regulated products such as machinery, August 2028), the EU Machinery Regulation from January 2027, and the revised ISO 10218 robot-safety standards (2025)."
  - "Beyond safety: privacy of cameras and microphones in homes and workplaces, fair performance across people, effects on work, clear accountability, and refusing harmful uses."
sources:
  - "European Union (2024). Regulation (EU) 2024/1689: the Artificial Intelligence Act. https://eur-lex.europa.eu/eli/reg/2024/1689/oj"
  - "Addleshaw Goddard (2026). EU AI Act: AI Omnibus formally adopted. https://www.addleshawgoddard.com/en/insights/insights-briefings/2026/technology/eu-ai-act-ai-omnibus-formally-adopted/"
  - "European Union (2023). Regulation (EU) 2023/1230 on machinery products. https://eur-lex.europa.eu/eli/reg/2023/1230/oj"
  - "ISO (2025). ISO 10218-1:2025 Robotics — Safety requirements — Part 1: Industrial robots. https://www.iso.org/standard/73933.html"
  - "NIST (2023). AI Risk Management Framework (AI RMF 1.0). https://www.nist.gov/itl/ai-risk-management-framework"
  - "Bommasani, R. et al. (2021). On the Opportunities and Risks of Foundation Models. https://arxiv.org/abs/2108.07258"
---

# Ethical and Social Implications

AI that runs on a robot acts in the physical world. A chatbot's mistake is a wrong sentence; a robot's mistake can be a broken object, an injured person or a leaked video of someone's home. The ethics of robotics is therefore practical: what to build, how to make it safe, whose data it uses, who answers when it fails.

> **Etymology.** *Ethics* is from Greek *ēthikos*, "of character", from *ēthos*, "custom, character". *Roboethics* was coined by Gianmarco Veruggio in 2002 for the ethics of the people who design and use robots. Asimov's fictional Three Laws (1942) started the public conversation, but they are a plot device, not an engineering method.

## The main issues

| Issue | Why robots make it sharper | What builders do |
|---|---|---|
| **Physical safety** | Learned policies can behave unpredictably outside their training data; humanoids and mobile manipulators work near people | Safety-rated stops, speed and force limits, independent monitors under the AI, testing on held-out conditions, staged deployment |
| **Privacy** | Cameras, depth sensors and microphones in homes, hospitals and workplaces; recorded data used for training | Minimise and blur, on-device processing, clear consent, retention limits, opt-outs for training use |
| **Bias and fairness** | Perception can work worse for some skin tones, body shapes, accents or mobility aids | Evaluate per group, diversify data, design for accessibility |
| **Work and jobs** | Automation moves from factories into logistics, retail, care and homes | Plan for transition and retraining; design robots to assist workers |
| **Accountability** | When a learned system fails, it is unclear whether the cause was the data, the model, the integrator or the operator | Logging (black-box recorders), documented data and models, clear responsibility in contracts and regulation |
| **Dual use and weapons** | Navigation, perception and autonomy transfer to weapons; debate on lethal autonomous weapons continues at the UN | Use policies, export controls, refusing harmful applications |
| **Environmental cost** | Training large models and building hardware use energy and materials | Efficient models, repairable hardware, measuring the footprint |

## Rules that apply (as of November 2026)

- **EU Artificial Intelligence Act** (Regulation 2024/1689). In force since August 2024, with prohibitions from February 2025 and general-purpose-model duties from August 2025.
  - A "Digital Omnibus" amendment, adopted in mid-2026, postponed the high-risk obligations: to **2 December 2027** for stand-alone high-risk systems, and to **2 August 2028** for AI built into products already regulated by EU product law, which includes AI safety components of robots and machinery.
  - Check the Official Journal for the consolidated text.
- **EU Machinery Regulation** (2023/1230). Replaces the Machinery Directive from **20 January 2027** and explicitly covers machines whose behaviour is shaped by self-learning software.
- **ISO 10218-1 and -2:2025.** The revised industrial-robot safety standards, published February 2025, integrate collaborative-robot requirements. ISO 13482 covers personal care robots.
- **NIST AI Risk Management Framework** (US, 2023). A voluntary framework for governing, mapping, measuring and managing AI risk, widely used by companies.

Regulation is changing quickly. Treat this list as a starting point and check the current texts.

## Learned policies raise new questions

[[Vision-Language-Action_Models|Robot foundation models]] can be told what to do in plain language. That raises questions classical industrial robots never did:
- Who may instruct the robot?
- How does it refuse unsafe or harmful instructions?
- How do you certify a system whose behaviour depends on billions of learned parameters?

Current practice keeps learned components *above* certified safety layers, so the AI decides what to do but cannot override force limits or emergency stops. Researchers are developing physical-safety benchmarks and formal "safety filters" that check commands before execution.

## Habits for people who build robots

1. Write down what the robot must never do, and enforce it outside the learned model.
2. Test where it will fail: new people, new objects, bad lighting, edge cases.
3. Collect only the data you need; tell people what you record and why.
4. Log enough to explain incidents.
5. Ask who benefits and who bears the risk, before deployment rather than after.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Ethical_and_Social_Implications]])
```
