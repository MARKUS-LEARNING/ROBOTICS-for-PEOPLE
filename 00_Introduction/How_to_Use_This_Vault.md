---
title: How to Use This Vault
description: A guide to navigating, contributing to, and enhancing the ROBOTICS-for-PEOPLE Obsidian vault.
tags:
  - guide
  - vault-meta
  - usage
  - help
layout: default
category: robotics
author: Jordan_Smith
date: 2026-04-23
permalink: /how_to_use_this_vault/
related:
  - "[[Vault_Taxonomy_Guide]]"
  - "[[Robotics_Vault_Dashboard]]"
  - "[[Resources_Index]]"
---

# How to Use This Vault

Welcome to **ROBOTICS-for-PEOPLE** — an open, markdown-first textbook for learning robotics from first principles. The entire vault is designed to be read and edited inside [Obsidian](https://obsidian.md), but every file is plain Markdown, so it renders fine on GitHub too.

This guide explains how to navigate the folder structure, how to add content without breaking the link graph, and which optional plugins unlock the best experience.

---

## Section 1: Philosophy

This vault is built on three commitments:

1. **First principles before formulas.** Every concept is introduced by explaining the physical or mathematical reality it models before any equation or code appears.
2. **Acronym hygiene.** Every acronym — ROS (Robot Operating System), SLAM (Simultaneous Localization and Mapping), EKF (Extended Kalman Filter), MPC (Model Predictive Control) — is expanded on first use in each file.
3. **Connectivity over completeness.** A half-linked note beats an isolated polished one. The value of the vault scales with how densely its ideas are interconnected through wikilinks, tags, and `related:` frontmatter.

---

## Section 2: Folder Structure

The vault uses a flat, numbered folder system. Numbers express a suggested reading order, not a strict dependency.

| Folder | Topic |
|---|---|
| `00_Introduction` | Meta: how to use the vault, taxonomy, dashboard, resource index. |
| `01_Fundamentals_of_Robotics` | Definitions, history, ethics, the robotics industry. |
| `02_Mathematics_for_Robotics` | Linear algebra, calculus, probability, Lie groups, optimization. |
| `03_Kinematics_and_Dynamics` | Forward/inverse kinematics, Jacobians, Newton–Euler, Lagrangian dynamics. |
| `04_Sensors_and_Perception` | Cameras, LiDAR, IMUs, encoders, sensor fusion, point clouds. |
| `05_AI_and_Machine_Learning` | Classical ML, deep learning, reinforcement learning, imitation learning, VLAs. |
| `06_Robot_Control` | PID, state-space, MPC, adaptive, robust, impedance control, HRI. |
| `07_Robot_Operating_System_(ROS)` | ROS 2 concepts — DDS (Data Distribution Service), topics, services, actions, TF, Gazebo. |
| `08_Robot_Types_and_Applications` | Industrial arms, mobile, humanoid, drones, medical, soft, swarm, etc. |
| `09_Advanced_Topics` | Multi-agent, manipulation, tactile, neuromorphic, quantum, ethics deep-dive. |
| `10_Research_and_Development` | Reading papers, benchmarks, conferences (ICRA, IROS, RSS, CoRL), reproducibility. |
| `11_Practical_Implementation` | Dev environments, Docker, CI/CD, sim-to-real, rosbag2. |
| `12_Labs_and_Tutorials` | Hands-on exercises: Turtlesim, Sim2Real, URDF authoring. |
| `13_Tools_References_and_Links` | MOOCs, papers index, simulators, IDEs, hardware. |

For a deeper explanation, see [[Vault_Taxonomy_Guide]].

---

## Section 3: Navigating with the Dashboard

[[Robotics_Vault_Dashboard]] provides dynamic indexes of notes by chapter using the Dataview plugin.

**Requirement:** The [Dataview](https://blacksmithgu.github.io/obsidian-dataview/) community plugin.
- Install: Obsidian `Settings` → `Community Plugins` → `Browse` → search "Dataview" → Install → Enable.
- The dashboard degrades gracefully on GitHub (queries render as code blocks, not tables), so it is safe to commit.

---

## Section 4: Adding and Editing Content

Consistency is what keeps the graph useful. When creating or modifying notes, follow this checklist.

**Placement.** Put the note in the folder matching its topic. A note on Kalman filters belongs in `04_Sensors_and_Perception`; a note on PID tuning belongs in `06_Robot_Control`.

**Filename.** Use `Title_Case_With_Underscores.md`. Examples: `Forward_Kinematics.md`, `Extended_Kalman_Filter.md`. This makes wikilinks predictable.

**Frontmatter.** Every note must start with a YAML frontmatter block. Copy from an existing note and update the relevant fields:

```yaml
---
title: Extended Kalman Filter
description: Nonlinear recursive state estimator used in robot localization.
tags:
  - sensor-fusion
  - estimation
  - slam
category: robotics
author: Jordan_Smith
date: 2026-04-23
related:
  - "[[Kalman_Filter]]"
  - "[[Sensor_Fusion]]"
  - "[[SLAM]]"
---
```

**Authorship.** All contributions are attributed to `Jordan_Smith`. Do not add co-authors or AI model names to the `author` field.

**Content.** Explanations should lead with physical intuition, then state the mathematical model, then show code or pseudocode. Use LaTeX (`$...$` inline, `$$...$$` block) for equations. Use fenced code blocks with a language tag for code (` ```python `, ` ```cpp `, ` ```bash `).

**Linking.** Use `[[Internal Link]]` generously — but only once per section per target file. The `related:` frontmatter field should list 2–5 genuinely related notes; these become graph edges without cluttering prose. See Section 6 below.

---

## Section 5: Enhancing Your Experience (Optional Plugins)

- **[Canvas](https://help.obsidian.md/Canvas)** — visual thinking, system diagrams, mind maps of related concepts.
- **[Excalidraw](https://github.com/zsviczian/obsidian-excalidraw-plugin)** — sketch kinematic chains, control loops, state machines directly inside notes.
- **DataviewJS** — advanced queries with JavaScript; useful for custom indexes.
- **Graph Analysis** — surface orphan notes, betweenness centrality, and recommended new links.

---

## Section 6: Maximizing Graph Connectivity

The Graph View in Obsidian is powered by the `MetadataCache.resolvedLinks` table (`Record<source, Record<destination, count>>`). Every wikilink, `related:` entry, and embed adds an edge to that graph. Four rules keep edges meaningful:

1. **Link on first mention.** The first time a file mentions "PID Control" or "SLAM", link it: `[[PID_Control]]`, `[[SLAM]]`. After that, use the term freely without re-linking.
2. **Populate `related:` frontmatter.** 2–5 adjacent topics per note. Avoid dumping every loosely related file.
3. **No broken links.** If a wikilink target does not exist, either create it as a stub or drop the link. Broken links appear as dimmed nodes in Graph View and pollute `MetadataCache.unresolvedLinks`.
4. **Use aliases for prose flow.** `[[Open_Loop_vs_Closed_Loop|open-loop control]]` reads naturally while still creating the graph edge.

---

## Section 7: Contributing

This is a public vault. Issues and pull requests are welcome at [github.com/MARKUS-LEARNING/ROBOTICS-for-PEOPLE](https://github.com/MARKUS-LEARNING/ROBOTICS-for-PEOPLE). The short version of the contribution workflow:

```bash
git pull                         # always pull first
# edit notes in your local Obsidian
git add <specific files>
git commit -m "Describe what changed, not what file changed"
git push
```

See `CONTRIBUTING.md` at the repo root for detail.

---

*This vault is a living document. Adapt it, expand it, and make it your own — that is the point.*
