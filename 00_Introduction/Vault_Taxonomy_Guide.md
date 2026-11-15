---
title: Vault Taxonomy Guide
description: The organizational structure (folders, tags, linking) of the ROBOTICS-for-PEOPLE vault.
tags:
  - vault-meta
type: guide
level: beginner
status: reviewed
author: Jordan_Smith
date: 2026-04-23
last_reviewed: 2026-04-23
related:
  - "[[How_to_Use_This_Vault]]"
  - "[[Robotics_Vault_Dashboard]]"
  - "[[Resources_Index]]"
---

# Vault Taxonomy Guide

This guide explains the organizational principles of the ROBOTICS-for-PEOPLE vault: the folder tree, the tagging vocabulary, and the linking strategy. Following these conventions keeps the graph coherent as the vault grows.

---

## Folder Structure

The vault uses a flat, numbered folder system. Numbers imply a suggested reading order for newcomers; they are not strict dependencies.

| Folder | Scope |
|---|---|
| `00_Introduction` | Vault meta: usage guide, taxonomy, dashboard, resources. |
| `01_Fundamentals_of_Robotics` | Definitions, history, ethics, industry landscape, safety. |
| `02_Mathematics_for_Robotics` | Linear algebra, calculus, probability, Lie groups SO(3)/SE(3), optimization. |
| `03_Kinematics_and_Dynamics` | Forward/inverse kinematics, DH parameters, Jacobians, Newton–Euler, Lagrangian formulations. |
| `04_Sensors_and_Perception` | Cameras, LiDAR, IMUs, encoders, sensor fusion, ICP, NDT. |
| `05_AI_and_Machine_Learning` | Classical ML, deep learning, reinforcement learning, imitation learning, VLAs (Vision-Language-Action models). |
| `06_Robot_Control` | PID, state-space, MPC (Model Predictive Control), adaptive, robust, impedance, HRI (Human-Robot Interaction). |
| `07_Robot_Operating_System_(ROS)` | ROS 2 (Humble/Iron/Jazzy), DDS, topics, services, actions, lifecycle nodes, TF, Gazebo. |
| `08_Robot_Types_and_Applications` | Industrial arms, mobile, humanoid, drones, medical, soft, swarm, exoskeletons, etc. |
| `09_Advanced_Topics` | Multi-agent, manipulation, grasping, tactile, neuromorphic, quantum, ethics deep-dive. |
| `10_Research_and_Development` | Reading papers, benchmarks, conferences (ICRA, IROS, RSS, CoRL), reproducibility. |
| `11_Practical_Implementation` | Dev environments, Docker, CI/CD, sim-to-real, rosbag2, debugging. |
| `12_Labs_and_Tutorials` | Hands-on exercises: Turtlesim, Sim2Real, URDF (Unified Robot Description Format) authoring. |
| `13_Tools_References_and_Links` | MOOCs, papers index, simulators, IDEs, hardware recommendations. |

---

## Tagging Strategy

Tags supplement folders with orthogonal metadata. A note on PID tuning sits in `06_Robot_Control`, but its tags (`#control-theory`, `#pid`, `#tuning`) connect it to every other note that cares about those dimensions.

**Specific concepts** — `#kinematics`, `#slam`, `#computer-vision`, `#reinforcement-learning`, `#sensor-fusion`, `#control-theory`, `#optimization`

**Robot types and components** — `#mobile-robot`, `#manipulator-arm`, `#humanoid`, `#lidar`, `#imu`, `#actuator`, `#camera`, `#encoder`

**Software and tools** — `#ros`, `#ros2`, `#python`, `#cpp`, `#rust`, `#cuda`, `#gazebo`, `#rviz`, `#opencv`, `#pcl`, `#eigen`

**Document type** — `#tutorial`, `#project-log`, `#research-paper`, `#course-notes`, `#concept`, `#example-code`

**Status and meta** — `#todo`, `#in-progress`, `#review-needed`, `#stub`, `#vault-meta`, `#guide`

**Rule of thumb.** Prefer reusing an existing tag over inventing a new one. Check the Obsidian Tags pane before adding a tag — if the concept is already covered under a different spelling (e.g., `#slam` vs `#SLAM`), normalize to the existing form.

**The controlled vocabulary.** Every allowed tag is listed in `tools/tags.yml`, and `tools/vault.py lint` checks each note against it. The file also records the decisions behind the list:

- **Spelling.** Lowercase kebab-case. Well-known acronyms stay acronyms, in lowercase: `ros`, `ros2`, `slam`, `pid`, `urdf`, `imu`, `mpc`.
- **Synonyms.** Each variant points to one canonical tag, e.g. `ROS2` → `ros2`, and `actuator`/`actuation` → `actuators`.
- **Dropped tags.** Tags too broad to tell notes apart are removed: `robotics` (every note is about robotics), `engineering`, `technology`, and the retired `glossary-term`.
- **Tags that were fields.** Document-type and level tags such as `tutorial`, `lab` and `beginner` now live in the `type:` and `level:` fields.
- **Bulk tags.** `mobile-robot` and `manipulator-arm` were once applied to most notes. They're kept only where the note is really about mobile robots or arms.

To add a genuinely new tag, add it to `vocabulary` in `tools/tags.yml` in the same pull request.

The legacy tag `#glossary-term` is retired. The schema v2 migration removes it.

---

## Linking Strategy

Wikilinks are what turn a collection of notes into a knowledge graph. Four guidelines keep the graph useful.

**Link on first mention.** The first time a file references "PID Control" or "SLAM", link it: `[[PID_Control]]`, `[[SLAM]]`. After that, use the term freely without re-linking.

**Link across chapters.** The deepest graph edges cross folder boundaries. A note on [[Forward_Kinematics]] (in `03_Kinematics_and_Dynamics`) should link to **Linear Algebra** (in `02_Mathematics_for_Robotics`) and to the **TF** transform library (in `07_Robot_Operating_System_(ROS)`). A note on [[Autonomous_Navigation]] (in `06_Robot_Control`) should link to [[SLAM]] (in `04_Sensors_and_Perception`) and [[Reinforcement_Learning]] (in `05_AI_and_Machine_Learning`). (The bold names above are stubs scheduled for later PRs — do not link to them until they exist.)

**Populate `related:` frontmatter.** Every note carries a `related:` list of 2–5 adjacent topics. This creates graph edges without cluttering prose. Example (for a note on inverse kinematics):

```yaml
related:
  - "[[Forward_Kinematics]]"
  - "[[Jacobian_Matrix]]"
  - "[[Degrees_of_Freedom]]"
```

**Do not link to nonexistent files.** Broken wikilinks pollute `MetadataCache.unresolvedLinks` and show as dimmed nodes in Graph View. Either create the target as a stub or drop the link.

---

## Filename Conventions

- **Case**: `Title_Case_With_Underscores.md`.
- **Acronyms in filenames**: keep uppercase (`SLAM.md`, `PID_Control.md`, `URDF.md`).
- **No spaces, no punctuation** other than underscores.
- **No duplicate filenames** anywhere in the vault — wikilinks resolve by filename, so duplicates create ambiguity.

---

## Frontmatter Schema (v2)

Every content note carries YAML frontmatter. The fields serve two readers. **People** browse by level and subfield, follow learning paths, and see which notes are reviewed. **Agents** read the same fields from `graph.json` and `llms.txt`, generated from the vault on every site build. The machine-readable version is `tools/schema.yml`.

```yaml
---
title: Inverse Kinematics
description: Finding the joint angles that put the end-effector at a desired pose.
tags:
  - kinematics
  - inverse-kinematics
type: method
level: intermediate
subfield:
  - kinematics-dynamics
  - manipulation
status: reviewed
author: Jordan_Smith
date: 2025-04-29
last_reviewed: 2026-11-15
prerequisites:
  - "[[Forward_Kinematics]]"
  - "[[Jacobian_Matrix]]"
related:
  - "[[Degrees_of_Freedom]]"
  - "[[Singularities]]"
key_takeaways:
  - IK inverts forward kinematics; it can have zero, one, several or infinitely many solutions.
  - Analytical solutions exist for some arms (e.g. a spherical wrist); otherwise use iterative Jacobian methods.
---
```

| Field | Required | Values | What it drives |
|---|---|---|---|
| `title` | yes | text | Page title |
| `description` | yes | one sentence, at most 400 characters | Search snippets, social cards, `llms.txt`, `graph.json` |
| `tags` | yes | tags from `tools/tags.yml` | Tag pages, Dataview queries |
| `type` | yes | `concept`, `method`, `component`, `robot-type`, `application`, `tool`, `tutorial`, `lab`, `overview`, `reference`, `guide`, `person` | Filters and site badges |
| `level` | yes | `beginner`, `intermediate`, `advanced`, `research` | Level filters, learning paths |
| `subfield` | yes | 1–3 keys from `tools/subfields.yml` | [[Robotics_Vault_Dashboard]], site navigation, the Subfield Atlas |
| `status` | yes | `stub`, `draft`, `reviewed`, `keystone` | The Frontier (stubs), quality badges |
| `author` | yes | `Jordan_Smith` | — |
| `date` | yes | `YYYY-MM-DD` | Created date |
| `last_reviewed` | no | `YYYY-MM-DD` | Freshness |
| `prerequisites` | no | wikilinks to existing notes, with no cycles | Learning paths: what to read first |
| `related` | no | 2–5 wikilinks | Graph edges without cluttering prose |
| `key_takeaways` | no | 3–5 short lines | The summary agents read first |
| `aliases` | no | alternative names | Obsidian link suggestions |
| `as_of` | no (expected in fast-moving subfields) | `YYYY-MM` | When the note's state-of-the-art content was last checked against current sources. Drives the site's "Current as of" badge, [[Whats_New\|What's New]] and the Frontier's "Behind the cutting edge" list |
| `sources` | at least 3 with `as_of` | `"Author or org (Year). Title. URL"` | Citations for people and agents; every URL is checked |

**Levels.** `beginner` needs no background beyond curiosity and school maths. `intermediate` assumes the notes it lists as prerequisites and is undergraduate level. `advanced` is graduate or practitioner level. `research` covers open problems and recent work.

**Statuses.** `stub` is a placeholder, listed on the Frontier page. `draft` is substantial but not yet reviewed. `reviewed` has passed a chapter review pass. `keystone` is a deep, highly linked entry written to the full template.

**Fast-moving subfields.** In `robot-learning`, `software`, `manipulation`, `legged-humanoid` and `frontiers` (marked `fast_moving` in `tools/subfields.yml`), a note is listed on the Frontier as *behind the cutting edge* until it has an `as_of` date. It is listed again when that date falls more than 12 months behind the newest one in the textbook. Bringing a note up to date means checking it against primary sources (papers, official documentation, release notes), citing them in `sources`, and setting `as_of` to the current month.

**Broken links.** Link to notes that exist. A link to a topic nobody has written yet is allowed in the body when it marks a real gap: it appears on the Frontier and shows as plain text on the website. A `related:` entry pointing at a missing note that nothing else mentions is a stray, and `vault.py trim-related --write` removes it.

**Removed in v2.** `layout`, `permalink` and `category` were Jekyll-era fields that nothing reads. `type`, `level` and `subfield` replace them.

**Author.** Always `Jordan_Smith`; do not add co-authors or AI model names.

---

## Tooling

`tools/vault.py` reads the vault exactly as Obsidian does: the same link resolution, the same tag rules, and links inside code blocks ignored. It needs Python 3.10+ and PyYAML.

```bash
python3 tools/vault.py stats            # notes, links, unresolved links, tags
python3 tools/vault.py lint --verbose   # every note checked against tools/schema.yml and tools/tags.yml
python3 tools/vault.py obsidian-check   # compare with Obsidian's own index (needs the Obsidian CLI)
python3 tools/vault.py pages --write    # rebuild Subfield_Atlas, Frontier and Learning_Paths from the notes
python3 tools/vault.py fix-links        # find links that misspell an existing note; --write repoints them
python3 tools/vault.py trim-related     # find related: links to missing notes nothing else mentions; --write removes them
```

The [[Subfield_Atlas]], [[Learning_Paths]], [[Frontier]] and [[Whats_New|What's New]] pages are generated from every note's frontmatter. After adding or editing notes, run `pages --write` and commit the result; CI fails if they are out of date.

### The website and the agent files

Every push to `main` publishes the vault as a website on GitHub Pages and generates files for AI agents: `llms.txt`, `llms-full.txt`, one text file per chapter, `graph.json` (the knowledge graph, described by `tools/graph.schema.json`), and the raw Markdown of every note. To build it locally (Node 22 and Python 3.10+):

```bash
git clone --depth 1 --branch v4.5.2 https://github.com/jackyzha0/quartz.git .quartz && (cd .quartz && npm ci)
(cd site && npm ci)                                       # the Explore page's graph libraries (Obsidian is set to ignore site/node_modules)
python3 tools/sitebuild.py prep --quartz .quartz        # copy the vault, compile Dataview, add metadata strips
(cd .quartz && npx quartz build -d content -o public --serve)
python3 tools/sitebuild.py export --out .quartz/public  # llms.txt, graph.json, raw/, explore/
```

Every citation in a note's `sources:` is checked by `python3 tools/check_sources.py`: each URL must answer, and arXiv titles must match arXiv's own record. CI runs it on every pull request.

On the site, each note opens with a strip showing its level, status, subfields and what to read first. Dataview queries are compiled to static lists and tables, and links to notes not written yet show as plain text.

`obsidian-check` uses the official [Obsidian CLI](https://help.obsidian.md/cli). It needs Obsidian 1.12+ running with this folder open as a vault, and Settings → General → Command line interface turned on. It checks that the tooling and Obsidian agree on every unresolved link and tag.

Every pull request runs the same lint in GitHub Actions. One rule always applies: **the number of unresolved links may only go down.** The baseline is in `tools/baseline.json`.

---

*This guide is itself part of the vault-meta layer. Keep it in sync whenever folder names change or new tagging conventions emerge.*
