---
title: Robotics Vault Dashboard
description: A Dataview-powered dynamic index of every note in the vault, grouped by chapter.
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
  - "[[Vault_Taxonomy_Guide]]"
  - "[[Resources_Index]]"
---

# Robotics Vault Dashboard

This dashboard is a live, queryable index of every note in the vault. It uses the [Dataview](https://blacksmithgu.github.io/obsidian-dataview/) community plugin — install and enable it in Obsidian for the tables below to render. On GitHub the queries appear as code blocks, which is fine. Three pages are generated from the same metadata and read well anywhere, GitHub included: the [[Subfield_Atlas]] (every subfield and its notes), the [[Learning_Paths]] (reading orders built from prerequisites) and the [[Frontier]] (what to write next).

---

## Vault-wide Snapshot

Total notes, by chapter:

```dataview
TABLE WITHOUT ID file.folder AS "Chapter", length(rows) AS "Notes"
FROM ""
WHERE file.folder != ""
GROUP BY file.folder
SORT file.folder ASC
```

By level, status and type (schema v2 frontmatter, see [[Vault_Taxonomy_Guide]]):

```dataview
TABLE WITHOUT ID level AS "Level", length(rows) AS "Notes"
FROM ""
WHERE level AND type != "guide"
GROUP BY level
```

```dataview
TABLE WITHOUT ID status AS "Status", length(rows) AS "Notes"
FROM ""
WHERE status AND type != "guide"
GROUP BY status
```

```dataview
TABLE WITHOUT ID type AS "Type", length(rows) AS "Notes"
FROM ""
WHERE type
GROUP BY type
SORT length(rows) DESC
```

By subfield (a note can belong to up to three):

```dataview
TABLE WITHOUT ID sf AS "Subfield", length(rows) AS "Notes"
FROM ""
FLATTEN subfield AS sf
WHERE sf
GROUP BY sf
SORT length(rows) DESC
```

---

## Keystones

The deepest, most linked entries, each rewritten to the full template.

```dataview
TABLE level, subfield, last_reviewed AS "Reviewed"
FROM ""
WHERE status = "keystone"
SORT file.folder ASC, file.name ASC
```

---

## 00 — Introduction

```dataview
TABLE title, type, level, status
FROM "00_Introduction"
SORT file.name ASC
```

---

## 01 — Fundamentals of Robotics

```dataview
TABLE title, type, level, status
FROM "01_Fundamentals_of_Robotics"
SORT file.name ASC
```

---

## 02 — Mathematics for Robotics

```dataview
TABLE title, type, level, status
FROM "02_Mathematics_for_Robotics"
SORT file.name ASC
```

---

## 03 — Kinematics and Dynamics

```dataview
TABLE title, type, level, status
FROM "03_Kinematics_and_Dynamics"
SORT file.name ASC
```

---

## 04 — Sensors and Perception

```dataview
TABLE title, type, level, status
FROM "04_Sensors_and_Perception"
SORT file.name ASC
```

---

## 05 — AI and Machine Learning

```dataview
TABLE title, type, level, status
FROM "05_AI_and_Machine_Learning"
SORT file.name ASC
```

---

## 06 — Robot Control

```dataview
TABLE title, type, level, status
FROM "06_Robot_Control"
SORT file.name ASC
```

---

## 07 — Robot Operating System (ROS)

```dataview
TABLE title, type, level, status
FROM "07_Robot_Operating_System_(ROS)"
SORT file.name ASC
```

---

## 08 — Robot Types and Applications

```dataview
TABLE title, type, level, status
FROM "08_Robot_Types_and_Applications"
SORT file.name ASC
```

---

## 09 — Advanced Topics

```dataview
TABLE title, type, level, status
FROM "09_Advanced_Topics"
SORT file.name ASC
```

---

## 10 — Research and Development

```dataview
TABLE title, type, level, status
FROM "10_Research_and_Development"
SORT file.name ASC
```

---

## 11 — Practical Implementation

```dataview
TABLE title, type, level, status
FROM "11_Practical_Implementation"
SORT file.name ASC
```

---

## 12 — Labs and Tutorials

```dataview
TABLE title, type, level, status
FROM "12_Labs_and_Tutorials"
SORT file.name ASC
```

---

## 13 — Tools, References, and Links

```dataview
TABLE title, type, level, status
FROM "13_Tools_References_and_Links"
SORT file.name ASC
```

---

## Maintenance Views

### Stubs (need writing)

```dataview
LIST description
FROM ""
WHERE status = "stub"
SORT file.name ASC
```

### Drafts awaiting a review pass, by chapter

```dataview
TABLE WITHOUT ID split(file.folder, "/")[0] AS "Chapter", length(rows) AS "Drafts"
FROM ""
WHERE status = "draft"
GROUP BY split(file.folder, "/")[0]
SORT split(file.folder, "/")[0] ASC
```

### Least recently reviewed

```dataview
TABLE last_reviewed AS "Reviewed", status
FROM ""
WHERE last_reviewed
SORT last_reviewed ASC
LIMIT 15
```

### Orphans — notes nothing links to

```dataview
LIST
FROM ""
WHERE length(file.inlinks) = 0
SORT file.name ASC
```

### Unresolved links — wikilinks to files that do not exist

```dataview
LIST file.outlinks
FROM ""
WHERE length(filter(file.outlinks, (l) => !l.file)) > 0
SORT file.name ASC
```

---

✅ If any table above is empty or broken, confirm the [Dataview plugin](https://obsidian.md/plugins?id=dataview) is installed and enabled.
