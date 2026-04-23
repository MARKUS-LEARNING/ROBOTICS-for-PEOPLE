---
title: Robotics Vault Dashboard
description: A Dataview-powered dynamic index of every note in the vault, grouped by chapter.
tags:
  - dashboard
  - vault-meta
layout: default
category: robotics
author: Jordan_Smith
date: 2026-04-23
permalink: /robotics-vault-dashboard/
related:
  - "[[How_to_Use_This_Vault]]"
  - "[[Vault_Taxonomy_Guide]]"
  - "[[Resources_Index]]"
---

# Robotics Vault Dashboard

This dashboard is a live, queryable index of every note in the vault. It uses the [Dataview](https://blacksmithgu.github.io/obsidian-dataview/) community plugin — install and enable it in Obsidian for the tables below to render. On GitHub the queries appear as code blocks, which is fine.

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

---

## 00 — Introduction

```dataview
TABLE title, tags
FROM "00_Introduction"
SORT file.name ASC
```

---

## 01 — Fundamentals of Robotics

```dataview
TABLE title, tags
FROM "01_Fundamentals_of_Robotics"
SORT file.name ASC
```

---

## 02 — Mathematics for Robotics

```dataview
TABLE title, tags
FROM "02_Mathematics_for_Robotics"
SORT file.name ASC
```

---

## 03 — Kinematics and Dynamics

```dataview
TABLE title, tags
FROM "03_Kinematics_and_Dynamics"
SORT file.name ASC
```

---

## 04 — Sensors and Perception

```dataview
TABLE title, tags
FROM "04_Sensors_and_Perception"
SORT file.name ASC
```

---

## 05 — AI and Machine Learning

```dataview
TABLE title, tags
FROM "05_AI_and_Machine_Learning"
SORT file.name ASC
```

---

## 06 — Robot Control

```dataview
TABLE title, tags
FROM "06_Robot_Control"
SORT file.name ASC
```

---

## 07 — Robot Operating System (ROS)

```dataview
TABLE title, tags
FROM "07_Robot_Operating_System_(ROS)"
SORT file.name ASC
```

---

## 08 — Robot Types and Applications

```dataview
TABLE title, tags
FROM "08_Robot_Types_and_Applications"
SORT file.name ASC
```

---

## 09 — Advanced Topics

```dataview
TABLE title, tags
FROM "09_Advanced_Topics"
SORT file.name ASC
```

---

## 10 — Research and Development

```dataview
TABLE title, tags
FROM "10_Research_and_Development"
SORT file.name ASC
```

---

## 11 — Practical Implementation

```dataview
TABLE title, tags
FROM "11_Practical_Implementation"
SORT file.name ASC
```

---

## 12 — Labs and Tutorials

```dataview
TABLE title, tags
FROM "12_Labs_and_Tutorials"
SORT file.name ASC
```

---

## 13 — Tools, References, and Links

```dataview
TABLE title, tags
FROM "13_Tools_References_and_Links"
SORT file.name ASC
```

---

## Maintenance Views

### Notes flagged `#stub` (need expansion)

```dataview
LIST
FROM #stub
SORT file.name ASC
```

### Notes flagged `#review-needed`

```dataview
LIST
FROM #review-needed
SORT file.name ASC
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
