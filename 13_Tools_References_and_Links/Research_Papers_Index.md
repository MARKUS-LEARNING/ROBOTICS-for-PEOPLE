---
title: Research Papers Index
description: An index and organizational guide for key robotics research papers.
tags:
  - index
  - research
  - papers
  - reading-list
  - references
  - bibliography
  - literature
layout: default
category: robotics
author: Jordan_Smith_&_le_Chat
date: 2025-04-29
permalink: /research_papers_index/
related:
  - "[[Resources_Index]]"
  - "[[Reading List]]"
  - "[[Dataview]]"
---

# Research Papers Index

This note serves as a central index and organizational structure for tracking and accessing important research papers relevant to robotics that you read, reference, or plan to read. Keeping track of key literature is crucial for understanding advancements, learning techniques, and conducting your own research or development.

See also: [[Resources_Index]], [[Tools_References_and_Links]], [[Reading_List]]

---

## Organization Strategy

It's recommended to create individual notes for each significant research paper you engage with. You can organize these notes within a subfolder (e.g., `10_Tools_References_and_Links/Research_Papers/`) or keep them distributed and rely on linking and tagging.

* **Linking:** Link individual paper notes back to this index page (`[[Research_Papers_Index]]`) and also link them from relevant topic notes within your vault (e.g., link a paper on SLAM from the [[SLAM]] note).
* **Tagging:** Use tags consistently for paper notes, such as `#research-paper` and specific topic tags like `#SLAM`, `#reinforcement-learning`, `#humanoid`, `#computer-vision`, `#ICRA2024`, etc.

---

## Information to Track (Suggested Template for Paper Notes)

For each paper note (e.g., `[[Levine DRL 2016 Notes]]`), consider including the following information, potentially using frontmatter for key fields:

```markdown
---
title: "[Paper Title]"
authors: [Author 1, Author 2, ...]
venue: "[Conference/Journal Name, Year]" # e.g., ICRA 2016, Science Robotics 2023
tags: [research-paper, topic1, topic2] # e.g., #research-paper, #RL, #manipulation
link: "[https://info.arxiv.org/help/submit_pdf.html](https://info.arxiv.org/help/submit_pdf.html)"
status: "[e.g., To Read, Reading, Read, Summarized]"
date_read: YYYY-MM-DD
rating: [1-5 stars, optional]
related: ["[[Research_Papers_Index]]", "[[Related_Concept_Note]]"]
---

# [Paper Title]

* **Authors:** [List Authors]
* **Venue:** [Conference/Journal, Year]
* **Link:** [URL]([URL])
* **Keywords:** [Relevant keywords]

## Summary / Abstract

* [Paste abstract or write your own brief summary]

## Key Contributions / Takeaways

* [Bullet point list of the main innovations or findings]
* * ## Methodology

* [Brief description of the technical approach, algorithms, experiments]

## Related Vault Notes

* [[Concept 1]]
* [[Related Algorithm Note]]

## Personal Notes / Critique

* [Your thoughts, questions, connections to other work, criticisms]

## Citation (BibTeX Example)

```bibtex
@inproceedings{key,
  author    = {},
  title     = {},
  booktitle = {},
  year      = {},
  pages     = {},
  doi       = {},
}
```
```


---

## Key Papers Index (Examples)

This section can serve as a manually curated index linking to your notes on specific papers, or you can use a Dataview query (see below) to list them dynamically. Here are examples based on documents provided earlier:

* **AI / Reinforcement Learning:**
    * [[Human-Level Control Through Deep Reinforcement Learning]] (Mnih et al., 2015 - DQN)
    * [[Deep Reinforcement Learning for Robotic Manipulation]] (Levine et al., 2016)
    * [[Learning Synergies Between Pushing and Grasping with Self-Supervised Deep Reinforcement Learning]] (Zeng et al., 2018)
    * [[Fleet Learning A Framework]] (Google Robotics, 2020)
    * [[Attention Is All You Need]] (Vaswani et al., 2017 - Transformers)
* **SLAM / Navigation:**
    * [[ORB-SLAM A Versatile and Accurate Monocular SLAM System]] (Mur-Artal et al., 2015)
    * [[SLAM Survey 2022]] *(Placeholder based on Resources Index)*
* **ROS / Systems:**
    * [[ROS An Open-Source Robot Operating System]] (Quigley et al., 2009)
    * [[A Survey of Research on Cloud Robotics and Automation]] (Kehoe et al., 2013)
* **Soft Robotics:**
    * [[Soft Robotics Current Trends and Prospects]] *(Placeholder based on 10 Papers list)*
* **Ethics:**
    * [[The Ethics of Artificial Intelligence in Robotics]] (Bostrom et al., 2014)
    * [[The Social Dilemma of Autonomous Vehicles]] (Bonnefon et al., 2016)

---

## Finding Papers

Good resources for finding relevant research papers include:
* Google Scholar
* arXiv (preprint server, especially for CS/AI/Robotics)
* Semantic Scholar
* IEEE Xplore Digital Library (for IEEE conferences/journals like ICRA, IROS, TRO)
* ACM Digital Library (for ACM conferences/journals)
* Proceedings of major robotics conferences: [[ICRA]], [[IROS]], [[RSS]], CoRL, [[CVPR]] (for vision).

---

## Dynamic Index with Dataview

Use the [[Dataview]] plugin to automatically list all your research paper notes:

````markdown
```dataview
TABLE venue AS "Venue", join(authors) AS "Authors", status as "Status", file.cday AS "Date Added"
FROM "" 
WHERE contains(file.tags, "#research-paper")
SORT venue DESC, title ASC
`````

```

*(This query finds all notes tagged `#research-paper` anywhere in your vault and displays key information from their frontmatter.)*

---

Maintaining this index and creating detailed notes for key papers will significantly enhance your understanding of the field and provide valuable references for your own work.

```


