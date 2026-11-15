# Contributing to ROBOTICS-for-PEOPLE

Welcome to the ROBOTICS-for-PEOPLE open-source textbook project! I'm excited that you're interested in contributing. This guide will help you get started.

## Purpose

The goal of ROBOTICS-for-PEOPLE is to create a comprehensive, accessible textbook on robotics that serves learners at all levels. The project covers everything from basic concepts to advanced applications.

## How to Contribute

I welcome contributions in many forms:
- Writing or editing content
- Providing code examples or tutorials
- Reporting bugs or suggesting improvements
- Translating content
- Reviewing pull requests

## Getting Started

### What to work on
The [Frontier](00_Introduction/Frontier.md) is the to-do list, rebuilt from the textbook itself:
- **Most wanted notes:** topics other notes already link to, ranked by how many are waiting, each with the file name to create.
- **Stubs:** notes that exist but need writing.
- **Thin subfields:** parts of robotics with fewer than three notes.
- **Drafts awaiting a review pass**, by chapter.

A part of robotics that isn't on the map at all is welcome too. Add it to `tools/subfields.yml` in the same pull request as its first note.

### Setting up
1. Fork the repository and clone your fork.
2. Open the folder in [Obsidian](https://obsidian.md) as a vault. The vault settings are included: enable community plugins when asked, so the Dataview views work. Any Markdown editor works too.
3. Install the checker: `pip install pyyaml` (Python 3.10+).

### Writing a note
1. Create the file where it belongs (`Title_Case_With_Underscores.md`, one note per topic, no duplicate names).
2. Start it with the schema v2 frontmatter from the [Vault Taxonomy Guide](00_Introduction/Vault_Taxonomy_Guide.md#frontmatter-schema-v2): `title`, `description`, `tags`, `type`, `level`, `subfield`, `status`, `author`, `date`, plus `prerequisites` and `related` where they apply.
3. Use tags from `tools/tags.yml`. If you really need a new one, add it there in the same pull request.
4. Link on first mention with `[[Note_Name]]`. Only link to notes that exist, or to a topic listed on the Frontier.

### Before you open a pull request
```bash
python3 tools/vault.py lint            # schema, tags, prerequisites, unclosed code blocks
python3 tools/vault.py fix-links       # finds links that misspell an existing note (--write fixes them)
python3 tools/vault.py pages --write   # rebuilds the Subfield Atlas, Learning Paths, Frontier and README stats
```
Commit what `pages --write` changes. Continuous integration runs the same checks and builds the website. Two rules always apply: the number of broken links may only go down, and prerequisites may not form a loop.

### Submitting
1. Create a branch for your contribution.
2. Commit with clear, descriptive messages.
3. Open a pull request describing what you added or changed.

## Content Guidelines

- **Writing style:** clear, concise and technical, but written so a newcomer can follow. Explain from first principles before the formalism.
- **Structure:** the deepest entries follow the keystone template: where the word comes from → first principles → maths and physics → a worked example → tooling → a reading list.
- **Formatting:** Markdown, with LaTeX maths (`$...$`, `$$...$$`) and fenced code blocks that are always closed.
- **Citations:** always cite your sources and give credit to original authors.

## License

This project is licensed under the MIT License. By contributing, you agree that your contributions will be licensed under its terms.

## AI Agent

I built an AI agent in Mistral, accessing an extensive dataset of top robotics textbooks, research papers, and deep research conducted by Gemini 2.5, ChatGPT, Claude. Each tool is used to cross-reference content, equations, and linting code for syntax. This is an opportunity for contributions from the community to build out content as the field continues to grow rapidly.

## Review Process

Contributions will be reviewed based on clarity, accuracy, and relevance. You can expect feedback within a reasonable timeframe.

## Recognition

All contributors will be recognized in our contributors' list and through social media shout-outs.

## Contact Information

For questions or support, contact me at jmsmith10603@gmail.com. Follow me here: https://x.com/MARKUS_Learn.
