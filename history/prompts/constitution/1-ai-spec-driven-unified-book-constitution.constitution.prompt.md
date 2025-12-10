---
id: 1
title: AI Spec-Driven Unified Book Constitution
stage: constitution
date: 2025-12-10
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: Claude Code
command: /sp.constitution
labels: [constitution, spec-kit-plus, claude-code, book-creation]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

Project: AI/Spec-Driven Unified Book Creation using Spec-Kit Plus and Claude Code

Objective:
To design, write, and deploy a complete, production-quality technical book using AI-native software development practices, powered by Spec-Kit Plus and Claude Code, and published through Docusaurus on GitHub Pages.

Core Principles:

* Accuracy through direct verification of primary and trusted technical sources
* Clarity and accessibility for a technical audience (students, developers, and AI engineers)
* AI-native development with human-in-the-loop validation
* Reproducibility of content, structure, and workflow
* Transparency of sources, tools, and generation process
* Continuous improvement through iterative refinement

Key Standards:

* All factual and technical claims must be verifiable through credible sources
* All explanations must be beginner-friendly while remaining technically correct
* AI-generated content must be reviewed, edited, and validated by humans
* Code examples must be runnable, minimal, and clearly explained
* Citations must be consistently formatted
* No hallucinated concepts, tools, or references are allowed
* All chapters must follow a consistent documentation structure defined by Spec-Kit Plus

Source & Citation Rules:

* Minimum 60% sources from official documentation, research papers, or authoritative technical platforms
* Remaining sources may include trusted blogs, GitHub repositories, and industry case studies
* Plagiarism tolerance: 0%
* All reused ideas must be rewritten originally with proper attribution

Writing & Content Quality:

* Language: Clear, precise, and structured
* Tone: Professional, educational, and practical
* Target readability: Intermediate technical level
* Each chapter must include:

  * Concept explanation
  * Practical example
  * Use case or real-world application
  * Summary

Technical Constraints:

* Platform: Docusaurus
* Deployment: GitHub Pages
* Tooling:

  * Spec-Kit Plus for specification-driven structure
  * Claude Code for AI-assisted content generation
* Version control must be maintained via GitHub
* All content must be written in Markdown
* The book must build successfully with zero build errors

Project Constraints:

* Book length: Determined by topic depth, no artificial padding
* Chapters must be modular and independently readable
* No copyrighted material may be copied directly
* All diagrams must be either original or properly attributed

Success Criteria:

* Fully deployed live book on GitHub Pages
* All chapters pass human factual verification
* Zero plagiarism detected
* Clean GitHub repository with documented workflow
* Clear demonstration of AI-native software development using Spec-driven processes
* Judges can reproduce the book-generation process using the provided specs

Ethical & AI Usage Guidelines:

* AI is used as a co-author, not a replacement for human judgment
* All misleading, biased, or unverifiable content must be removed
* Safety, privacy, and responsible AI principles must be respected at all stages

Final Acceptance Conditions:

* The book builds successfully
* The deployed site is publicly accessible
* The content reflects real understanding, not just AI output
* The project clearly demonstrates the power of Spec-Kit Plus + Claude Code working together

## Response snapshot

The constitution for the AI/Spec-Driven Unified Book Creation project has been successfully created and updated. The new constitution includes six core principles:

1. Accuracy through Direct Verification - All factual and technical claims must be verifiable through credible sources
2. AI-Native Development with Human-in-the-Loop - AI as co-author with human validation
3. Reproducibility and Transparency - Reproducible workflow with transparent sources and tools
4. No Plagiarism or Hallucination - Zero tolerance for plagiarism or hallucinated content
5. Modular and Accessible Content - Clear, accessible content with modular chapters
6. Continuous Improvement - Iterative refinement with human verification

The constitution also includes sections on Technical Constraints, Development Workflow, and Governance. The file is now stored at .specify/memory/constitution.md with version 1.0.0, ratified on 2025-12-10.

## Outcome

- ✅ Impact: Created project constitution with 6 core principles and governance structure
- 🧪 Tests: No tests required for constitution creation
- 📁 Files: Updated .specify/memory/constitution.md with new constitution content
- 🔁 Next prompts: Project can now proceed with feature specification and planning
- 🧠 Reflection: Constitution provides clear guidelines for AI-assisted book creation with human oversight

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin feature specification based on constitution