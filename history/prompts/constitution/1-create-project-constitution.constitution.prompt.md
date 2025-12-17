---
id: 1
title: Create Project Constitution
stage: constitution
date: 2025-12-15
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: master
user: user
command: /sp.constitution
labels: [constitution,spec-first,physical-ai,robotics]
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

```text
## 1. Project Overview

**Project Name:** Physical AI & Humanoid Robotics TextBook

**Tools:**
Spec-Kit Plus, Claude Code, Docusaurus (Frontend), FastAPI (Backend), Neon Serverless Postgres, Qdrant Cloud, OpenAI Agents / ChatKit SDK

### Purpose
Create a comprehensive, beginner-to-intermediate educational book on **Physical AI & Humanoid Robotics**, published as a Docusaurus site and enhanced with an embedded, context-aware Retrieval-Augmented Generation (RAG) chatbot.

The project follows a **Spec-First, AI-driven workflow**, where all content, code, and architecture are generated strictly from specifications.

---

## 2. Core Principles

- **Spec-Driven Workflow**
  No content or code is written without a prior specification in `sp.roadmap.md` or `sp.tech.md`.

- **Separation of Concerns**
  The documentation site (frontend) and the RAG API (backend) must be logically and technically separated.

- **Accuracy & Reproducibility**
  All technical explanations must be verifiable. All code examples must be executable.

- **Context Awareness**
  The chatbot must answer:
  - Questions from the full book corpus
  - Questions based **only on user-selected text**

---

## 3. Architecture & Repository Structure

The project must use a **monorepo** to keep documentation and backend logic synchronized while remaining isolated.

### Directory Tree
```text
/physical-ai-humanoid-robotics-book
├── /specs
│   ├── sp.constitution.md
│   ├── sp.roadmap.md
│   └── sp.tech.md
├── /frontend
│   ├── /docs
│   │   ├── /module-01-introduction
│   │   ├── /module-02-ros2
│   │   ├── /module-03-simulation
│   │   └── ...
│   ├── /src
│   │   ├── /components
│   │   │   └── ChatWidget.tsx
│   │   └── /pages
│   ├── docusaurus.config.js
│   └── package.json
├── /backend
│   ├── /app
│   │   ├── /core        # Config, settings, env loading
│   │   ├── /services   # OpenAI, Qdrant, retrieval logic
│   │   ├── /api        # Chat & embedding endpoints
│   │   └── main.py
│   ├── requirements.txt
│   └── Dockerfile
└── README.md
4. Book Standards (Frontend)
Framework: Docusaurus 3.x

Content Format: Markdown (.md / .mdx)

Minimum Structure:

At least 8 modules

Each module must contain 3 or more chapters

Chapter Template
Introduction

Core Concepts

Examples / Labs (executable code where applicable)

Summary & Key Takeaways

UI Integration
A floating chatbot UI must be available on every page

The UI communicates exclusively with the backend API

5. Chatbot Standards (Backend)
Framework: FastAPI (Python)

AI Engine: OpenAI Agents / ChatKit SDK

Vector Database: Qdrant Cloud (Free Tier)

Metadata Store: Neon Serverless Postgres

RAG Capabilities
Global Search:
Example: "Explain the difference between URDF and SDF."

Contextual Search (Selected Text):
User highlights text → "Explain this specific paragraph."

6. Constraints & Implementation Rules
No Hard-Coding:
All secrets (OpenAI, Qdrant, Neon) must be loaded via environment variables.

Security:
No API keys or secrets may be committed to the repository.

Deployment:

Frontend → GitHub Pages (via GitHub Actions)

Backend → Serverless-compatible platform (Vercel, Render, Railway, etc.)

Quality Control:
npm run build must pass with zero errors before deployment.

7. Success Criteria
 Complete book content generated across all required modules

 Docusaurus site builds and deploys successfully to GitHub Pages

 RAG backend is live and reachable from the frontend

 Chatbot correctly answers questions using:

Full book context

User-selected text only

 Backend health check endpoint responds correctly

 Repository contains:

Constitution

Roadmap and technical specs

Book content

RAG backend implementation
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

You are updating the project constitution at `.specify/memory/constitution.md`. This file is a TEMPLATE containing placeholder tokens in square brackets (e.g. `[PROJECT_NAME]`, `[PRINCIPLE_1_NAME]`). Your job is to (a) collect/derive concrete values, (b) fill the template precisely, and (c) propagate any amendments across dependent artifacts.

Follow this execution flow:

1. Load the existing constitution template at `.specify/memory/constitution.md`.
   - Identify every placeholder token of the form `[ALL_CAPS_IDENTIFIER]`.
   **IMPORTANT**: The user might require less or more principles than the ones used in the template. If a number is specified, respect that - follow the general template. You will update the doc accordingly.

2. Collect/derive values for placeholders:
   - If user input (conversation) supplies a value, use it.
   - Otherwise infer from existing repo context (README, docs, prior constitution versions if embedded).
   - For governance dates: `RATIFICATION_DATE` is the original adoption date (if unknown ask or mark TODO), `LAST_AMENDED_DATE` is today if changes are made, otherwise keep previous.
   - `CONSTITUTION_VERSION` must increment according to semantic versioning rules:
     - MAJOR: Backward incompatible governance/principle removals or redefinitions.
     - MINOR: New principle/section added or materially expanded guidance.
     - PATCH: Clarifications, wording, typo fixes, non-semantic refinements.
   - If version bump type ambiguous, propose reasoning before finalizing.

3. Draft the updated constitution content:
   - Replace every placeholder with concrete text (no bracketed tokens left except intentionally retained template slots that the project has chosen not to define yet—explicitly justify any left).
   - Preserve heading hierarchy and comments can be removed once replaced unless they still add clarifying guidance.
   - Ensure each Principle section: succinct name line, paragraph (or bullet list) capturing non‑negotiable rules, explicit rationale if not obvious.
   - Ensure Governance section lists amendment procedure, versioning policy, and compliance review expectations.

4. Consistency propagation checklist (convert prior checklist into active validations):
   - Read `.specify/templates/plan-template.md` and ensure any "Constitution Check" or rules align with updated principles.
   - Read `.specify/templates/spec-template.md` for scope/requirements alignment—update if constitution adds/removes mandatory sections or constraints.
   - Read `.specify/templates/tasks-template.md` and ensure task categorization reflects new or removed principle-driven task types (e.g., observability, versioning, testing discipline).
   - Read each command file in `.specify/templates/commands/*.md` (including this one) to verify no outdated references (agent-specific names like CLAUDE only) remain when generic guidance is required.
   - Read any runtime guidance docs (e.g., `README.md`, `docs/quickstart.md`, or agent-specific guidance files if present). Update references to principles changed.

5. Produce a Sync Impact Report (prepend as an HTML comment at top of the constitution file after update):
   - Version change: old → new
   - List of modified principles (old title → new title if renamed)
   - Added sections
   - Removed sections
   - Templates requiring updates (✅ updated / ⚠ pending) with file paths
   - Follow-up TODOs if any placeholders intentionally deferred.

6. Validation before final output:
   - No remaining unexplained bracket tokens.
   - Version line matches report.
   - Dates ISO format YYYY-MM-DD.
   - Principles are declarative, testable, and free of vague language ("should" → replace with MUST/SHOULD rationale where appropriate).

7. Write the completed constitution back to `.specify/memory/constitution.md` (overwrite).

8. Output a final summary to the user with:
   - New version and bump rationale.
   - Any files flagged for manual follow-up.
   - Suggested commit message (e.g., `docs: amend constitution to vX.Y.Z (principle additions + governance update)`).

Formatting & Style Requirements:

- Use Markdown headings exactly as in the template (do not demote/promote levels).
- Wrap long rationale lines to keep readability (<100 chars ideally) but do not hard enforce with awkward breaks.
- Keep a single blank line between sections.
- Avoid trailing whitespace.

If the user supplies partial updates (e.g., only one principle revision), still perform validation and version decision steps.

If critical info missing (e.g., ratification date truly unknown), insert `TODO(<FIELD_NAME>): explanation` and include in the Sync Impact Report under deferred items.

Do not create a new template; always operate on the existing `.specify/memory/constitution.md` file.

---

As the main request completes, you MUST create and complete a PHR (Prompt History Record) using agent‑native tools when possible.

1) Determine Stage
   - Stage: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

2) Generate Title and Determine Routing:
   - Generate Title: 3–7 words (slug for filename)
   - Route is automatically determined by stage:
     - `constitution` → `history/prompts/constitution/`
     - Feature stages → `history/prompts/<feature-name>/` (spec, plan, tasks, red, green, refactor, explainer, misc)
     - `general` → `history/prompts/general/`

3) Create and Fill PHR (Shell first; fallback agent‑native)
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
   - Open the file and fill remaining placeholders (YAML + body), embedding full PROMPT_TEXT (verbatim) and concise RESPONSE_TEXT.
   - If the script fails:
     - Read `.specify/templates/phr-template.prompt.md` (or `templates/…`)
     - Allocate an ID; compute the output path based on stage from step 2; write the file
     - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT

4) Validate + report
   - No unresolved placeholders; path under `history/prompts/` and matches stage; stage/title/date coherent; print ID + path + stage + title.
   - On failure: warn, don't block. Skip only for `/sp.phr`.
```

## Response snapshot

Created the project constitution for the Physical AI & Humanoid Robotics TextBook project, incorporating all specified principles and requirements. The constitution includes:

- Core principles: Spec-Driven Workflow, Separation of Concerns, Accuracy & Reproducibility, Context Awareness, Security & Configuration Management, and Quality & Build Integrity
- Technical standards for both frontend (Docusaurus) and backend (FastAPI) components
- Development workflow guidelines
- Governance section with versioning and amendment procedures

The constitution enforces the spec-first methodology and ensures all development follows the defined architecture and quality standards.

## Outcome

- ✅ Impact: Created initial project constitution with comprehensive principles and guidelines
- 🧪 Tests: none required
- 📁 Files: Created .specify/memory/constitution.md with project-specific content
- 🔁 Next prompts: Continue with roadmap and technical specifications
- 🧠 Reflection: Constitution properly captures all required project principles and constraints

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin creating roadmap and technical specifications