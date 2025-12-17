<!-- SYNC IMPACT REPORT
Version change: 1.0.0 → 1.0.0
Modified principles: None (initial creation)
Added sections: All sections (initial constitution)
Removed sections: None
Templates requiring updates:
- .specify/templates/plan-template.md ✅ updated
- .specify/templates/spec-template.md ✅ updated
- .specify/templates/tasks-template.md ✅ updated
- .specify/templates/commands/*.md ⚠ pending review
Follow-up TODOs: None
-->

# Physical AI & Humanoid Robotics TextBook Constitution

## Core Principles

### Spec-Driven Workflow
No content or code is written without a prior specification in `sp.roadmap.md` or `sp.tech.md`. All development must follow the spec-first, AI-driven workflow where requirements are clearly defined before implementation.

### Separation of Concerns
The documentation site (frontend) and the RAG API (backend) must be logically and technically separated. The Docusaurus frontend and FastAPI backend must remain isolated while being synchronized in the monorepo structure.

### Accuracy & Reproducibility
All technical explanations must be verifiable. All code examples must be executable. Every concept, tutorial, and code snippet in the textbook must be tested and reproducible by readers.

### Context Awareness
The embedded RAG chatbot must answer questions from the full book corpus and also provide contextual answers based only on user-selected text. The chatbot must support both global search and contextual search capabilities.

### Security & Configuration Management
No hard-coding of secrets or API keys. All sensitive information (OpenAI, Qdrant, Neon) must be loaded via environment variables. No API keys or secrets may be committed to the repository.

### Quality & Build Integrity
All code must pass quality control checks. The `npm run build` command must pass with zero errors before deployment. All technical content must be accurate and verifiable.

## Technical Standards

### Frontend Standards
The frontend uses Docusaurus 3.x framework with content in Markdown (.md/.mdx) format. The book must contain at least 8 modules, each with 3+ chapters following the template: Introduction, Core Concepts, Examples/Labs, and Summary & Key Takeaways. A floating chatbot UI must be available on every page and communicate exclusively with the backend API.

### Backend Standards
The backend uses FastAPI (Python) with OpenAI Agents/ChatKit SDK as the AI engine. Vector storage uses Qdrant Cloud (Free Tier) and metadata storage uses Neon Serverless Postgres. The RAG system must support both global search across the full book corpus and contextual search based on user-selected text only.

### Deployment & Infrastructure
Frontend deployment targets GitHub Pages via GitHub Actions. Backend deployment targets serverless-compatible platforms (Vercel, Render, Railway, etc.). Both systems must be deployed separately while maintaining synchronization.

## Development Workflow

### Content Creation Process
All content creation follows the Spec-First methodology. Each module and chapter must have corresponding specifications before writing begins. Content must progress from beginner to intermediate level, ensuring accessibility for newcomers to Physical AI and Humanoid Robotics.

### Code Integration
Executable code examples must be integrated into documentation where applicable. All code examples must follow the chapter template structure and be verified for correctness. Code must be organized in a way that supports the educational progression of concepts.

### Review & Quality Assurance
All content and code must undergo verification for accuracy and reproducibility. Technical concepts must be validated by subject matter expertise. The chatbot functionality must be tested with both global and contextual queries to ensure proper RAG behavior.

## Governance

This constitution governs all development activities for the Physical AI & Humanoid Robotics TextBook project. All team members must adhere to these principles. Amendments to this constitution require explicit approval and must be documented with clear justification. Versioning follows semantic versioning principles: MAJOR for backward incompatible changes, MINOR for new principles or material expansions, and PATCH for clarifications and corrections.

All pull requests and reviews must verify compliance with these constitutional principles. Changes that violate core principles require special justification and approval. This constitution serves as the authoritative guide for all project decisions and trade-offs.

**Version**: 1.0.0 | **Ratified**: 2025-12-15 | **Last Amended**: 2025-12-15