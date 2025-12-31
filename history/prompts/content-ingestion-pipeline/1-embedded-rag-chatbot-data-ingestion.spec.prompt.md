---
id: 1
title: embedded-rag-chatbot-data-ingestion
stage: spec
date_iso: 2025-12-25
surface: agent
model: Claude Sonnet 4.5
feature: content-ingestion-pipeline
branch: 010-content-ingestion-pipeline
user: user
command: /sp.specify "Embedded RAG Chatbot – Part 1: Data Ingestion & Embeddings"
labels:
  - rag
  - data-ingestion
  - embeddings
  - textbook
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - specs/010-content-ingestion-pipeline/spec.md
  - specs/010-content-ingestion-pipeline/checklists/requirements.md
tests_yaml:
  - specification creation
  - quality validation
prompt_text: |
  Embedded RAG Chatbot – Part 1: Data Ingestion & Embeddings

  Objective:
  Design and implement the data ingestion and embedding pipeline for an embedded RAG chatbot
  used inside a Docusaurus-based textbook on Physical AI & Humanoid Robotics.

  Scope:
  This part covers ONLY offline and repeatable data preparation.
  No chat APIs or UI components are included.

  Target Runtime:
  Python 3.11+
  uv-based project management

  Inputs:
  - Docusaurus markdown files located in /frontend/docs
  - Book structure (modules, chapters, sections)

  Responsibilities:
  1. Load and parse Docusaurus markdown content
  2. Preserve structural metadata:
     - module
     - chapter
     - section
     - page URL
  3. Chunk text into 500–800 token segments with overlap
  4. Generate embeddings using Cohere Embed (English)
  5. Store vectors in Qdrant Cloud (Free Tier)
  6. Store metadata and ingestion logs in Neon Postgres
  7. Support re-ingestion on content updates

  Constraints:
  - No LLM generation
  - No FastAPI routes
  - No hallucination-prone logic
  - Deterministic
response_text: |
  Created comprehensive specification for the Embedded RAG Chatbot data ingestion pipeline.
  The specification includes:
  - Three prioritized user stories covering core functionality, content updates, and monitoring
  - 10 functional requirements covering all aspects of the ingestion pipeline
  - Key entities definition
  - 5 measurable success criteria
  - Edge cases handling
  Also created a quality checklist that validates all requirements were met.
outcome: |
  Specification successfully created and validated. Ready for planning phase (/sp.plan).
evaluation: |
  The specification meets all quality criteria and is ready for implementation planning.
---