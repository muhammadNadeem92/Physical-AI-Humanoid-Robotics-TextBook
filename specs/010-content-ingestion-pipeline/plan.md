# Implementation Plan: Embedded RAG Chatbot – Part 1: Data Ingestion & Embeddings

**Branch**: `010-content-ingestion-pipeline` | **Date**: 2025-12-25 | **Spec**: [link](../010-content-ingestion-pipeline/spec.md)
**Input**: Feature specification from `/specs/[###-feature-name]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of a data ingestion and embedding pipeline for an embedded RAG chatbot used inside a Docusaurus-based textbook on Physical AI & Humanoid Robotics. The system will load and parse Docusaurus markdown content, preserve structural metadata, chunk text into 500-800 token segments, generate embeddings using Cohere Embed, store vectors in Qdrant Cloud, and maintain metadata/logs in Neon Postgres with support for re-ingestion on content updates.

## Technical Context

**Language/Version**: Python 3.11+ (as specified in requirements)
**Primary Dependencies**:
- markdown (for parsing Docusaurus markdown content)
- cohere (for generating embeddings using Cohere API)
- qdrant-client (for storing vectors in Qdrant Cloud)
- psycopg2-binary or asyncpg (for Neon Postgres database operations)
- python-dotenv (for environment variable management)
- uv for project management (as specified in requirements)
- tiktoken (for token counting to ensure 500-800 token segments)
- hashlib (for content hashing to detect changes)
**Storage**: Qdrant Cloud (vector database for embeddings), Neon Postgres (metadata and ingestion logs)
**Testing**: pytest for unit and integration tests with proper test coverage
**Target Platform**: Cross-platform (Linux/Windows/macOS) server environment with Python 3.11+
**Project Type**: Single project with CLI tools for ingestion pipeline
**Performance Goals**: Process all Docusaurus markdown files within 10 minutes for typical textbook size (SC-001), 99% successful embedding rate (SC-002), correct preservation of structural metadata for 100% of content segments (SC-003)
**Constraints**:
- Deterministic processing (FR-008)
- Graceful error handling (FR-009)
- Support for re-ingestion on content updates (FR-007)
- No LLM generation (constraint from spec)
- No FastAPI routes (constraint from spec)
- No hallucination-prone logic (constraint from spec)
- Must validate markdown files for proper syntax (FR-010)
**Scale/Scope**: Handle textbook content with modules, chapters, and sections structure, supporting the full book corpus with proper metadata preservation (FR-002)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Based on the project constitution, the following checks pass:
- ✅ **Spec-Driven Workflow**: Following spec-first approach as outlined in constitution (Core Principle #1)
- ✅ **Security & Configuration Management**: Using python-dotenv for environment variable management to avoid hard-coding secrets (Core Principle #5)
- ✅ **Quality & Build Integrity**: Planning to implement proper testing with pytest (Core Principle #6)
- ✅ **Backend Standards**: Using Cohere API and Qdrant Cloud as specified in constitution (Technical Standard #2)
- ✅ **Separation of Concerns**: This ingestion pipeline is separate from the frontend and future chatbot API (Core Principle #2)
- ✅ **Technical Standards**: Using Python backend stack as specified in constitution (Technical Standard #2)
- ✅ **No API keys in code**: Planning to use environment variables for Qdrant, Neon, and Cohere API keys (Core Principle #5)
- ✅ **Performance Goals**: Meeting the specified processing time goals (10 minutes for typical textbook size)
- ✅ **Reliability**: Design includes graceful error handling and data integrity measures
- ✅ **Scalability**: Supporting the expected content volume with modules, chapters, and sections structure

## Project Structure

### Documentation (this feature)

```text
specs/010-content-ingestion-pipeline/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
src/
├── ingestion/
│   ├── __init__.py
│   ├── loader.py          # Load and parse Docusaurus markdown content
│   ├── chunker.py         # Text chunking logic with overlap
│   ├── embedder.py        # Cohere embedding generation
│   ├── storage.py         # Qdrant and Postgres storage logic
│   └── orchestrator.py    # Main pipeline orchestration
├── models/
│   ├── __init__.py
│   ├── content_segment.py # Content segment data model
│   ├── embedding.py       # Embedding data model
│   └── metadata.py        # Metadata data model
├── config/
│   ├── __init__.py
│   └── settings.py        # Configuration and settings management
├── utils/
│   ├── __init__.py
│   ├── file_utils.py      # File handling utilities
│   └── hash_utils.py      # Content hashing for change detection
└── cli/
    ├── __init__.py
    └── main.py            # CLI entry point for the ingestion pipeline

tests/
├── unit/
│   ├── __init__.py
│   └── test_ingestion/    # Unit tests for ingestion components
├── integration/
│   ├── __init__.py
│   └── test_storage/      # Integration tests for storage components
└── contract/
    ├── __init__.py
    └── test_embeddings/   # Contract tests for embedding services

pyproject.toml              # Project dependencies and build config
README.md                   # Project documentation
```

**Structure Decision**: Single project structure with clear separation of concerns. The project is organized into logical modules (ingestion, models, config, utils, cli) with corresponding test directories. This structure supports the requirements of loading, processing, and storing textbook content while maintaining clean architecture and testability.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |