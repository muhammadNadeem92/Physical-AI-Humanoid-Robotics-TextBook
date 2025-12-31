# Feature Specification: Embedded RAG Chatbot – Part 1: Data Ingestion & Embeddings

**Feature Branch**: `010-content-ingestion-pipeline`
**Created**: 2025-12-25
**Status**: Draft
**Input**: User description: "Embedded RAG Chatbot – Part 1: Data Ingestion & Embeddings

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
- Deterministic"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Content Pipeline Setup (Priority: P1)

As a developer working on the Physical AI & Humanoid Robotics textbook, I need to set up an automated data ingestion pipeline that processes Docusaurus markdown files and converts them into searchable embeddings, so that I can later implement a RAG chatbot that understands the textbook content.

**Why this priority**: This is foundational functionality that must exist before any chatbot functionality can be implemented. Without properly ingested and embedded content, the entire RAG system cannot function.

**Independent Test**: Can be fully tested by running the ingestion pipeline on sample Docusaurus markdown files and verifying that embeddings are generated and stored in Qdrant Cloud, delivering the core capability to transform textbook content into a searchable format.

**Acceptance Scenarios**:

1. **Given** a set of Docusaurus markdown files in /frontend/docs, **When** I run the ingestion pipeline, **Then** the system loads and parses all markdown content preserving structural metadata
2. **Given** parsed markdown content with structural metadata, **When** I run the chunking process, **Then** the text is segmented into 500-800 token segments with appropriate overlap
3. **Given** text chunks with metadata, **When** I run the embedding generation process, **Then** embeddings are created using Cohere Embed and stored in Qdrant Cloud
4. **Given** generated embeddings, **When** I run the storage process, **Then** metadata and ingestion logs are stored in Neon Postgres

---

### User Story 2 - Content Update Handling (Priority: P2)

As a content maintainer for the textbook, I need the system to detect and re-process updated content automatically, so that the RAG chatbot always has access to the most current version of the textbook material.

**Why this priority**: Ensures data consistency and freshness, which is critical for a textbook where content accuracy is paramount.

**Independent Test**: Can be fully tested by modifying existing markdown files, running the re-ingestion process, and verifying that only updated content is processed and stored, delivering the capability to maintain content currency.

**Acceptance Scenarios**:

1. **Given** previously ingested content in the system, **When** I update a markdown file in /frontend/docs, **Then** the system detects the change and re-processes only the updated content
2. **Given** content that hasn't changed, **When** I run the ingestion pipeline, **Then** the system skips unchanged content to avoid unnecessary processing

---

### User Story 3 - Ingestion Monitoring and Logging (Priority: P3)

As a system administrator, I need to monitor the ingestion process and access detailed logs, so that I can troubleshoot issues and verify that the pipeline is functioning correctly.

**Why this priority**: Operational visibility is important for maintaining system reliability and debugging issues that may arise during ingestion.

**Independent Test**: Can be fully tested by running the ingestion pipeline and examining the generated logs in Neon Postgres, delivering the capability to monitor and audit the ingestion process.

**Acceptance Scenarios**:

1. **Given** an ingestion process running, **When** errors or warnings occur, **Then** detailed logs are recorded in Neon Postgres with timestamps and error details
2. **Given** a completed ingestion run, **When** I query the system, **Then** I can access ingestion statistics and status information

---

### Edge Cases

- What happens when a markdown file is corrupted or contains invalid syntax?
- How does the system handle very large markdown files that exceed reasonable token limits?
- What if the Qdrant Cloud service is temporarily unavailable during ingestion?
- How does the system handle network interruptions during the embedding generation process?
- What happens if Neon Postgres is unavailable when trying to store metadata and logs?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST load and parse Docusaurus markdown content from the /frontend/docs directory
- **FR-002**: System MUST preserve structural metadata including module, chapter, section, and page URL for each content segment
- **FR-003**: System MUST chunk text into 500-800 token segments with appropriate overlap to maintain context
- **FR-004**: System MUST generate embeddings using Cohere Embed (English) for each text segment
- **FR-005**: System MUST store generated embeddings in Qdrant Cloud (Free Tier)
- **FR-006**: System MUST store metadata and ingestion logs in Neon Postgres
- **FR-007**: System MUST support re-ingestion of content when updates are detected in the source files
- **FR-008**: System MUST be deterministic in its processing to ensure consistent results
- **FR-009**: System MUST handle errors gracefully and continue processing other content when individual files fail
- **FR-010**: System MUST validate markdown files for proper syntax before attempting to parse them

### Key Entities

- **ContentSegment**: A chunk of text from the textbook content (500-800 tokens) with associated metadata
- **Embedding**: Vector representation of a content segment generated by Cohere Embed
- **Metadata**: Structural information about the content including module, chapter, section, and page URL
- **IngestionLog**: Record of ingestion activities including timestamps, status, and error information

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: The system processes all Docusaurus markdown files in /frontend/docs within 10 minutes for a typical textbook size
- **SC-002**: 99% of content segments are successfully embedded and stored in Qdrant Cloud without errors
- **SC-003**: The system correctly preserves structural metadata for 100% of content segments
- **SC-004**: Re-ingestion detects and processes only updated content, reducing processing time by at least 80% when only a few files change
- **SC-005**: Ingestion logs provide sufficient detail to diagnose and resolve 95% of processing issues within 30 minutes