# Implementation Tasks: Embedded RAG Chatbot – Part 1: Data Ingestion & Embeddings

**Feature**: 010-content-ingestion-pipeline
**Date**: 2025-12-25
**Status**: Initial

## Summary

Implementation of a data ingestion and embedding pipeline for an embedded RAG chatbot used inside a Docusaurus-based textbook on Physical AI & Humanoid Robotics. The system will load and parse Docusaurus markdown content, preserve structural metadata, chunk text into 500-800 token segments, generate embeddings using Cohere Embed, store vectors in Qdrant Cloud, and maintain metadata/logs in Neon Postgres with support for re-ingestion on content updates.

## Dependencies

- User Story 2 (Content Update Handling) depends on User Story 1 (Content Pipeline Setup) foundational components
- User Story 3 (Ingestion Monitoring and Logging) depends on User Story 1 (Content Pipeline Setup) foundational components

## Parallel Execution Examples

- Models can be developed in parallel: ContentSegment, Embedding, Metadata, and IngestionLog models can be created simultaneously
- Utility functions can be developed in parallel: file_utils.py and hash_utils.py can be implemented independently
- Unit tests can be developed in parallel with their corresponding components

## Implementation Strategy

- MVP: Implement User Story 1 (Content Pipeline Setup) with basic functionality to load, parse, chunk, embed, and store content
- Incremental delivery: Add re-ingestion capabilities and monitoring in subsequent phases
- Focus on core functionality first, then add error handling and logging

---

## Phase 1: Setup Tasks

### Project Initialization

- [X] T001 Create project structure directories (src/ingestion, src/models, src/config, src/utils, src/cli, tests/unit, tests/integration, tests/contract)
- [X] T002 [P] Initialize pyproject.toml with dependencies (cohere, qdrant-client, psycopg2-binary, python-dotenv, tiktoken, markdown, pytest)
- [X] T003 Create initial README.md with project overview and setup instructions
- [X] T004 Create .env.example file with environment variable placeholders
- [X] T005 Create .gitignore for Python project

---

## Phase 2: Foundational Tasks

### Configuration and Settings

- [X] T006 Create config/settings.py with configuration management using python-dotenv
- [X] T007 [P] Create models/__init__.py to organize model imports
- [X] T008 [P] Create utils/__init__.py to organize utility imports
- [X] T009 [P] Create ingestion/__init__.py to organize ingestion imports
- [X] T010 [P] Create cli/__init__.py to organize CLI imports

### Utility Functions

- [X] T011 Create utils/file_utils.py with functions for file operations and directory traversal
- [X] T012 Create utils/hash_utils.py with functions for content hashing using hashlib
- [X] T013 [P] Create models/content_segment.py with ContentSegment data model
- [X] T014 [P] Create models/embedding.py with Embedding data model
- [X] T015 [P] Create models/metadata.py with Metadata data model

---

## Phase 3: User Story 1 - Content Pipeline Setup (Priority: P1)

### Goal
As a developer working on the Physical AI & Humanoid Robotics textbook, I need to set up an automated data ingestion pipeline that processes Docusaurus markdown files and converts them into searchable embeddings, so that I can later implement a RAG chatbot that understands the textbook content.

### Independent Test Criteria
Can be fully tested by running the ingestion pipeline on sample Docusaurus markdown files and verifying that embeddings are generated and stored in Qdrant Cloud, delivering the core capability to transform textbook content into a searchable format.

### Implementation Tasks

#### Content Loading

- [X] T016 [US1] Create ingestion/loader.py with function to load and parse Docusaurus markdown content from /frontend/docs
- [X] T017 [US1] Implement parsing of Docusaurus markdown with metadata extraction (module, chapter, section, page URL)
- [X] T018 [US1] Add validation for markdown syntax to ensure proper parsing (FR-010)

#### Content Chunking

- [X] T019 [US1] Create ingestion/chunker.py with function to chunk text into 500-800 token segments
- [X] T020 [US1] Implement token counting using tiktoken to ensure segments are within 500-800 token range (FR-003)
- [X] T021 [US1] Add overlap functionality between chunks to maintain context

#### Embedding Generation

- [X] T022 [US1] Create ingestion/embedder.py with function to generate embeddings using Cohere API
- [X] T023 [US1] Implement Cohere API integration with proper error handling (FR-004)
- [X] T024 [US1] Add retry logic for API calls to handle temporary unavailability

#### Storage Implementation

- [X] T025 [US1] Create ingestion/storage.py with functions to store embeddings in Qdrant Cloud
- [X] T026 [US1] Implement storage of metadata and ingestion logs in Neon Postgres
- [X] T027 [US1] Create database schema for metadata and ingestion logs tables

#### Pipeline Orchestration

- [X] T028 [US1] Create ingestion/orchestrator.py with main pipeline orchestration logic
- [X] T029 [US1] Implement end-to-end pipeline flow: load → parse → chunk → embed → store
- [X] T030 [US1] Add deterministic processing to ensure consistent results (FR-008)

#### CLI Interface

- [X] T031 [US1] Create cli/main.py with CLI entry point for the ingestion pipeline
- [X] T032 [US1] Implement command-line arguments for source path, collection name, and other options
- [X] T033 [US1] Add logging to track pipeline progress and status

---

## Phase 4: User Story 2 - Content Update Handling (Priority: P2)

### Goal
As a content maintainer for the textbook, I need the system to detect and re-process updated content automatically, so that the RAG chatbot always has access to the most current version of the textbook material.

### Independent Test Criteria
Can be fully tested by modifying existing markdown files, running the re-ingestion process, and verifying that only updated content is processed and stored, delivering the capability to maintain content currency.

### Implementation Tasks

#### Change Detection

- [X] T034 [US2] Enhance hash_utils.py to store and compare file hashes for change detection (FR-007)
- [X] T035 [US2] Implement content comparison logic to identify updated markdown files
- [X] T036 [US2] Create function to track which files have been processed in previous runs

#### Selective Processing

- [X] T037 [US2] Modify orchestrator to process only updated content
- [X] T038 [US2] Implement logic to skip unchanged content to avoid unnecessary processing
- [X] T039 [US2] Add option to force re-processing of all content

#### Re-ingestion Logic

- [X] T040 [US2] Update storage logic to handle re-ingestion of content segments
- [X] T041 [US2] Implement deletion of old embeddings when content is updated
- [X] T042 [US2] Add performance optimization to reduce processing time when few files change (SC-004)

---

## Phase 5: User Story 3 - Ingestion Monitoring and Logging (Priority: P3)

### Goal
As a system administrator, I need to monitor the ingestion process and access detailed logs, so that I can troubleshoot issues and verify that the pipeline is functioning correctly.

### Independent Test Criteria
Can be fully tested by running the ingestion pipeline and examining the generated logs in Neon Postgres, delivering the capability to monitor and audit the ingestion process.

### Implementation Tasks

#### Logging Implementation

- [X] T043 [US3] Create IngestionLog model in models/ to track ingestion activities
- [X] T044 [US3] Enhance storage.py to record ingestion logs in Neon Postgres
- [X] T045 [US3] Implement detailed logging for timestamps, status, and error information (FR-006)

#### Error Handling

- [X] T046 [US3] Add comprehensive error handling to continue processing other content when individual files fail (FR-009)
- [X] T047 [US3] Implement error tracking and reporting in ingestion logs
- [X] T048 [US3] Add retry mechanisms for transient failures

#### Monitoring Features

- [X] T049 [US3] Create functions to query and analyze ingestion statistics
- [X] T050 [US3] Implement summary statistics for ingestion runs
- [X] T051 [US3] Add functionality to diagnose and resolve processing issues (SC-005)

---

## Phase 6: Polish & Cross-Cutting Concerns

### Testing

- [X] T052 [P] Create unit tests for ingestion/loader.py components
- [X] T053 [P] Create unit tests for ingestion/chunker.py components
- [ ] T054 [P] Create unit tests for ingestion/embedder.py components
- [ ] T055 [P] Create unit tests for ingestion/storage.py components
- [X] T056 [P] Create integration tests for storage components
- [ ] T057 [P] Create contract tests for embedding services

### Documentation

- [X] T058 Update README.md with detailed usage instructions
- [ ] T059 Add configuration documentation with environment variable explanations
- [ ] T060 Create troubleshooting guide based on common error scenarios

### Performance & Optimization

- [ ] T061 Optimize token counting and chunking for performance (SC-001)
- [ ] T062 Implement batch processing for embeddings to improve efficiency
- [ ] T063 Add performance monitoring to track processing times and success rates (SC-002)

### Edge Case Handling

- [ ] T064 Handle corrupted or invalid markdown files gracefully (Edge Case 1)
- [ ] T065 Handle very large markdown files that exceed reasonable token limits (Edge Case 2)
- [ ] T066 Handle temporary unavailability of Qdrant Cloud service (Edge Case 3)
- [ ] T067 Handle network interruptions during embedding generation (Edge Case 4)
- [ ] T068 Handle Neon Postgres unavailability during metadata storage (Edge Case 5)

### Final Validation

- [ ] T069 Verify 99% successful embedding rate (SC-002)
- [ ] T070 Verify 100% structural metadata preservation (SC-003)
- [ ] T071 Verify 80%+ processing time reduction for incremental updates (SC-004)
- [ ] T072 Verify 95% of processing issues can be diagnosed within 30 minutes (SC-005)