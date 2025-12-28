# Implementation Tasks: Embedded RAG Chatbot – Part 2: RAG Orchestration & Chat APIs

**Feature**: 011-rag-orchestration-api
**Date**: 2025-12-25
**Status**: Initial

## Summary

Implementation of a backend RAG orchestration and chat API layer for an embedded chatbot used inside a Docusaurus-based textbook on Physical AI & Humanoid Robotics. The system will accept user queries via REST APIs, retrieve relevant content from Qdrant Cloud using Cohere embeddings, process them through a Gemini-based Agent SDK, and return contextual responses with source citations. The system supports global book queries, scoped content queries, selected text queries, and maintains conversational memory per session.

## Dependencies

- User Story 2 (Scoped Content Query) depends on User Story 1 (Global Book Query) foundational components
- User Story 3 (Selected Text Query) depends on User Story 1 (Global Book Query) foundational components
- User Story 4 (Conversational Memory & History) depends on User Story 1 (Global Book Query) foundational components

## Parallel Execution Examples

- Models can be developed in parallel: ChatSession, UserQuery, RetrievedChunk, ChatResponse, Feedback, and TokenUsageLog models can be created simultaneously
- Service components can be developed in parallel: retrieval_service.py and agent_service.py can be implemented independently
- API endpoints can be developed in parallel: chat.py endpoints can be implemented in parallel with service development
- Unit tests can be developed in parallel with their corresponding components

## Implementation Strategy

- MVP: Implement User Story 1 (Global Book Query) with basic functionality to accept queries, retrieve from Qdrant, and return responses with citations
- Incremental delivery: Add scoped queries, selected text mode, and conversation history in subsequent phases
- Focus on core functionality first, then add error handling and comprehensive validation

---

## Phase 1: Setup Tasks

### Project Initialization

- [X] T001 Create project structure directories (src/api, src/models, src/services, src/config, src/utils, tests/unit, tests/integration, tests/contract)
- [X] T002 [P] Initialize pyproject.toml with dependencies (fastapi, openai, google-generativeai, qdrant-client, psycopg2-binary, python-dotenv, pydantic, sse-starlette, cohere, pytest)
- [X] T003 Create initial README.md with project overview and setup instructions
- [X] T004 Create .env.example file with environment variable placeholders
- [X] T005 Create .gitignore for Python project

---

## Phase 2: Foundational Tasks

### Configuration and Settings

- [X] T006 Create config/settings.py with configuration management using python-dotenv
- [X] T007 [P] Create models/__init__.py to organize model imports
- [X] T008 [P] Create utils/__init__.py to organize utility imports
- [X] T009 [P] Create api/__init__.py to organize API imports
- [X] T010 [P] Create services/__init__.py to organize service imports

### Data Models

- [X] T011 [P] Create models/chat_session.py with ChatSession data model
- [X] T012 [P] Create models/user_query.py with UserQuery data model
- [X] T013 [P] Create models/retrieved_chunk.py with RetrievedChunk data model
- [X] T014 [P] Create models/chat_response.py with ChatResponse data model
- [X] T015 [P] Create models/feedback.py with Feedback data model
- [X] T016 [P] Create models/token_usage_log.py with TokenUsageLog data model

---

## Phase 3: User Story 1 - Global Book Query (Priority: P1)

### Goal
As a student using the Physical AI & Humanoid Robotics textbook, I want to ask questions about the entire book content, so that I can get comprehensive answers that draw from the full textbook corpus and understand complex concepts through relevant citations.

### Independent Test Criteria
Can be fully tested by sending questions to the /chat/query endpoint without scope filters and verifying that responses include cited source snippets with page URLs from across the entire book, delivering the core capability to access textbook knowledge through conversation.

### Implementation Tasks

#### Configuration and Setup

- [X] T017 [US1] Create config/database.py with database connection setup for Neon Postgres
- [X] T018 [US1] Create config/qdrant.py with Qdrant Cloud connection setup
- [X] T019 [US1] Create config/llm.py with Gemini API configuration

#### Core Services

- [X] T020 [US1] Create services/retrieval_service.py with functions to retrieve content chunks from Qdrant
- [X] T021 [US1] Create services/agent_service.py with Agent SDK integration for Gemini
- [X] T022 [US1] Create services/session_service.py with session management functions
- [X] T023 [US1] Create services/history_service.py with conversation history functions
- [X] T024 [US1] Create services/feedback_service.py with feedback processing functions

#### RAG Orchestration

- [X] T025 [US1] Create services/rag_orchestrator.py with main RAG orchestration logic
- [X] T026 [US1] Implement query processing and scope determination in services/query_processor.py
- [X] T027 [US1] Add grounding rule enforcement to prevent responses without context (FR-004)

#### API Implementation

- [X] T028 [US1] Create api/chat.py with basic API endpoint structure
- [X] T029 [US1] Implement POST /chat/query endpoint for global book queries (FR-001)
- [X] T030 [US1] Add streaming response functionality using SSE for ChatKit compatibility (FR-006)

#### Response Generation

- [X] T031 [US1] Implement response generation with cited source snippets and page URLs (FR-007)
- [X] T032 [US1] Add conversational memory support per session (FR-008)
- [X] T033 [US1] Store chat sessions, user queries, and retrieved chunks in Neon Postgres (FR-009)

#### Main Application

- [X] T034 [US1] Create main.py with FastAPI application initialization
- [X] T035 [US1] Set up API routes and dependency injection
- [X] T036 [US1] Add error handling and logging for the API

---

## Phase 4: User Story 2 - Scoped Content Query (Priority: P2)

### Goal
As a student studying a specific module or chapter, I want to ask questions that are limited to that specific content section, so that I can focus my learning on targeted material without getting responses that reference unrelated parts of the book.

### Independent Test Criteria
Can be fully tested by sending queries with module/chapter filters to the /chat/query endpoint and verifying that responses only include citations from the specified sections, delivering the capability to focus on targeted content areas.

### Implementation Tasks

#### Query Filtering

- [X] T037 [US2] Enhance query_processor.py to handle module/chapter scope filters
- [X] T038 [US2] Update retrieval_service.py to apply scope filters when querying Qdrant
- [X] T039 [US2] Add validation to ensure scope filters are properly applied

#### API Enhancement

- [X] T040 [US2] Enhance POST /chat/query endpoint to accept module/chapter parameters
- [X] T041 [US2] Add scoped query processing logic to rag_orchestrator.py
- [X] T042 [US2] Support three retrieval scopes: entire book, specific module/chapter, and user-selected text (FR-013)

---

## Phase 5: User Story 3 - Selected Text Query (Priority: P3)

### Goal
As a student reading a specific section of the textbook, I want to ask questions about only the text I've selected, so that I can get clarifications that are strictly based on that specific content without external context.

### Independent Test Criteria
Can be fully tested by sending selected text with questions to the /chat/selected-text endpoint and verifying that responses are strictly grounded in the provided text with no external retrieval, delivering the capability for focused analysis of specific content.

### Implementation Tasks

#### Selected Text Processing

- [X] T043 [US3] Implement selected text mode in rag_orchestrator.py
- [X] T044 [US3] Add strict grounding enforcement to block external retrieval in selected-text mode (FR-005)
- [X] T045 [US3] Create validation to ensure responses are based only on provided text

#### API Endpoint

- [X] T046 [US3] Implement POST /chat/selected-text endpoint (FR-002)
- [X] T047 [US3] Add selected text query processing to the orchestrator
- [X] T048 [US3] Ensure 100% grounding in provided text without external retrieval (SC-006)

---

## Phase 6: User Story 4 - Conversational Memory & History (Priority: P4)

### Goal
As a student having an ongoing conversation with the chatbot, I want to maintain context across multiple questions in a session, so that I can have natural, flowing discussions about the textbook content.

### Independent Test Criteria
Can be fully tested by maintaining a session_id across multiple queries and verifying that the system maintains context and can reference previous exchanges, delivering the capability for contextual conversations.

### Implementation Tasks

#### History Management

- [X] T049 [US4] Implement GET /chat/history/{session_id} endpoint (FR-010)
- [X] T050 [US4] Enhance session_service.py to maintain conversation context
- [X] T051 [US4] Add context awareness to the RAG orchestrator for multi-turn conversations

#### Feedback System

- [X] T052 [US4] Implement POST /chat/feedback endpoint (FR-011)
- [X] T053 [US4] Add feedback recording and storage in Neon Postgres
- [X] T054 [US4] Implement token usage logging for operational monitoring (FR-012)

---

## Phase 7: Polish & Cross-Cutting Concerns

### Testing

- [X] T055 [P] Create unit tests for services/retrieval_service.py components
- [X] T056 [P] Create unit tests for services/agent_service.py components
- [X] Create unit tests for services/session_service.py components
- [X] Create unit tests for api/chat.py endpoints
- [X] Create integration tests for API endpoints with database
- [X] Create contract tests for agent services with Gemini API

### Performance & Optimization

- [X] T061 Optimize response times to meet 10-second target (SC-001)
- [X] T062 Implement proper grounding validation to achieve 98% accuracy (SC-004)
- [X] T063 Add performance monitoring to track response times and success rates
- [X] T064 Implement proper citation inclusion to achieve 90% target (SC-002)

### Error Handling & Validation

- [X] T065 Handle Qdrant Cloud unavailability gracefully
- [X] T066 Handle Gemini API rate limiting and unavailability
- [X] T067 Handle Neon Postgres database connection issues
- [X] T068 Validate responses are properly grounded in retrieved content (FR-015)

### Edge Case Handling

- [X] T069 Handle very long user queries that exceed token limits
- [X] T070 Handle queries when no relevant content is found in vector database
- [X] T071 Handle concurrent users accessing the API simultaneously
- [X] T072 Handle different languages in queries vs. textbook content

### Final Validation

- [X] T073 Verify 95% of queries respond within 10 seconds (SC-001)
- [X] T074 Verify 90% of responses include proper source citations (SC-002)
- [X] T075 Verify 85% conversational context accuracy across 10+ exchanges (SC-003)
- [X] T076 Verify 98% of responses are properly grounded (SC-004)
- [X] T077 Verify system handles 100 concurrent users (SC-005)
- [X] T078 Verify selected-text mode responses are 100% grounded (SC-006)