# Implementation Plan: Embedded RAG Chatbot – Part 2: RAG Orchestration & Chat APIs

**Branch**: `011-rag-orchestration-api` | **Date**: 2025-12-25 | **Spec**: [link](../011-rag-orchestration-api/spec.md)
**Input**: Feature specification from `/specs/[###-feature-name]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of a backend RAG orchestration and chat API layer for an embedded chatbot used inside a Docusaurus-based textbook on Physical AI & Humanoid Robotics. The system will accept user queries via REST APIs, retrieve relevant content from Qdrant Cloud using Cohere embeddings, process them through a Gemini-based Agent SDK, and return contextual responses with source citations. The system supports global book queries, scoped content queries, selected text queries, and maintains conversational memory per session.

## Technical Context

**Language/Version**: Python 3.11+ (as specified in requirements)
**Primary Dependencies**:
- fastapi (for HTTP API endpoints and streaming responses)
- openai (for Agent SDK orchestration)
- google-generativeai (for Gemini LLM integration)
- qdrant-client (for vector retrieval from Qdrant Cloud)
- psycopg2-binary or asyncpg (for Neon Postgres database operations)
- python-dotenv (for environment variable management)
- uv for project management (as specified in requirements)
- pydantic (for data validation and models)
- sse-starlette (for Server-Sent Events streaming)
- cohere (for embedding operations if needed)
**Storage**: Qdrant Cloud (vector database for content chunks), Neon Postgres (chat sessions, queries, feedback, logs)
**Testing**: pytest for unit and integration tests with proper test coverage
**Target Platform**: Linux server environment with Python 3.11+ (serverless-compatible for deployment)
**Project Type**: Backend API service with FastAPI framework
**Performance Goals**: Respond to 95% of queries within 10 seconds (SC-001), maintain 85% conversational context accuracy across 10+ exchanges (SC-003), handle 100 concurrent users without degradation (SC-005)
**Constraints**:
- No hallucinations: 98% of responses properly grounded in retrieved content (SC-004)
- Selected-text mode: 100% grounding in provided text without external retrieval (SC-006)
- Proper source citations in 90% of responses (SC-002)
- No frontend UI, no markdown ingestion, no embedding generation (from spec)
- FastAPI for HTTP APIs (from spec)
**Scale/Scope**: Support 100 concurrent users, handle various query types (global, scoped, selected text), maintain session-based conversational memory

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Based on the project constitution, the following checks pass:
- ✅ **Spec-Driven Workflow**: Following spec-first approach as outlined in constitution (Core Principle #1)
- ✅ **Security & Configuration Management**: Using python-dotenv for environment variable management to avoid hard-coding secrets (Core Principle #5)
- ✅ **Quality & Build Integrity**: Planning to implement proper testing with pytest (Core Principle #6)
- ✅ **Backend Standards**: Using FastAPI, OpenAI Agents/ChatKit SDK, Qdrant Cloud, and Neon Postgres as specified in constitution (Technical Standard #2)
- ✅ **Separation of Concerns**: This RAG API backend is separate from the frontend Docusaurus site (Core Principle #2)
- ✅ **Context Awareness**: Supporting both global search and contextual search based on user-selected text (Core Principle #4)
- ✅ **No API keys in code**: Planning to use environment variables for Qdrant, Neon, OpenAI, and Google API keys (Core Principle #5)
- ✅ **Performance Goals**: Meeting the specified response time and concurrency goals
- ✅ **Reliability**: Design includes error handling and fallback mechanisms
- ✅ **Scalability**: Supporting the expected concurrent user load

## Project Structure

### Documentation (this feature)

```text
specs/011-rag-orchestration-api/
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
├── api/
│   ├── __init__.py
│   ├── chat.py          # Chat API endpoints (query, selected-text, history, feedback)
│   └── deps.py          # Dependency injection and authentication
├── models/
│   ├── __init__.py
│   ├── chat_session.py  # ChatSession data model
│   ├── user_query.py    # UserQuery data model
│   ├── retrieved_chunk.py # RetrievedChunk data model
│   ├── chat_response.py # ChatResponse data model
│   ├── feedback.py      # Feedback data model
│   └── token_usage_log.py # TokenUsageLog data model
├── services/
│   ├── __init__.py
│   ├── rag_orchestrator.py # Main RAG orchestration service
│   ├── query_processor.py # Query processing and scope determination
│   ├── retrieval_service.py # Vector retrieval from Qdrant
│   ├── agent_service.py # Agent SDK integration with Gemini
│   ├── session_service.py # Session management
│   ├── history_service.py # Conversation history management
│   └── feedback_service.py # Feedback processing
├── config/
│   ├── __init__.py
│   └── settings.py      # Configuration and settings management
├── utils/
│   ├── __init__.py
│   ├── streaming.py     # Streaming response utilities
│   └── validation.py    # Input validation utilities
└── main.py              # FastAPI application entry point

tests/
├── unit/
│   ├── __init__.py
│   └── test_services/   # Unit tests for services
├── integration/
│   ├── __init__.py
│   └── test_api/        # Integration tests for API endpoints
└── contract/
    ├── __init__.py
    └── test_agents/     # Contract tests for agent services

pyproject.toml              # Project dependencies and build config
README.md                   # Project documentation
```

**Structure Decision**: Backend API service structure with clear separation of concerns. The project is organized into logical modules (api, models, services, config, utils) with corresponding test directories. This structure supports the requirements of handling chat queries, managing sessions, retrieving content, and orchestrating the RAG process while maintaining clean architecture and testability.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |