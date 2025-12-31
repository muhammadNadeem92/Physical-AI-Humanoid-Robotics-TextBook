---
id: 1
title: rag-orchestration-api
stage: spec
date_iso: 2025-12-25
surface: agent
model: Claude Sonnet 4.5
feature: rag-orchestration-api
branch: 011-rag-orchestration-api
user: user
command: /sp.specify "Embedded RAG Chatbot – Part 2: RAG Orchestration & Chat APIs"
labels:
  - rag
  - chat-api
  - orchestration
  - textbook
links:
  spec: specs/011-rag-orchestration-api/spec.md
  ticket: null
  adr: null
  pr: null
files_yaml:
  - specs/011-rag-orchestration-api/spec.md
  - specs/011-rag-orchestration-api/checklists/requirements.md
tests_yaml:
  - specification creation
  - quality validation
prompt_text: |
  Embedded RAG Chatbot – Part 2: RAG Orchestration & Chat APIs

  Objective:
  Design and implement the backend RAG orchestration and chat API layer for an
  embedded chatbot used inside a Docusaurus-based textbook on Physical AI & Humanoid Robotics.

  This part connects pre-ingested embeddings (from Part 1) to a conversational AI system.

  Scope:
  This part includes ONLY backend runtime logic.
  No frontend UI, no markdown ingestion, and no embedding generation.

  Target Runtime:
  Python 3.11+
  uv-based project
  FastAPI for HTTP APIs

  Core Technologies:
  - FastAPI
  - OpenAI Agent SDK (Agent orchestration)
  - ChatKit-compatible streaming responses
  - Gemini (Google Generative AI) as the LLM provider
  - Qdrant Cloud (vector retrieval)
  - Neon Serverless Postgres (sessions, logs, feedback)

  Responsibilities:
  1. Implement RAG query pipeline:
     - Accept user queries
     - Determine retrieval scope:
       a) entire book
       b) specific module/chapter
       c) user-selected text only
  2. Retrieve relevant chunks from Qdrant using Cohere embeddings
  3. Inject retrieved context into Gemini prompts via Agent SDK
  4. Enforce grounding rules:
     - No response without retrieved context
     - Selected-text mode strictly blocks external retrieval
  5. Support conversational memory per session
  6. Return answers with:
     - cited source snippets
     - page URLs
  7. Stream responses in a ChatKit-compatible format

  Agent Design:
  - Use Agent SDK as the orchestration layer
  - Gemini is used as the LLM behind the Agent
  - Agent responsibilities:
    - Query understanding
    - Tool-based retrieval invocation
    - Context injection
    - Response validation against retrieved chunks

  API Endpoints:
  - POST /chat/query
    Input: question, optional filters (module, chapter), session_id
  - POST /chat/selected-text
    Input: selected_text, question, session_id
  - GET /chat/history/{session_id}
  - POST /chat/feedback

  Database Responsibilities (Neon Postgres):
  - Store chat sessions
  - Store user queries
  - Store retrieved chunk IDs
  - Store feedback (thumbs up/down)
  - Log token usage and
response_text: |
  Created comprehensive specification for the Embedded RAG Chatbot Part 2: RAG Orchestration & Chat APIs feature.
  The specification includes:
  - Four prioritized user stories covering global queries, scoped queries, selected text queries, and conversational memory
  - 15 functional requirements covering all aspects of the RAG orchestration system
  - 6 key entities for data modeling
  - 6 measurable success criteria with specific metrics
  - Edge cases handling for various failure scenarios
  Also created a quality checklist that validates all requirements were met.
outcome: |
  Specification successfully created and validated. Ready for planning phase (/sp.plan).
evaluation: |
  The specification meets all quality criteria and is ready for implementation planning.
---