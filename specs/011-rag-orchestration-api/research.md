# Research Document: Embedded RAG Chatbot – Part 2: RAG Orchestration & Chat APIs

**Feature**: 011-rag-orchestration-api
**Date**: 2025-12-25
**Status**: Completed

## Research Summary

This research document addresses the technical decisions and best practices needed to implement the RAG orchestration and chat API layer for the embedded chatbot.

## Decision: FastAPI for HTTP Endpoints

**Rationale**: The specification requires FastAPI for HTTP APIs to support the required endpoints and streaming responses.

**Approach**: Use FastAPI with Pydantic models for request/response validation and SSE (Server-Sent Events) for streaming responses in ChatKit-compatible format.

**Alternatives considered**:
- Flask (simpler but less streaming support)
- Django (more complex, not ideal for API-only service)
- Aiohttp (async but less integrated validation)

## Decision: OpenAI Agent SDK Integration

**Rationale**: The specification requires using OpenAI Agent SDK as the orchestration layer with Gemini as the LLM provider.

**Approach**: Since OpenAI Agent SDK typically works with OpenAI models, we'll need to create an abstraction layer that allows the Agent SDK to work with Gemini. This will involve implementing a custom LLM interface that translates between Agent SDK calls and Gemini API calls.

**Alternatives considered**:
- Direct Gemini API calls without Agent SDK (loses orchestration benefits)
- LangChain with Gemini (different orchestration approach)
- Anthropic Claude (different provider than specified)

## Decision: Qdrant Vector Retrieval

**Rationale**: The specification requires retrieving relevant content chunks from Qdrant Cloud using Cohere embeddings.

**Approach**: Use the qdrant-client library to interface with Qdrant Cloud. Implement proper filtering based on scope (entire book, specific module/chapter, or user-selected text only).

**Alternatives considered**:
- Pinecone (different provider)
- Weaviate (different provider)
- Self-hosted vector databases (not cloud-based as specified)

## Decision: ChatKit-Compatible Streaming

**Rationale**: The specification requires streaming responses in a ChatKit-compatible format.

**Approach**: Use Server-Sent Events (SSE) with Starlette's streaming capabilities to provide real-time responses. Structure the response format to be compatible with ChatKit expectations.

**Alternatives considered**:
- WebSocket connections (more complex for simple streaming)
- Regular HTTP responses (no real-time streaming)
- GraphQL subscriptions (overkill for this use case)

## Decision: Session Management

**Rationale**: The specification requires maintaining conversational memory per session.

**Approach**: Implement session management using UUID-based session IDs stored in Neon Postgres. Maintain conversation history and context within each session.

**Alternatives considered**:
- In-memory storage (not persistent across deployments)
- Redis (additional infrastructure complexity)
- JWT tokens (limited storage for conversation history)

## Decision: Grounding Rules Enforcement

**Rationale**: The specification requires enforcing grounding rules (no response without retrieved context, selected-text mode blocks external retrieval).

**Approach**: Implement validation logic that ensures responses are based on retrieved content. For selected-text mode, create a separate processing pipeline that only uses the provided text without external retrieval.

**Alternatives considered**:
- LLM-based validation (less reliable)
- Post-processing checks (may miss violations)
- Manual review (not scalable)

## Decision: Database Schema Design

**Rationale**: The specification requires storing chat sessions, user queries, retrieved chunk IDs, feedback, and token usage logs.

**Approach**: Design normalized PostgreSQL schema with tables for sessions, queries, responses, retrieved chunks, and feedback. Use proper indexing for performance.

**Alternatives considered**:
- NoSQL solutions (less structured than required)
- File-based storage (not suitable for concurrent access)
- In-memory storage (not persistent)

## Decision: Error Handling Strategy

**Rationale**: The system needs to handle various failure scenarios gracefully.

**Approach**: Implement comprehensive error handling with fallback mechanisms, proper logging, and user-friendly error messages. Include retry logic for transient failures.

**Alternatives considered**:
- Fail-fast approach (not resilient)
- Silent error handling (no visibility into issues)
- Basic try-catch (not comprehensive enough)