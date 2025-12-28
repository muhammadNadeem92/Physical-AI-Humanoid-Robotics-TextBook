# Research Document: Embedded RAG Chatbot – Part 1: Data Ingestion & Embeddings

**Feature**: 010-content-ingestion-pipeline
**Date**: 2025-12-25
**Status**: Completed

## Research Summary

This research document addresses the technical decisions and best practices needed to implement the data ingestion and embedding pipeline for the embedded RAG chatbot.

## Decision: Markdown Parsing for Docusaurus Content

**Rationale**: Docusaurus uses Markdown/MDX files with frontmatter and special syntax. We need a robust parser that can handle Docusaurus-specific features while extracting the core content.

**Approach**: Use the `markdown` library combined with custom parsing for Docusaurus frontmatter. Alternatively, consider using `frontmatter` library to extract YAML frontmatter separately.

**Alternatives considered**:
- Using `pandoc` for more advanced parsing
- Using `mistune` for faster parsing
- Direct file reading with regex (not recommended for complex cases)

## Decision: Text Chunking Strategy

**Rationale**: The specification requires chunking text into 500-800 token segments with overlap. This requires careful consideration of tokenization and overlap strategy to maintain context.

**Approach**: Use `tiktoken` library (from OpenAI) for consistent token counting, then implement a sliding window approach with configurable overlap (e.g., 20% overlap).

**Alternatives considered**:
- Character-based chunking (less accurate)
- Sentence-based chunking (may not meet token requirements)
- Recursive chunking (more complex but potentially better context preservation)

## Decision: Embedding Generation with Cohere

**Rationale**: The specification specifically requires using Cohere Embed (English) for generating embeddings.

**Approach**: Use the official `cohere` Python library to interface with the Cohere API. Implement proper error handling and rate limiting.

**Alternatives considered**:
- OpenAI embeddings (not specified in requirements)
- Hugging Face transformers (self-hosted, but more complex)
- Other embedding providers (not specified in requirements)

## Decision: Qdrant Cloud Integration

**Rationale**: The specification requires storing vectors in Qdrant Cloud (Free Tier).

**Approach**: Use the official `qdrant-client` library to interface with Qdrant Cloud. Implement proper collection management and vector storage/retrieval.

**Alternatives considered**:
- Pinecone (different provider)
- Weaviate (different provider)
- Self-hosted vector databases (not cloud-based)

## Decision: Metadata and Logging Storage

**Rationale**: The specification requires storing metadata and ingestion logs in Neon Postgres.

**Approach**: Use `psycopg2-binary` or `asyncpg` for PostgreSQL connections. Design appropriate table schemas for content metadata and ingestion logs.

**Alternatives considered**:
- Other SQL databases (not specified)
- NoSQL solutions (not specified in requirements)

## Decision: Content Change Detection

**Rationale**: The specification requires supporting re-ingestion on content updates.

**Approach**: Implement content hashing using `hashlib` to detect changes in source files. Store file hashes in the database to compare with current content.

**Alternatives considered**:
- File modification timestamps (less reliable)
- Git-based change detection (requires git integration)
- Manual update flags (not automated)

## Decision: Error Handling Strategy

**Rationale**: The specification requires graceful error handling to continue processing other content when individual files fail.

**Approach**: Implement try-catch blocks at appropriate levels with detailed logging. Use a "continue on error" approach for individual files while tracking errors in the ingestion logs.

**Alternatives considered**:
- Fail-fast approach (stops entire process on any error)
- Batch error handling (collects all errors before reporting)