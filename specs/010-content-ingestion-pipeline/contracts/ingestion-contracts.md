# Ingestion Pipeline Contracts

**Feature**: 010-content-ingestion-pipeline
**Date**: 2025-12-25

## Content Loader Interface

### Load Docusaurus Markdown Content

**Input**:
- `source_path` (string): Path to the Docusaurus docs directory
- `file_patterns` (list): Patterns to match markdown files (default: ["**/*.md", "**/*.mdx"])

**Output**:
- `content_files` (list): List of ContentFile objects with path, content, and metadata

**Error Conditions**:
- Source path does not exist or is not readable
- No matching files found in the source path

## Content Chunker Interface

### Chunk Content

**Input**:
- `content` (string): Raw content to be chunked
- `max_tokens` (int): Maximum number of tokens per chunk (default: 800)
- `min_tokens` (int): Minimum number of tokens per chunk (default: 500)
- `overlap_ratio` (float): Ratio of overlap between chunks (default: 0.2)

**Output**:
- `chunks` (list): List of ContentChunk objects with text and metadata

**Error Conditions**:
- Content is empty or invalid
- Unable to tokenize content properly

## Embedder Interface

### Generate Embeddings

**Input**:
- `texts` (list): List of text chunks to embed
- `model_name` (string): Name of the Cohere embedding model

**Output**:
- `embeddings` (list): List of Embedding objects with vectors and metadata

**Error Conditions**:
- Cohere API is unavailable
- Invalid API key
- Rate limit exceeded
- Texts contain content that violates API terms

## Storage Interface

### Store Content Segments

**Input**:
- `content_segments` (list): List of ContentSegment objects
- `collection_name` (string): Name of the Qdrant collection

**Output**:
- `success` (bool): Whether storage was successful
- `stored_count` (int): Number of segments successfully stored

**Error Conditions**:
- Qdrant Cloud is unavailable
- Invalid API key or endpoint
- Database connection issues

### Store Metadata

**Input**:
- `metadata_records` (list): List of Metadata objects

**Output**:
- `success` (bool): Whether storage was successful
- `stored_count` (int): Number of records successfully stored

**Error Conditions**:
- Neon Postgres is unavailable
- Invalid database credentials
- Database schema issues

## Orchestrator Interface

### Run Ingestion Pipeline

**Input**:
- `source_path` (string): Path to Docusaurus docs directory
- `collection_name` (string): Name of Qdrant collection to use
- `reprocess_updates` (bool): Whether to reprocess updated content (default: true)

**Output**:
- `run_id` (string): Unique identifier for this ingestion run
- `summary` (dict): Summary statistics for the run

**Error Conditions**:
- Any of the underlying components fail
- Critical configuration is missing