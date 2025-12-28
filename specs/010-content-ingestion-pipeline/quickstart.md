# Quickstart Guide: Embedded RAG Chatbot – Data Ingestion Pipeline

**Feature**: 010-content-ingestion-pipeline
**Date**: 2025-12-25

## Prerequisites

- Python 3.11+
- uv package manager
- Access to Cohere API (API key)
- Access to Qdrant Cloud (API key and endpoint)
- Access to Neon Postgres (connection string)

## Setup

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. **Install dependencies with uv**
   ```bash
   uv sync
   ```

3. **Set up environment variables**
   Create a `.env` file in the project root:
   ```env
   COHERE_API_KEY=your_cohere_api_key
   QDRANT_API_KEY=your_qdrant_api_key
   QDRANT_URL=your_qdrant_cloud_endpoint
   DATABASE_URL=your_neon_postgres_connection_string
   ```

## Running the Ingestion Pipeline

1. **Run the ingestion pipeline**
   ```bash
   uv run python -m src.cli.main ingest
   ```

2. **To run with specific options**
   ```bash
   uv run python -m src.cli.main ingest --source-path /frontend/docs --collection-name textbook-content
   ```

## Verify the Setup

1. **Check that embeddings were created in Qdrant Cloud**
   - Log into your Qdrant Cloud dashboard
   - Verify the collection exists and contains vectors

2. **Check that metadata and logs were created in Neon Postgres**
   - Connect to your Neon Postgres database
   - Verify the metadata and ingestion log tables contain records

## Troubleshooting

- **API key errors**: Ensure all API keys are correctly set in the `.env` file
- **Connection errors**: Verify network connectivity to Qdrant Cloud and Neon Postgres
- **Token limit errors**: Check that the token counting is working correctly for chunking