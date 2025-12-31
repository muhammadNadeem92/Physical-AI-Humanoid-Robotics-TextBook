# Quickstart Guide: Embedded RAG Chatbot – RAG Orchestration & Chat APIs

**Feature**: 011-rag-orchestration-api
**Date**: 2025-12-25

## Prerequisites

- Python 3.11+
- uv package manager
- Access to Qdrant Cloud (API key and endpoint)
- Access to Neon Postgres (connection string)
- Access to Google Gemini API (API key)
- Access to OpenAI API (API key for Agent SDK)

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
   QDRANT_API_KEY=your_qdrant_api_key
   QDRANT_URL=your_qdrant_cloud_endpoint
   DATABASE_URL=your_neon_postgres_connection_string
   GEMINI_API_KEY=your_gemini_api_key
   OPENAI_API_KEY=your_openai_api_key
   ```

4. **Initialize the database**
   ```bash
   uv run python -m src.init_db
   ```

## Running the API Server

1. **Start the FastAPI server**
   ```bash
   uv run uvicorn src.main:app --host 0.0.0.0 --port 8000 --reload
   ```

2. **Access the API documentation**
   Open your browser to `http://localhost:8000/docs` to see the interactive API documentation

## Testing the API

1. **Test a global book query**
   ```bash
   curl -X POST http://localhost:8000/chat/query \
     -H "Content-Type: application/json" \
     -d '{
       "question": "What are the key principles of humanoid robotics?",
       "session_id": "test-session-123"
     }'
   ```

2. **Test a selected text query**
   ```bash
   curl -X POST http://localhost:8000/chat/selected-text \
     -H "Content-Type: application/json" \
     -d '{
       "selected_text": "Humanoid robots are robots with physical characteristics resembling humans.",
       "question": "What does this mean?",
       "session_id": "test-session-123"
     }'
   ```

3. **View conversation history**
   ```bash
   curl http://localhost:8000/chat/history/test-session-123
   ```

## Verify the Setup

1. **Check that the API endpoints are responding**
   - Access the `/docs` endpoint to verify FastAPI is running
   - Test the `/chat/query` endpoint with a simple query

2. **Verify database connectivity**
   - Check that chat sessions are being stored in Neon Postgres
   - Verify that retrieved chunks are being logged

3. **Verify Qdrant connectivity**
   - Check that vector retrieval is working by testing a query
   - Verify that relevant content chunks are being returned

## Troubleshooting

- **API key errors**: Ensure all API keys are correctly set in the `.env` file
- **Connection errors**: Verify network connectivity to Qdrant Cloud and Neon Postgres
- **Streaming errors**: Check that the client supports Server-Sent Events (SSE)
- **Grounding issues**: Verify that retrieved content is properly being injected into prompts