# Physical AI & Humanoid Robotics Textbook - Chat Interface

This repository contains both the RAG Chatbot API backend and the embedded chat interface for the Physical AI & Humanoid Robotics textbook.

## Project Overview

For complete project details, directory structure, and technical specifications, see [PROJECT_OVERVIEW.md](PROJECT_OVERVIEW.md).

## Components

### Backend API (RAG Orchestration)
A RAG (Retrieval-Augmented Generation) Chatbot API built with FastAPI, Qdrant, and Google Gemini.
Located in the `backend/` directory.

### Frontend Chat Interface (ChatKit UI & Docusaurus Integration)
An embedded chatbot interface integrated into the Docusaurus-based textbook with text selection and contextual questioning capabilities.
Located in the `frontend/` directory.

## Overview

This API provides a conversational interface to the Physical AI & Humanoid Robotics textbook content. It uses RAG (Retrieval-Augmented Generation) to provide accurate, context-aware responses with proper citations.

### Features

- **Global Book Queries**: Ask questions about the entire textbook content
- **Scoped Queries**: Limit questions to specific modules or chapters
- **Selected Text Queries**: Ask questions about specific text selections only
- **Conversational Memory**: Maintain context across multiple exchanges in a session
- **Citation Support**: Responses include source citations with page URLs
- **Streaming Responses**: Real-time responses compatible with ChatKit
- **Feedback System**: Users can provide feedback on responses

## Architecture

The system is built with the following components:

- **FastAPI**: Web framework for building the API
- **Qdrant Cloud**: Vector database for content retrieval
- **Google Gemini**: Language model for response generation
- **Cohere**: Embedding model for content representation
- **Neon Postgres**: Database for storing sessions, queries, and feedback
- **Server-Sent Events (SSE)**: For streaming responses

## Setup

### Prerequisites

- Python 3.11+
- uv package manager
- Access to Qdrant Cloud (API key and endpoint)
- Access to Neon Postgres (connection string)
- Access to Google Gemini API (API key)
- Access to OpenAI API (API key for Agent SDK)
- Access to Cohere API (API key for embeddings)

### Installation

1. Clone the repository
2. Install dependencies with uv:
   ```bash
   uv sync
   ```

3. Set up environment variables by copying `.env.example` to `.env` and filling in your credentials:
   ```bash
   cp .env.example .env
   # Edit .env with your actual credentials
   ```

4. Start the API server:
   ```bash
   uv run uvicorn src.main:app --host 0.0.0.0 --port 8000 --reload
   ```

## API Endpoints

### Chat Query
`POST /chat/query`

Query the entire book or specific scope.

Request body:
```json
{
  "question": "What are the key principles of humanoid robotics?",
  "session_id": "session-123",
  "module": "optional-module-name",
  "chapter": "optional-chapter-name"
}
```

### Selected Text Query
`POST /chat/selected-text`

Query specifically about selected text.

Request body:
```json
{
  "selected_text": "Humanoid robots are robots with physical characteristics resembling humans.",
  "question": "What does this mean?",
  "session_id": "session-123"
}
```

### Chat History
`GET /chat/history/{session_id}`

Retrieve conversation history for a session.

### Chat Feedback
`POST /chat/feedback`

Submit feedback on a response.

Request body:
```json
{
  "session_id": "session-123",
  "query_id": "query-456",
  "response_id": "response-789",
  "feedback_type": "thumbs_up",
  "feedback_text": "optional feedback text"
}
```

## Configuration

The application can be configured using environment variables in the `.env` file. See `.env.example` for all available options.

## Project Structure

```
src/
├── api/                 # API endpoints
├── models/              # Data models
├── services/            # Business logic services
├── config/              # Configuration and settings
└── utils/               # Utility functions
```

## Development

To run tests:
```bash
uv run pytest
```

## License

MIT