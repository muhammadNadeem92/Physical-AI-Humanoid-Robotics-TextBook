# RAG Chatbot API Backend

This directory contains the backend API for the RAG (Retrieval-Augmented Generation) Chatbot for the Physical AI & Humanoid Robotics textbook.

## Overview

The backend is built with FastAPI and provides:
- Chat query endpoints for textbook content
- Session management for conversations
- Content retrieval from Qdrant vector database
- Response generation using Google Gemini
- Citation support with source references

## Project Structure

```
backend/
├── src/                    # Source code
│   ├── api/                # API route definitions
│   ├── config/             # Configuration management
│   ├── models/             # Data models
│   ├── services/           # Business logic services
│   ├── utils/              # Utility functions
│   └── main.py             # Main application entry point
├── start_server.py         # Server startup script
└── README.md               # This file
```

## Requirements

- Python 3.11+
- FastAPI
- Uvicorn
- Google Generative AI SDK
- Qdrant Client
- PostgreSQL driver
- Other dependencies listed in pyproject.toml

## Environment Variables

The backend requires the following environment variables to be set in a `.env` file:

```
# Qdrant Cloud Configuration
QDRANT_URL=your-qdrant-url
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_COLLECTION_NAME=textbook-content

# Google Gemini Configuration
GEMINI_API_KEY=your-gemini-api-key

# OpenAI Configuration (for Agent SDK)
OPENAI_API_KEY=your-openai-api-key

# Cohere Configuration (for embeddings)
COHERE_API_KEY=your-cohere-api-key

# Database Configuration
DATABASE_URL=your-postgres-connection-string

# Application Configuration
DEBUG=true
ALLOWED_ORIGINS=*
```

## Running the Server

### Method 1: Using the start script
```bash
python backend/start_server.py
```

### Method 2: Using uvicorn directly
```bash
cd backend
uvicorn src.main:app --host 0.0.0.0 --port 8000 --reload
```

### Method 3: From project root
```bash
uvicorn backend.src.main:app --host 0.0.0.0 --port 8000 --reload
```

The server will be available at `http://localhost:8000` or `http://0.0.0.0:8000`.

## API Endpoints

- `GET /` - Root endpoint with API info
- `GET /docs` - Swagger UI documentation
- `GET /redoc` - ReDoc documentation
- `POST /chat/query` - Global book queries
- `POST /chat/selected-text` - Selected text queries
- `GET /chat/history/{session_id}` - Chat history
- `POST /chat/feedback` - Submit feedback
- `GET /health` - Health check

## Configuration

The application is configured through environment variables. See `src/config/settings.py` for all available configuration options.