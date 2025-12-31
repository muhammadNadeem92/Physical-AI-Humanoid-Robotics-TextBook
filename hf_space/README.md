---
title: Physical AI & Humanoid Robotics Textbook RAG Chatbot API
emoji: 🤖
colorFrom: blue
colorTo: green
sdk: docker
pinned: false
license: mit
---

This is a Retrieval-Augmented Generation (RAG) chatbot API for the Physical AI & Humanoid Robotics Textbook, deployed on Hugging Face Spaces.

## About

This FastAPI-based backend provides:
- Chat query endpoints for the textbook content
- Session management for conversation continuity
- Content retrieval from vector database
- AI-powered response generation using Google Gemini
- Citation system for source references
- Feedback collection for response quality

The API is built with FastAPI and runs on port 7860.

## Environment Variables

To run this Space, you need to set the following secrets in your Hugging Face Space settings:

- `COHERE_API_KEY`: Your Cohere API key for embeddings
- `QDRANT_API_KEY`: Your Qdrant Cloud API key
- `QDRANT_URL`: Your Qdrant Cloud URL
- `GEMINI_API_KEY`: Your Google Gemini API key
- `DATABASE_URL`: Your PostgreSQL database URL
- `OPENAI_API_KEY`: Your OpenAI API key (optional)

## API Endpoints

- `GET /` - API root with information
- `POST /chat/query` - Global book content queries
- `POST /chat/selected-text` - Contextual queries about selected text
- `GET /chat/history/{session_id}` - Retrieve conversation history
- `POST /chat/feedback` - Submit feedback on responses
- `GET /health` - Health check endpoint
- `GET /docs` - Interactive API documentation (Swagger UI)

## Architecture

- **FastAPI**: Modern Python web framework with automatic API documentation
- **Qdrant**: Vector database for semantic content retrieval
- **Google Gemini**: AI model for response generation
- **PostgreSQL**: Database for session and metadata storage
- **Cohere**: Embedding model for content representation