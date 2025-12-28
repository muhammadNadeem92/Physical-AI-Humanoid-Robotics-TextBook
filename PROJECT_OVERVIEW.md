# Physical AI & Humanoid Robotics Textbook - Project Overview

## Complete Project Structure

This repository contains a complete RAG (Retrieval-Augmented Generation) chatbot system for the Physical AI & Humanoid Robotics textbook, with both backend and frontend components.

### Directory Structure

```
Physical-AI-Humanoid-Robotics-Textbook/
├── backend/                          # RAG Chatbot API Backend
│   ├── src/                          # Backend source code
│   │   ├── api/                      # API route definitions
│   │   ├── config/                   # Configuration management
│   │   ├── models/                   # Data models (Pydantic)
│   │   ├── services/                 # Business logic services
│   │   ├── utils/                    # Utility functions
│   │   └── main.py                   # Main FastAPI application
│   ├── start_server.py               # Server startup script
│   └── README.md                     # Backend documentation
├── frontend/                         # Docusaurus-based Textbook Frontend
│   ├── docs/                         # Textbook content in Markdown
│   ├── src/                          # Frontend source code
│   │   ├── components/               # React components
│   │   ├── contexts/                 # React context providers
│   │   ├── plugins/                  # Docusaurus plugins
│   │   ├── services/                 # Frontend services
│   │   └── utils/                    # Frontend utilities
│   ├── static/                       # Static assets
│   ├── build/                        # Built frontend (generated)
│   ├── node_modules/                 # Node.js dependencies
│   ├── docusaurus.config.js          # Docusaurus configuration
│   ├── sidebars.js                   # Navigation configuration
│   ├── package.json                  # Frontend dependencies
│   └── README.md                     # Frontend documentation
├── specs/                            # Feature Specifications
│   ├── 010-content-ingestion-pipeline/  # Content ingestion spec
│   ├── 011-rag-orchestration-api/       # RAG orchestration spec
│   └── 012-chatkit-docusaurus-integration/ # ChatKit integration spec
├── history/                          # Prompt History Records
│   └── prompts/                      # Claude Code prompt history
├── .env                             # Environment variables
├── .env.example                     # Environment variables template
├── pyproject.toml                   # Python project configuration
├── requirements.txt                 # Python dependencies
├── README.md                        # Main project README
├── PROJECT_OVERVIEW.md              # This file
└── tasks.md                         # Implementation task list
```

## Backend (RAG Chatbot API)

The backend is a FastAPI application that provides:

### Core Features
- **Chat Query Endpoints**: `/chat/query` for global book queries and `/chat/selected-text` for specific text queries
- **Session Management**: Maintains conversation context across page navigations
- **Content Retrieval**: Integrates with Qdrant vector database for semantic search
- **Response Generation**: Uses Google Gemini for AI-powered responses
- **Citation System**: Provides source references for all answers
- **Feedback Collection**: Collects user feedback on response quality

### Architecture
- **FastAPI**: Modern Python web framework with automatic API documentation
- **Qdrant**: Vector database for semantic content retrieval
- **Google Gemini**: AI model for response generation
- **PostgreSQL**: Database for session and metadata storage
- **Cohere**: Embedding model for content representation

### Endpoints
- `GET /` - API root with information
- `POST /chat/query` - Global book content queries
- `POST /chat/selected-text` - Contextual queries about selected text
- `GET /chat/history/{session_id}` - Retrieve conversation history
- `POST /chat/feedback` - Submit feedback on responses
- `GET /health` - Health check endpoint
- `GET /docs` - Interactive API documentation (Swagger UI)

## Frontend (Docusaurus Textbook with Chat Interface)

The frontend is a Docusaurus v3 application with integrated chat functionality:

### Core Features
- **Interactive Textbook**: Complete textbook content in Docusaurus format
- **Embedded Chat Interface**: Floating chat widget using ChatKit UI
- **Text Selection**: "Ask about this selection" functionality
- **Session Continuity**: Maintains conversation state across pages
- **Source Transparency**: Displays citations with clickable links
- **Responsive Design**: Works on desktop and mobile devices

### Architecture
- **Docusaurus v3**: Static site generator with React-based documentation
- **React 18**: Component-based UI with hooks and context
- **TypeScript**: Type-safe frontend development
- **Custom Plugin**: Docusaurus plugin for chat integration
- **CSS Modules**: Encapsulated component styling

### Components
- `ChatWidget`: Main chat interface with floating toggle
- `TextSelectionHandler`: Text selection detection system
- `CitationDisplay`: Source citation rendering
- `SelectionOverlay`: UI for "Ask about this selection" functionality

## Integration

The backend and frontend are integrated through:

### API Communication
- RESTful API endpoints for chat queries
- Server-Sent Events (SSE) for streaming responses
- Session management with persistent session IDs
- CORS-enabled communication

### Plugin System
- Custom Docusaurus plugin for chat widget injection
- Client-side integration with backend API
- Configuration management via Docusaurus config

## Development Setup

### Backend Development
```bash
# Install Python dependencies
pip install -e .

# Start backend server
python -m uvicorn backend.src.main:app --host 0.0.0.0 --port 8000 --reload
```

### Frontend Development
```bash
cd frontend

# Install Node.js dependencies
npm install

# Start development server
npm start
```

### Environment Configuration
Copy `.env.example` to `.env` and configure the required API keys and endpoints.

## Production Deployment

### Backend
Deploy the FastAPI application to a cloud platform (AWS, GCP, Azure, etc.) with appropriate scaling.

### Frontend
Build and deploy the Docusaurus site to a static hosting service (Netlify, Vercel, GitHub Pages, etc.).

## Key Technologies

### Backend
- Python 3.11+
- FastAPI
- Uvicorn ASGI server
- Google Generative AI SDK
- Qdrant Client
- PostgreSQL with asyncpg
- Pydantic for data validation
- OpenAI Agent SDK

### Frontend
- Node.js v18+
- Docusaurus v3
- React 18
- TypeScript
- CSS Modules
- Custom Docusaurus plugin system

## Security Considerations

- Input validation and sanitization
- Rate limiting for API endpoints
- Secure handling of API keys
- CORS configuration for cross-origin requests
- Session management with appropriate timeouts
- Data encryption for sensitive information

## Performance Optimization

- Caching strategies for frequently accessed content
- Asynchronous processing for long-running operations
- Efficient vector search with Qdrant
- Optimized database queries
- CDN for static assets
- Lazy loading for components