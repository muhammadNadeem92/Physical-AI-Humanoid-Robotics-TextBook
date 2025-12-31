"""
Test suite for the RAG services.
"""
import pytest
import asyncio
from unittest.mock import Mock, AsyncMock, patch
from datetime import datetime

from src.services.rag_orchestrator import RAGOrchestrator
from src.services.session_manager import SessionManager
from src.services.content_retriever import ContentRetriever
from src.services.response_generator import ResponseGenerator
from src.services.history_manager import HistoryManager
from src.models.chat_session import ChatSession
from src.models.user_query import UserQuery
from src.models.chat_response import ChatResponse
from src.models.retrieved_chunk import RetrievedChunk


class TestSessionManager:
    """Tests for SessionManager service."""

    @pytest.fixture
    def session_manager(self):
        return SessionManager()

    async def test_create_session(self, session_manager):
        """Test creating a new session."""
        session_id = "test-session-123"
        session = await session_manager.create_session(session_id)

        assert session.id == session_id
        assert session.is_active is True
        assert isinstance(session.created_at, datetime)
        assert isinstance(session.last_accessed, datetime)

    async def test_get_or_create_session(self, session_manager):
        """Test getting an existing session or creating a new one."""
        session_id = "test-session-456"

        # Create session
        session1 = await session_manager.get_or_create_session(session_id)
        assert session1.id == session_id

        # Get existing session
        session2 = await session_manager.get_or_create_session(session_id)
        assert session2.id == session_id
        assert session1.id == session2.id

    async def test_close_session(self, session_manager):
        """Test closing a session."""
        session_id = "test-session-789"
        await session_manager.create_session(session_id)

        result = await session_manager.close_session(session_id)
        assert result is True

        # Verify session is removed
        session = await session_manager.get_session(session_id)
        assert session is None


class TestContentRetriever:
    """Tests for ContentRetriever service."""

    @pytest.fixture
    def content_retriever(self):
        # Create a mock client since we can't connect to real services in tests
        mock_qdrant = Mock()
        mock_cohere = Mock()
        return ContentRetriever(mock_qdrant, mock_cohere)

    async def test_retrieve_global(self, content_retriever):
        """Test retrieving global content."""
        # Mock the cohere embedding response
        mock_embedding_response = Mock()
        mock_embedding_response.embeddings = [[0.1, 0.2, 0.3]]
        content_retriever.cohere_client.embed = Mock(return_value=mock_embedding_response)

        # Mock the qdrant search response
        mock_result = Mock()
        mock_result.payload = {"content": "test content", "source_url": "http://example.com"}
        mock_result.score = 0.9
        content_retriever.qdrant_client.search = Mock(return_value=[mock_result])

        results = await content_retriever.retrieve_global("test query")

        assert len(results) == 1
        assert isinstance(results[0], RetrievedChunk)
        assert results[0].content == "test content"


class TestResponseGenerator:
    """Tests for ResponseGenerator service."""

    @pytest.fixture
    def response_generator(self):
        # Create a mock Gemini model
        mock_model = Mock()
        mock_response = Mock()
        mock_response.text = "Test response"
        mock_model.generate_content_async = AsyncMock(return_value=mock_response)

        return ResponseGenerator(mock_model)

    async def test_generate_response(self, response_generator):
        """Test generating a response."""
        chunk = RetrievedChunk(
            content="Test context content",
            metadata={"source": "test_source"}
        )

        response = await response_generator.generate_response(
            question="Test question?",
            context_chunks=[chunk],
            history=[]
        )

        assert response == "Test response"

    async def test_build_prompt(self, response_generator):
        """Test building a prompt with context."""
        chunk = RetrievedChunk(
            content="Test context content",
            metadata={"source_url": "http://example.com"}
        )

        prompt = response_generator._build_prompt(
            question="Test question?",
            context_chunks=[chunk],
            history=[]
        )

        assert "Test question?" in prompt
        assert "Test context content" in prompt
        assert "http://example.com" in prompt


class TestHistoryManager:
    """Tests for HistoryManager service."""

    @pytest.fixture
    def history_manager(self):
        return HistoryManager()

    async def test_add_interaction(self, history_manager):
        """Test adding an interaction to history."""
        session_id = "test-session-123"

        query = UserQuery(
            session_id=session_id,
            query_text="Test question?",
            query_type="global",
            timestamp=datetime.utcnow()
        )

        response = ChatResponse(
            session_id=session_id,
            query_id=query.id,
            response_text="Test response",
            citations=["http://example.com"],
            timestamp=datetime.utcnow()
        )

        result = await history_manager.add_interaction(session_id, query, response)
        assert result is True

        history = await history_manager.get_history(session_id)
        assert len(history) == 1
        assert history[0]["question"] == "Test question?"
        assert history[0]["answer"] == "Test response"

    async def test_get_history_empty(self, history_manager):
        """Test getting history for a session that doesn't exist."""
        history = await history_manager.get_history("nonexistent-session")
        assert len(history) == 0


class TestRAGOrchestrator:
    """Tests for RAGOrchestrator service."""

    @pytest.fixture
    def rag_orchestrator(self):
        # Mock all dependencies
        orchestrator = RAGOrchestrator.__new__(RAGOrchestrator)  # Create without __init__

        orchestrator.session_manager = Mock(spec=SessionManager)
        orchestrator.content_retriever = Mock(spec=ContentRetriever)
        orchestrator.response_generator = Mock(spec=ResponseGenerator)
        orchestrator.history_manager = Mock(spec=HistoryManager)

        # Mock the _validate_connections method to avoid actual connections
        orchestrator._validate_connections = Mock()

        return orchestrator

    async def test_process_global_query(self, rag_orchestrator):
        """Test processing a global query."""
        # Mock session creation
        mock_session = ChatSession(
            id="test-session-123",
            created_at=datetime.utcnow(),
            last_accessed=datetime.utcnow(),
            is_active=True
        )
        rag_orchestrator.session_manager.get_or_create_session = AsyncMock(return_value=mock_session)

        # Mock content retrieval
        mock_chunk = RetrievedChunk(
            content="Test context content",
            metadata={"source_url": "http://example.com"}
        )
        rag_orchestrator.content_retriever.retrieve_global = AsyncMock(return_value=[mock_chunk])

        # Mock history retrieval
        rag_orchestrator.history_manager.get_history = AsyncMock(return_value=[])

        # Mock response generation
        rag_orchestrator.response_generator.generate_response = AsyncMock(return_value="Test response")

        # Mock history storage
        rag_orchestrator.history_manager.add_interaction = AsyncMock(return_value=True)

        response = await rag_orchestrator.process_global_query(
            question="Test question?",
            session_id="test-session-123"
        )

        assert isinstance(response, ChatResponse)
        assert response.response_text == "Test response"
        assert response.citations == ["http://example.com"]


# Run tests if this file is executed directly
if __name__ == "__main__":
    pytest.main([__file__])