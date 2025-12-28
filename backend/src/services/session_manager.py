"""
Session Manager Service

Handles session creation, retrieval, and management for chat conversations.
"""
import asyncio
import logging
from typing import Dict, Optional
from datetime import datetime, timedelta

from src.models.chat_session import ChatSession
from src.config.settings import settings


logger = logging.getLogger(__name__)


class SessionManager:
    """
    Manages chat sessions, including creation, retrieval, and cleanup.
    """

    def __init__(self):
        # In-memory session store (for development)
        # In production, this would use a database or Redis
        self.sessions: Dict[str, ChatSession] = {}
        self.session_timeout = timedelta(seconds=settings.session.timeout_seconds)

    async def get_or_create_session(self, session_id: str) -> ChatSession:
        """
        Get an existing session or create a new one.

        Args:
            session_id: Unique identifier for the session

        Returns:
            ChatSession object
        """
        if session_id in self.sessions:
            session = self.sessions[session_id]
            # Update last access time
            session.last_accessed = datetime.utcnow()

            # Check if session has expired
            if datetime.utcnow() - session.last_accessed > self.session_timeout:
                await self.close_session(session_id)
                return await self.create_session(session_id)

            return session
        else:
            return await self.create_session(session_id)

    async def create_session(self, session_id: str) -> ChatSession:
        """
        Create a new chat session.

        Args:
            session_id: Unique identifier for the session

        Returns:
            ChatSession object
        """
        session = ChatSession(
            id=session_id,
            created_at=datetime.utcnow(),
            last_accessed=datetime.utcnow(),
            is_active=True
        )
        self.sessions[session_id] = session
        logger.info(f"Created new session: {session_id}")
        return session

    async def get_session(self, session_id: str) -> Optional[ChatSession]:
        """
        Retrieve an existing session.

        Args:
            session_id: Unique identifier for the session

        Returns:
            ChatSession object or None if not found
        """
        if session_id in self.sessions:
            session = self.sessions[session_id]
            # Check if session has expired
            if datetime.utcnow() - session.last_accessed > self.session_timeout:
                await self.close_session(session_id)
                return None
            return session
        return None

    async def close_session(self, session_id: str) -> bool:
        """
        Close and remove a session.

        Args:
            session_id: Unique identifier for the session

        Returns:
            bool indicating success
        """
        if session_id in self.sessions:
            del self.sessions[session_id]
            logger.info(f"Closed session: {session_id}")
            return True
        return False

    async def cleanup_expired_sessions(self):
        """
        Remove expired sessions from memory.
        """
        current_time = datetime.utcnow()
        expired_sessions = [
            session_id for session_id, session in self.sessions.items()
            if current_time - session.last_accessed > self.session_timeout
        ]

        for session_id in expired_sessions:
            del self.sessions[session_id]
            logger.info(f"Cleaned up expired session: {session_id}")