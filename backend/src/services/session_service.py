"""
Session service for the RAG Chatbot API.

This module provides functions for managing chat sessions,
including creation, retrieval, and updating of session state.
"""

from datetime import datetime, timedelta
from typing import Optional, Dict, Any
from ..models.chat_session import ChatSession
from ..config.settings import settings
import logging
import asyncio
from collections import defaultdict


class SessionService:
    """
    Service class for handling chat session management.
    In a production system, this would interface with a database,
    but for this implementation we'll use an in-memory store.
    """

    def __init__(self):
        self.sessions: Dict[str, ChatSession] = {}
        self.session_ttl = timedelta(seconds=settings.session_ttl_seconds)

    async def create_session(
        self,
        session_id: str,
        metadata: Optional[Dict[str, Any]] = None,
        user_id: Optional[str] = None
    ) -> ChatSession:
        """
        Create a new chat session.

        Args:
            session_id: Unique identifier for the session
            metadata: Optional metadata for the session
            user_id: Optional user identifier

        Returns:
            Created ChatSession object
        """
        try:
            expires_at = datetime.utcnow() + self.session_ttl

            session = ChatSession(
                session_id=session_id,
                created_at=datetime.utcnow(),
                updated_at=datetime.utcnow(),
                expires_at=expires_at,
                metadata=metadata,
                is_active=True,
                user_id=user_id
            )

            self.sessions[session_id] = session
            logging.info(f"Created new session: {session_id}")
            return session

        except Exception as e:
            logging.error(f"Error creating session {session_id}: {str(e)}")
            raise

    async def get_session(self, session_id: str) -> Optional[ChatSession]:
        """
        Retrieve an existing chat session.

        Args:
            session_id: Unique identifier for the session

        Returns:
            ChatSession object if found, None otherwise
        """
        try:
            session = self.sessions.get(session_id)

            if session:
                # Check if session has expired
                if session.expires_at and session.expires_at < datetime.utcnow():
                    await self.expire_session(session_id)
                    return None

                # Update the last accessed time
                session.updated_at = datetime.utcnow()
                self.sessions[session_id] = session

            return session

        except Exception as e:
            logging.error(f"Error retrieving session {session_id}: {str(e)}")
            raise

    async def update_session(self, session_id: str, metadata: Optional[Dict[str, Any]] = None) -> Optional[ChatSession]:
        """
        Update an existing chat session.

        Args:
            session_id: Unique identifier for the session
            metadata: Optional metadata to update

        Returns:
            Updated ChatSession object if found, None otherwise
        """
        try:
            session = await self.get_session(session_id)
            if not session:
                return None

            # Update metadata if provided
            if metadata is not None:
                if session.metadata is None:
                    session.metadata = {}
                session.metadata.update(metadata)

            # Update the last updated time
            session.updated_at = datetime.utcnow()
            self.sessions[session_id] = session

            logging.info(f"Updated session: {session_id}")
            return session

        except Exception as e:
            logging.error(f"Error updating session {session_id}: {str(e)}")
            raise

    async def expire_session(self, session_id: str) -> bool:
        """
        Mark a session as expired.

        Args:
            session_id: Unique identifier for the session

        Returns:
            True if session was found and expired, False otherwise
        """
        try:
            session = self.sessions.get(session_id)
            if session:
                session.is_active = False
                session.expires_at = datetime.utcnow()
                logging.info(f"Expired session: {session_id}")
                return True

            return False

        except Exception as e:
            logging.error(f"Error expiring session {session_id}: {str(e)}")
            raise

    async def cleanup_expired_sessions(self):
        """
        Remove expired sessions from memory.
        """
        try:
            current_time = datetime.utcnow()
            expired_sessions = [
                session_id for session_id, session in self.sessions.items()
                if session.expires_at and session.expires_at < current_time
            ]

            for session_id in expired_sessions:
                del self.sessions[session_id]

            if expired_sessions:
                logging.info(f"Cleaned up {len(expired_sessions)} expired sessions")

        except Exception as e:
            logging.error(f"Error cleaning up expired sessions: {str(e)}")
            raise


# Global session service instance
session_service = SessionService()