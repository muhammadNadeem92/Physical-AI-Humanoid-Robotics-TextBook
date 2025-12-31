"""
History service for the RAG Chatbot API.

This module provides functions for managing conversation history
within chat sessions.
"""

from datetime import datetime
from typing import List, Optional, Dict, Any
from ..models.user_query import UserQuery
from ..models.chat_response import ChatResponse
from ..models.chat_session import ChatSession
from ..services.session_service import session_service
import logging
from collections import defaultdict


class HistoryItem:
    """
    Represents an item in the conversation history.
    """
    def __init__(self, item_id: str, item_type: str, content: str, timestamp: datetime, citations: Optional[List] = None):
        self.id = item_id
        self.type = item_type  # 'user_query' or 'chat_response'
        self.content = content
        self.timestamp = timestamp
        self.citations = citations or []


class HistoryService:
    """
    Service class for handling conversation history management.
    In a production system, this would interface with a database,
    but for this implementation we'll use an in-memory store.
    """

    def __init__(self):
        self.history: Dict[str, List[HistoryItem]] = defaultdict(list)

    async def add_user_query(self, user_query: UserQuery) -> bool:
        """
        Add a user query to the conversation history.

        Args:
            user_query: The user query to add

        Returns:
            True if successfully added, False otherwise
        """
        try:
            history_item = HistoryItem(
                item_id=user_query.query_id,
                item_type='user_query',
                content=user_query.question,
                timestamp=user_query.timestamp
            )

            self.history[user_query.session_id].append(history_item)
            logging.info(f"Added user query to history for session {user_query.session_id}")
            return True

        except Exception as e:
            logging.error(f"Error adding user query to history: {str(e)}")
            return False

    async def add_chat_response(self, chat_response: ChatResponse) -> bool:
        """
        Add a chat response to the conversation history.

        Args:
            chat_response: The chat response to add

        Returns:
            True if successfully added, False otherwise
        """
        try:
            # Convert citations to the format expected by HistoryItem
            citations = [citation.dict() for citation in chat_response.citations]

            history_item = HistoryItem(
                item_id=chat_response.response_id,
                item_type='chat_response',
                content=chat_response.answer,
                timestamp=chat_response.timestamp,
                citations=citations
            )

            self.history[chat_response.session_id].append(history_item)
            logging.info(f"Added chat response to history for session {chat_response.session_id}")
            return True

        except Exception as e:
            logging.error(f"Error adding chat response to history: {str(e)}")
            return False

    async def get_history(self, session_id: str) -> List[Dict[str, Any]]:
        """
        Retrieve the conversation history for a session.

        Args:
            session_id: The session ID to retrieve history for

        Returns:
            List of history items in dictionary format
        """
        try:
            # Check if session exists and is active
            session = await session_service.get_session(session_id)
            if not session:
                logging.warning(f"Session {session_id} not found or expired")
                return []

            # Sort history by timestamp to ensure chronological order
            session_history = self.history[session_id]
            sorted_history = sorted(session_history, key=lambda x: x.timestamp)

            # Convert to dictionary format for API response
            history_list = []
            for item in sorted_history:
                history_item = {
                    'id': item.id,
                    'type': item.type,
                    'content': item.content,
                    'timestamp': item.timestamp.isoformat(),
                }
                if item.citations:
                    history_item['citations'] = item.citations

                history_list.append(history_item)

            logging.info(f"Retrieved history for session {session_id} with {len(history_list)} items")
            return history_list

        except Exception as e:
            logging.error(f"Error retrieving history for session {session_id}: {str(e)}")
            return []

    async def clear_history(self, session_id: str) -> bool:
        """
        Clear the conversation history for a session.

        Args:
            session_id: The session ID to clear history for

        Returns:
            True if successfully cleared, False otherwise
        """
        try:
            if session_id in self.history:
                del self.history[session_id]
                logging.info(f"Cleared history for session {session_id}")
            return True

        except Exception as e:
            logging.error(f"Error clearing history for session {session_id}: {str(e)}")
            return False

    async def get_recent_context(self, session_id: str, max_items: int = 5) -> str:
        """
        Get a string representation of the recent conversation context.

        Args:
            session_id: The session ID to get context for
            max_items: Maximum number of recent items to include

        Returns:
            String representation of recent conversation context
        """
        try:
            history = await self.get_history(session_id)
            if not history:
                return ""

            # Get the most recent items
            recent_items = history[-max_items:]

            context_parts = []
            for item in recent_items:
                if item['type'] == 'user_query':
                    context_parts.append(f"User: {item['content']}")
                elif item['type'] == 'chat_response':
                    context_parts.append(f"Assistant: {item['content']}")

            return "\n\n".join(context_parts)

        except Exception as e:
            logging.error(f"Error getting recent context for session {session_id}: {str(e)}")
            return ""


# Global history service instance
history_service = HistoryService()