"""
History Manager Service

Manages conversation history for chat sessions, including storage and retrieval.
"""
import asyncio
import logging
from typing import List, Dict, Any, Optional
from datetime import datetime

from src.models.user_query import UserQuery
from src.models.chat_response import ChatResponse
from src.models.feedback import Feedback
from src.config.settings import settings


logger = logging.getLogger(__name__)


class HistoryManager:
    """
    Manages conversation history for chat sessions.
    """

    def __init__(self):
        # In-memory history store (for development)
        # In production, this would use a database
        self.history: Dict[str, List[Dict[str, Any]]] = {}
        self.feedback: Dict[str, Feedback] = {}

    async def add_interaction(
        self,
        session_id: str,
        query: UserQuery,
        response: ChatResponse
    ) -> bool:
        """
        Add a query-response interaction to the session history.

        Args:
            session_id: Session identifier
            query: UserQuery object
            response: ChatResponse object

        Returns:
            bool indicating success
        """
        try:
            if session_id not in self.history:
                self.history[session_id] = []

            interaction = {
                "query_id": query.id,
                "response_id": response.id,
                "question": query.query_text,
                "answer": response.response_text,
                "timestamp": query.timestamp,
                "query_type": query.query_type,
                "citations": response.citations
            }

            self.history[session_id].append(interaction)

            # Limit history size to prevent memory issues
            if len(self.history[session_id]) > settings.session.max_history_length:
                self.history[session_id] = self.history[session_id][-settings.session.max_history_length:]

            logger.info(f"Added interaction to session {session_id}")
            return True

        except Exception as e:
            logger.error(f"Error adding interaction to history: {e}")
            return False

    async def get_history(self, session_id: str) -> List[Dict[str, Any]]:
        """
        Retrieve conversation history for a session.

        Args:
            session_id: Session identifier

        Returns:
            List of conversation interactions
        """
        return self.history.get(session_id, [])

    async def clear_history(self, session_id: str) -> bool:
        """
        Clear conversation history for a session.

        Args:
            session_id: Session identifier

        Returns:
            bool indicating success
        """
        try:
            if session_id in self.history:
                del self.history[session_id]
                logger.info(f"Cleared history for session {session_id}")
            return True
        except Exception as e:
            logger.error(f"Error clearing history: {e}")
            return False

    async def add_feedback(self, feedback: Feedback) -> bool:
        """
        Add feedback for a query-response pair.

        Args:
            feedback: Feedback object

        Returns:
            bool indicating success
        """
        try:
            feedback_key = f"{feedback.session_id}_{feedback.query_id}_{feedback.response_id}"
            self.feedback[feedback_key] = feedback
            logger.info(f"Added feedback for session {feedback.session_id}")
            return True
        except Exception as e:
            logger.error(f"Error adding feedback: {e}")
            return False

    async def get_feedback(
        self,
        session_id: str,
        query_id: Optional[str] = None,
        response_id: Optional[str] = None
    ) -> List[Feedback]:
        """
        Retrieve feedback for a session or specific query-response pair.

        Args:
            session_id: Session identifier
            query_id: Optional query identifier
            response_id: Optional response identifier

        Returns:
            List of Feedback objects
        """
        feedback_list = []
        for feedback_key, feedback in self.feedback.items():
            if feedback.session_id == session_id:
                if query_id and feedback.query_id != query_id:
                    continue
                if response_id and feedback.response_id != response_id:
                    continue
                feedback_list.append(feedback)
        return feedback_list

    async def get_session_stats(self, session_id: str) -> Dict[str, Any]:
        """
        Get statistics for a session.

        Args:
            session_id: Session identifier

        Returns:
            Dictionary with session statistics
        """
        history = await self.get_history(session_id)
        feedback_list = await self.get_feedback(session_id)

        total_queries = len(history)
        total_feedback = len(feedback_list)
        positive_feedback = len([f for f in feedback_list if f.feedback_type in ["positive", "thumbs_up"]])
        negative_feedback = len([f for f in feedback_list if f.feedback_type in ["negative", "thumbs_down"]])

        stats = {
            "session_id": session_id,
            "total_queries": total_queries,
            "total_feedback": total_feedback,
            "positive_feedback": positive_feedback,
            "negative_feedback": negative_feedback,
            "last_activity": max([h["timestamp"] for h in history]) if history else None
        }

        return stats