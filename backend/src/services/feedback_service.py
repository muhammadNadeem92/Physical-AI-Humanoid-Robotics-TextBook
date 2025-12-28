"""
Feedback service for the RAG Chatbot API.

This module provides functions for processing and storing user feedback
on chat responses.
"""

from datetime import datetime
from typing import Optional, Dict, Any
from ..models.feedback import Feedback, FeedbackType
from ..models.chat_session import ChatSession
from ..services.session_service import session_service
import logging
from collections import defaultdict


class FeedbackService:
    """
    Service class for handling feedback processing and storage.
    In a production system, this would interface with a database,
    but for this implementation we'll use an in-memory store.
    """

    def __init__(self):
        self.feedback_store: Dict[str, Feedback] = {}
        self.session_feedback: Dict[str, list] = defaultdict(list)

    async def submit_feedback(self, feedback: Feedback) -> Dict[str, Any]:
        """
        Submit feedback for a chat response.

        Args:
            feedback: The feedback object to submit

        Returns:
            Dictionary with success status and feedback ID
        """
        try:
            # Validate that the session, query, and response exist
            session = await session_service.get_session(feedback.session_id)
            if not session:
                return {
                    "success": False,
                    "error": f"Session {feedback.session_id} not found or expired"
                }

            # Store the feedback
            self.feedback_store[feedback.feedback_id] = feedback
            self.session_feedback[feedback.session_id].append(feedback.feedback_id)

            logging.info(f"Feedback submitted for session {feedback.session_id}, response {feedback.response_id}")
            return {
                "success": True,
                "feedback_id": feedback.feedback_id
            }

        except Exception as e:
            logging.error(f"Error submitting feedback: {str(e)}")
            return {
                "success": False,
                "error": str(e)
            }

    async def get_feedback(self, feedback_id: str) -> Optional[Feedback]:
        """
        Retrieve feedback by ID.

        Args:
            feedback_id: The ID of the feedback to retrieve

        Returns:
            Feedback object if found, None otherwise
        """
        try:
            return self.feedback_store.get(feedback_id)

        except Exception as e:
            logging.error(f"Error retrieving feedback {feedback_id}: {str(e)}")
            return None

    async def get_session_feedback(self, session_id: str) -> list:
        """
        Retrieve all feedback for a session.

        Args:
            session_id: The session ID to retrieve feedback for

        Returns:
            List of feedback IDs for the session
        """
        try:
            return self.session_feedback.get(session_id, [])

        except Exception as e:
            logging.error(f"Error retrieving feedback for session {session_id}: {str(e)}")
            return []

    async def get_feedback_summary(self, session_id: str) -> Dict[str, Any]:
        """
        Get a summary of feedback for a session.

        Args:
            session_id: The session ID to get feedback summary for

        Returns:
            Dictionary with feedback summary statistics
        """
        try:
            feedback_ids = await self.get_session_feedback(session_id)
            feedback_list = [self.feedback_store[fid] for fid in feedback_ids if fid in self.feedback_store]

            # Count feedback by type
            type_counts = defaultdict(int)
            for feedback in feedback_list:
                type_counts[feedback.feedback_type.value] += 1

            summary = {
                "session_id": session_id,
                "total_feedback": len(feedback_list),
                "feedback_types": dict(type_counts),
                "recent_feedback": [
                    {
                        "feedback_id": f.feedback_id,
                        "type": f.feedback_type.value,
                        "timestamp": f.timestamp.isoformat(),
                        "has_text": bool(f.feedback_text)
                    }
                    for f in sorted(feedback_list, key=lambda x: x.timestamp, reverse=True)[:10]  # Last 10
                ]
            }

            return summary

        except Exception as e:
            logging.error(f"Error getting feedback summary for session {session_id}: {str(e)}")
            return {
                "session_id": session_id,
                "total_feedback": 0,
                "feedback_types": {},
                "recent_feedback": []
            }

    async def validate_feedback(self, feedback: Feedback) -> Dict[str, list]:
        """
        Validate feedback before storing.

        Args:
            feedback: The feedback object to validate

        Returns:
            Dictionary with validation errors if any
        """
        errors = []

        # Check if required fields are present
        if not feedback.session_id:
            errors.append("session_id is required")

        if not feedback.query_id:
            errors.append("query_id is required")

        if not feedback.response_id:
            errors.append("response_id is required")

        if not feedback.feedback_type:
            errors.append("feedback_type is required")

        # Check if feedback text is too long (optional validation)
        if feedback.feedback_text and len(feedback.feedback_text) > 1000:
            errors.append("feedback_text is too long (max 1000 characters)")

        return {"errors": errors}


# Global feedback service instance
feedback_service = FeedbackService()