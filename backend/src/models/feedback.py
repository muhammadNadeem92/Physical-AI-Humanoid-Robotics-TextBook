"""
Feedback data model for the RAG Chatbot API.

This module defines the Feedback Pydantic model for representing
user feedback on responses.
"""

from datetime import datetime
from typing import Optional
from pydantic import BaseModel, Field
from enum import Enum


class FeedbackType(str, Enum):
    """
    Enumeration of possible feedback types.
    """
    THUMBS_UP = "thumbs_up"
    THUMBS_DOWN = "thumbs_down"
    REPORT_INACCURATE = "report_inaccurate"
    HELPFUL = "helpful"
    NOT_HELPFUL = "not_helpful"


class Feedback(BaseModel):
    """
    Data model representing user feedback on a response.
    """
    feedback_id: str = Field(
        ...,
        description="Unique identifier for the feedback",
        example="feedback-12345"
    )
    session_id: str = Field(
        ...,
        description="ID of the session this feedback belongs to",
        example="session-12345"
    )
    query_id: str = Field(
        ...,
        description="ID of the query this feedback refers to",
        example="query-12345"
    )
    response_id: str = Field(
        ...,
        description="ID of the response this feedback refers to",
        example="response-12345"
    )
    feedback_type: FeedbackType = Field(
        ...,
        description="Type of feedback provided"
    )
    feedback_text: Optional[str] = Field(
        default=None,
        description="Additional feedback text provided by the user",
        example="The response was very helpful and accurate."
    )
    timestamp: datetime = Field(
        default_factory=datetime.utcnow,
        description="Timestamp when the feedback was submitted"
    )
    metadata: Optional[dict] = Field(
        default=None,
        description="Additional metadata for the feedback"
    )

    class Config:
        json_schema_extra = {
            "example": {
                "feedback_id": "feedback-12345",
                "session_id": "session-12345",
                "query_id": "query-12345",
                "response_id": "response-12345",
                "feedback_type": "thumbs_up",
                "feedback_text": "The response was very helpful and accurate.",
                "timestamp": "2023-10-20T10:00:00Z",
                "metadata": {"user_rating": 5}
            }
        }