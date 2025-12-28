"""
ChatSession data model for the RAG Chatbot API.

This module defines the ChatSession Pydantic model for representing
chat sessions in the system, including session metadata and state.
"""

from datetime import datetime
from typing import Optional, List
from pydantic import BaseModel, Field, UUID4
from uuid import UUID


class ChatSession(BaseModel):
    """
    Data model representing a chat session.
    """
    session_id: str = Field(
        ...,
        description="Unique identifier for the chat session",
        example="session-12345"
    )
    created_at: datetime = Field(
        default_factory=datetime.utcnow,
        description="Timestamp when the session was created"
    )
    updated_at: datetime = Field(
        default_factory=datetime.utcnow,
        description="Timestamp when the session was last updated"
    )
    expires_at: Optional[datetime] = Field(
        default=None,
        description="Timestamp when the session expires"
    )
    metadata: Optional[dict] = Field(
        default=None,
        description="Additional metadata for the session"
    )
    is_active: bool = Field(
        default=True,
        description="Whether the session is currently active"
    )
    user_id: Optional[str] = Field(
        default=None,
        description="Optional user identifier if authenticated"
    )

    class Config:
        json_schema_extra = {
            "example": {
                "session_id": "session-12345",
                "created_at": "2023-10-20T10:00:00Z",
                "updated_at": "2023-10-20T10:00:00Z",
                "expires_at": "2023-10-21T10:00:00Z",
                "metadata": {"source": "textbook_module_1"},
                "is_active": True,
                "user_id": "user-67890"
            }
        }