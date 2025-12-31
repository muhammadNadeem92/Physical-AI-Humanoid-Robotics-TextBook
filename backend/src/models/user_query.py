"""
UserQuery data model for the RAG Chatbot API.

This module defines the UserQuery Pydantic model for representing
user queries in the system, including query text and metadata.
"""

from datetime import datetime
from typing import Optional, List
from pydantic import BaseModel, Field
from enum import Enum


class QueryScope(str, Enum):
    """
    Enumeration of possible query scopes.
    """
    GLOBAL = "global"  # Entire book
    MODULE = "module"  # Specific module
    CHAPTER = "chapter"  # Specific chapter
    SELECTED_TEXT = "selected_text"  # User-selected text only


class UserQuery(BaseModel):
    """
    Data model representing a user query.
    """
    query_id: str = Field(
        ...,
        description="Unique identifier for the query",
        example="query-12345"
    )
    session_id: str = Field(
        ...,
        description="ID of the session this query belongs to",
        example="session-12345"
    )
    question: str = Field(
        ...,
        description="The question text from the user",
        example="What are the key principles of humanoid robotics?"
    )
    scope: QueryScope = Field(
        default=QueryScope.GLOBAL,
        description="The scope of the query (global, module, chapter, selected_text)"
    )
    scope_filter: Optional[str] = Field(
        default=None,
        description="Additional filter for scoped queries (module/chapter name)",
        example="module-01-introduction"
    )
    timestamp: datetime = Field(
        default_factory=datetime.utcnow,
        description="Timestamp when the query was made"
    )
    selected_text: Optional[str] = Field(
        default=None,
        description="Text selected by the user for selected-text queries"
    )
    metadata: Optional[dict] = Field(
        default=None,
        description="Additional metadata for the query"
    )

    class Config:
        json_schema_extra = {
            "example": {
                "query_id": "query-12345",
                "session_id": "session-12345",
                "question": "What are the key principles of humanoid robotics?",
                "scope": "global",
                "scope_filter": None,
                "timestamp": "2023-10-20T10:00:00Z",
                "selected_text": None,
                "metadata": {"source": "frontend"}
            }
        }