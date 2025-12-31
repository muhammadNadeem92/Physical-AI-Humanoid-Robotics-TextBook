"""
RetrievedChunk data model for the RAG Chatbot API.

This module defines the RetrievedChunk Pydantic model for representing
content chunks retrieved from the vector database.
"""

from datetime import datetime
from typing import Optional
from pydantic import BaseModel, Field


class RetrievedChunk(BaseModel):
    """
    Data model representing a content chunk retrieved from the vector database.
    """
    chunk_id: str = Field(
        ...,
        description="Unique identifier for the retrieved chunk",
        example="chunk-12345"
    )
    query_id: str = Field(
        ...,
        description="ID of the query that retrieved this chunk",
        example="query-12345"
    )
    content: str = Field(
        ...,
        description="The actual content of the retrieved chunk",
        example="Humanoid robots are robots with physical characteristics resembling humans..."
    )
    page_url: str = Field(
        ...,
        description="URL of the source page where this content appears",
        example="https://example.com/textbook/module-01-introduction"
    )
    module: str = Field(
        ...,
        description="Name of the module containing this content",
        example="module-01-introduction"
    )
    chapter: str = Field(
        ...,
        description="Name of the chapter containing this content",
        example="introduction-to-humanoid-robotics"
    )
    section: str = Field(
        ...,
        description="Name of the section containing this content",
        example="what-is-humanoid-robotics"
    )
    score: float = Field(
        ...,
        description="Similarity score for the retrieved chunk",
        ge=0.0,
        le=1.0,
        example=0.85
    )
    position: int = Field(
        ...,
        description="Position of the chunk in the original document",
        example=1
    )
    timestamp: datetime = Field(
        default_factory=datetime.utcnow,
        description="Timestamp when the chunk was retrieved"
    )
    metadata: Optional[dict] = Field(
        default=None,
        description="Additional metadata for the chunk"
    )

    class Config:
        json_schema_extra = {
            "example": {
                "chunk_id": "chunk-12345",
                "query_id": "query-12345",
                "content": "Humanoid robots are robots with physical characteristics resembling humans...",
                "page_url": "https://example.com/textbook/module-01-introduction",
                "module": "module-01-introduction",
                "chapter": "introduction-to-humanoid-robotics",
                "section": "what-is-humanoid-robotics",
                "score": 0.85,
                "position": 1,
                "timestamp": "2023-10-20T10:00:00Z",
                "metadata": {"source_file": "introduction.md"}
            }
        }