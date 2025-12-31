"""
TokenUsageLog data model for the RAG Chatbot API.

This module defines the TokenUsageLog Pydantic model for representing
token usage logs for operational monitoring.
"""

from datetime import datetime
from typing import Optional
from pydantic import BaseModel, Field


class TokenUsageLog(BaseModel):
    """
    Data model representing token usage for operational monitoring.
    """
    log_id: str = Field(
        ...,
        description="Unique identifier for the log entry",
        example="log-12345"
    )
    session_id: str = Field(
        ...,
        description="ID of the session this log belongs to",
        example="session-12345"
    )
    query_id: Optional[str] = Field(
        default=None,
        description="ID of the query associated with this log",
        example="query-12345"
    )
    response_id: Optional[str] = Field(
        default=None,
        description="ID of the response associated with this log",
        example="response-12345"
    )
    model_name: str = Field(
        ...,
        description="Name of the model used",
        example="gemini-1.5-pro"
    )
    input_tokens: int = Field(
        ...,
        description="Number of input tokens processed",
        example=150
    )
    output_tokens: int = Field(
        ...,
        description="Number of output tokens generated",
        example=200
    )
    total_tokens: int = Field(
        ...,
        description="Total number of tokens (input + output)",
        example=350
    )
    timestamp: datetime = Field(
        default_factory=datetime.utcnow,
        description="Timestamp when the usage was logged"
    )
    cost_usd: Optional[float] = Field(
        default=None,
        description="Estimated cost in USD (if available)",
        example=0.0025
    )
    metadata: Optional[dict] = Field(
        default=None,
        description="Additional metadata for the usage log"
    )

    class Config:
        json_schema_extra = {
            "example": {
                "log_id": "log-12345",
                "session_id": "session-12345",
                "query_id": "query-12345",
                "response_id": "response-12345",
                "model_name": "gemini-1.5-pro",
                "input_tokens": 150,
                "output_tokens": 200,
                "total_tokens": 350,
                "timestamp": "2023-10-20T10:00:00Z",
                "cost_usd": 0.0025,
                "metadata": {"region": "us-central1"}
            }
        }