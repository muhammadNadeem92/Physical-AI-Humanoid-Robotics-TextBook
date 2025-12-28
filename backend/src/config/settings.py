"""
Configuration management for the RAG Chatbot API.

This module handles configuration using python-dotenv to load environment variables
from .env files and provides centralized access to all configuration values.
"""

from pydantic_settings import BaseSettings


class Settings(BaseSettings):
    """
    Application settings loaded from environment variables.
    """
    # Database settings
    database_url: str = ""

    # Qdrant settings
    qdrant_url: str = ""
    qdrant_api_key: str = ""
    qdrant_collection_name: str = "textbook_content"

    # LLM settings
    gemini_api_key: str = ""
    gemini_model: str = "gemini-1.5-pro"

    # OpenAI settings for Agent SDK
    openai_api_key: str = ""

    # Cohere settings for embeddings
    cohere_api_key: str = ""
    embedding_model: str = "embed-multilingual-v3.0"

    # Application settings
    app_name: str = "RAG Chatbot API"
    app_version: str = "1.0.0"
    debug: bool = False
    api_prefix: str = "/chat"
    allowed_origins: str = "*"

    # Logging settings
    log_level: str = "INFO"

    # Performance settings
    max_tokens: int = 2000
    timeout_seconds: int = 30
    max_concurrent_requests: int = 10

    # RAG settings
    retrieval_top_k: int = 5
    similarity_threshold: float = 0.5

    # Session settings
    session_ttl_seconds: int = 3600  # 1 hour

    class Config:
        env_file = ".env"
        env_file_encoding = 'utf-8'
        extra = "ignore"  # This will ignore extra environment variables


# Create a singleton instance of settings
settings = Settings()