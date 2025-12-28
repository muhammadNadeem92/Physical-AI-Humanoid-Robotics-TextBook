"""
Qdrant configuration for the RAG Chatbot API.

This module handles the Qdrant Cloud connection setup
and provides centralized access to Qdrant configuration values.
"""

from qdrant_client import QdrantClient
from .settings import settings


class QdrantConfig:
    """
    Qdrant configuration and client management.
    """

    def __init__(self):
        self.qdrant_url = settings.qdrant_url
        self.qdrant_api_key = settings.qdrant_api_key
        self.collection_name = settings.qdrant_collection_name
        self.client: QdrantClient = None

    def init_client(self):
        """
        Initialize the Qdrant client.
        """
        if not self.qdrant_url or not self.qdrant_api_key:
            raise ValueError("QDRANT_URL and QDRANT_API_KEY environment variables must be set")

        self.client = QdrantClient(
            url=self.qdrant_url,
            api_key=self.qdrant_api_key,
            # Set timeout to match the application's timeout
            timeout=30
        )

    def get_client(self) -> QdrantClient:
        """
        Get the Qdrant client instance.
        """
        if not self.client:
            self.init_client()

        return self.client


# Global Qdrant config instance
qdrant_config = QdrantConfig()