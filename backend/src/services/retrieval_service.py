"""
Retrieval service for the RAG Chatbot API.

This module provides functions to retrieve content chunks from Qdrant
based on user queries and scope filters.
"""

from typing import List, Optional
from qdrant_client.http import models
from ..config.qdrant import qdrant_config
from ..models.user_query import UserQuery, QueryScope
from ..models.retrieved_chunk import RetrievedChunk
from ..config.settings import settings
import logging


class RetrievalService:
    """
    Service class for handling content retrieval from Qdrant.
    """

    def __init__(self):
        self.qdrant_client = qdrant_config.get_client()
        self.collection_name = qdrant_config.collection_name
        self.top_k = settings.retrieval_top_k
        self.similarity_threshold = settings.similarity_threshold

    async def retrieve_chunks(
        self,
        user_query: UserQuery,
        embedding: List[float]
    ) -> List[RetrievedChunk]:
        """
        Retrieve relevant content chunks from Qdrant based on the user query and embedding.

        Args:
            user_query: The user query object containing scope and filters
            embedding: The embedding vector for the query

        Returns:
            List of RetrievedChunk objects
        """
        try:
            # Build query conditions based on scope
            filter_conditions = self._build_filter_conditions(user_query)

            # Perform vector search in Qdrant
            search_results = self.qdrant_client.search(
                collection_name=self.collection_name,
                query_vector=embedding,
                query_filter=filter_conditions,
                limit=self.top_k,
                with_payload=True,
                with_vectors=False,
                score_threshold=self.similarity_threshold
            )

            # Convert search results to RetrievedChunk objects
            retrieved_chunks = []
            for i, hit in enumerate(search_results):
                chunk = RetrievedChunk(
                    chunk_id=hit.id,
                    query_id=user_query.query_id,
                    content=hit.payload.get("content", ""),
                    page_url=hit.payload.get("page_url", ""),
                    module=hit.payload.get("module", ""),
                    chapter=hit.payload.get("chapter", ""),
                    section=hit.payload.get("section", ""),
                    score=hit.score,
                    position=i + 1,  # Position in the search results
                    metadata=hit.payload.get("metadata", {})
                )
                retrieved_chunks.append(chunk)

            logging.info(f"Retrieved {len(retrieved_chunks)} chunks for query {user_query.query_id}")
            return retrieved_chunks

        except Exception as e:
            logging.error(f"Error retrieving chunks for query {user_query.query_id}: {str(e)}")
            raise

    def _build_filter_conditions(self, user_query: UserQuery) -> Optional[models.Filter]:
        """
        Build filter conditions based on the query scope and filters.

        Args:
            user_query: The user query object containing scope and filters

        Returns:
            Qdrant filter conditions or None
        """
        conditions = []

        if user_query.scope == QueryScope.MODULE and user_query.scope_filter:
            conditions.append(
                models.FieldCondition(
                    key="module",
                    match=models.MatchValue(value=user_query.scope_filter)
                )
            )
        elif user_query.scope == QueryScope.CHAPTER and user_query.scope_filter:
            conditions.append(
                models.FieldCondition(
                    key="chapter",
                    match=models.MatchValue(value=user_query.scope_filter)
                )
            )
        elif user_query.scope == QueryScope.SELECTED_TEXT:
            # For selected text mode, we don't retrieve from the database
            # This is handled in the orchestrator
            pass

        if conditions:
            return models.Filter(must=conditions)

        return None

    async def retrieve_chunks_by_text(
        self,
        user_query: UserQuery,
        query_text: str
    ) -> List[RetrievedChunk]:
        """
        Retrieve relevant content chunks from Qdrant based on text query.
        This method would be used when we have the Cohere API integrated for embedding generation.

        Args:
            user_query: The user query object containing scope and filters
            query_text: The text query to search for

        Returns:
            List of RetrievedChunk objects
        """
        # This is a placeholder - in a real implementation, we would:
        # 1. Generate an embedding for the query_text using Cohere
        # 2. Call retrieve_chunks with the generated embedding
        # For now, we'll raise an exception indicating this method needs to be implemented
        # with the embedding generation logic
        raise NotImplementedError(
            "retrieve_chunks_by_text requires embedding generation which needs to be "
            "implemented with Cohere API integration"
        )


# Global retrieval service instance
retrieval_service = RetrievalService()