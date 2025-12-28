"""
Content Retriever Service

Handles retrieval of relevant content from Qdrant vector database
based on user queries and various scoping options.
"""
import asyncio
import logging
from typing import List, Optional
from datetime import datetime

import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models

from src.models.retrieved_chunk import RetrievedChunk
from src.config.settings import settings


logger = logging.getLogger(__name__)


class ContentRetriever:
    """
    Retrieves relevant content from Qdrant based on user queries.
    """

    def __init__(self, qdrant_client: QdrantClient, cohere_client: cohere.Client):
        self.qdrant_client = qdrant_client
        self.cohere_client = cohere_client

    async def retrieve_global(self, query: str, limit: int = 10) -> List[RetrievedChunk]:
        """
        Retrieve content from the entire knowledge base.

        Args:
            query: The user's query
            limit: Maximum number of chunks to retrieve

        Returns:
            List of RetrievedChunk objects
        """
        try:
            # Generate embedding for the query
            response = self.cohere_client.embed(
                texts=[query],
                model=settings.cohere.embed_model
            )
            query_embedding = response.embeddings[0]

            # Search in Qdrant
            search_results = self.qdrant_client.search(
                collection_name=settings.qdrant.collection_name,
                query_vector=query_embedding,
                limit=limit,
                with_payload=True,
                with_vectors=False
            )

            # Convert search results to RetrievedChunk objects
            retrieved_chunks = []
            for result in search_results:
                chunk = RetrievedChunk(
                    content=result.payload.get("content", ""),
                    metadata=result.payload,
                    embedding=None,  # Embedding not needed after retrieval
                    score=result.score
                )
                retrieved_chunks.append(chunk)

            logger.info(f"Retrieved {len(retrieved_chunks)} chunks for global query")
            return retrieved_chunks

        except Exception as e:
            logger.error(f"Error retrieving global content: {e}")
            return []

    async def retrieve_scoped(
        self,
        query: str,
        module: Optional[str] = None,
        chapter: Optional[str] = None,
        limit: int = 10
    ) -> List[RetrievedChunk]:
        """
        Retrieve content limited to specific module or chapter.

        Args:
            query: The user's query
            module: Optional module name to scope the query
            chapter: Optional chapter name to scope the query
            limit: Maximum number of chunks to retrieve

        Returns:
            List of RetrievedChunk objects
        """
        try:
            # Generate embedding for the query
            response = self.cohere_client.embed(
                texts=[query],
                model=settings.cohere.embed_model
            )
            query_embedding = response.embeddings[0]

            # Build filter conditions
            must_conditions = []

            if module:
                must_conditions.append(
                    models.FieldCondition(
                        key="module",
                        match=models.MatchValue(value=module)
                    )
                )

            if chapter:
                must_conditions.append(
                    models.FieldCondition(
                        key="chapter",
                        match=models.MatchValue(value=chapter)
                    )
                )

            # Create filter if there are conditions
            query_filter = None
            if must_conditions:
                query_filter = models.Filter(must=must_conditions)

            # Search in Qdrant with filter
            search_results = self.qdrant_client.search(
                collection_name=settings.qdrant.collection_name,
                query_vector=query_embedding,
                query_filter=query_filter,
                limit=limit,
                with_payload=True,
                with_vectors=False
            )

            # Convert search results to RetrievedChunk objects
            retrieved_chunks = []
            for result in search_results:
                chunk = RetrievedChunk(
                    content=result.payload.get("content", ""),
                    metadata=result.payload,
                    embedding=None,  # Embedding not needed after retrieval
                    score=result.score
                )
                retrieved_chunks.append(chunk)

            logger.info(f"Retrieved {len(retrieved_chunks)} chunks for scoped query (module={module}, chapter={chapter})")
            return retrieved_chunks

        except Exception as e:
            logger.error(f"Error retrieving scoped content: {e}")
            return []

    async def retrieve_by_ids(self, chunk_ids: List[str]) -> List[RetrievedChunk]:
        """
        Retrieve specific content chunks by their IDs.

        Args:
            chunk_ids: List of chunk IDs to retrieve

        Returns:
            List of RetrievedChunk objects
        """
        try:
            # Get points by IDs from Qdrant
            points = self.qdrant_client.retrieve(
                collection_name=settings.qdrant.collection_name,
                ids=chunk_ids,
                with_payload=True,
                with_vectors=False
            )

            # Convert points to RetrievedChunk objects
            retrieved_chunks = []
            for point in points:
                chunk = RetrievedChunk(
                    content=point.payload.get("content", ""),
                    metadata=point.payload,
                    embedding=None,
                    score=1.0  # Score not applicable when retrieving by ID
                )
                retrieved_chunks.append(chunk)

            logger.info(f"Retrieved {len(retrieved_chunks)} chunks by IDs")
            return retrieved_chunks

        except Exception as e:
            logger.error(f"Error retrieving content by IDs: {e}")
            return []

    async def retrieve_similar(self, content: str, limit: int = 10) -> List[RetrievedChunk]:
        """
        Retrieve content similar to the provided content.

        Args:
            content: Content to find similar chunks for
            limit: Maximum number of chunks to retrieve

        Returns:
            List of RetrievedChunk objects
        """
        try:
            # Generate embedding for the content
            response = self.cohere_client.embed(
                texts=[content],
                model=settings.cohere.embed_model
            )
            content_embedding = response.embeddings[0]

            # Search in Qdrant
            search_results = self.qdrant_client.search(
                collection_name=settings.qdrant.collection_name,
                query_vector=content_embedding,
                limit=limit,
                with_payload=True,
                with_vectors=False
            )

            # Convert search results to RetrievedChunk objects
            retrieved_chunks = []
            for result in search_results:
                chunk = RetrievedChunk(
                    content=result.payload.get("content", ""),
                    metadata=result.payload,
                    embedding=None,  # Embedding not needed after retrieval
                    score=result.score
                )
                retrieved_chunks.append(chunk)

            logger.info(f"Retrieved {len(retrieved_chunks)} similar chunks")
            return retrieved_chunks

        except Exception as e:
            logger.error(f"Error retrieving similar content: {e}")
            return []