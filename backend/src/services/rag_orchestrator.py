"""
RAG orchestrator for the RAG Chatbot API.

This module provides the main RAG orchestration logic that coordinates
retrieval, response generation, and grounding enforcement.
"""

from typing import List, Optional
from datetime import datetime
import time
import logging
from ..models.user_query import UserQuery
from ..models.chat_response import ChatResponse
from ..models.retrieved_chunk import RetrievedChunk
from ..services.retrieval_service import retrieval_service
from ..services.agent_service import agent_service
from ..services.history_service import history_service
from ..config.settings import settings


class RAGOrchestrator:
    """
    Main orchestrator for the RAG system that coordinates retrieval,
    response generation, and grounding enforcement.
    """

    def __init__(self):
        self.grounding_required = True  # Ensure responses are grounded in retrieved content
        self.selected_text_strict_mode = True  # In selected-text mode, block external retrieval

    async def process_query(self, user_query: UserQuery) -> ChatResponse:
        """
        Process a user query through the RAG pipeline.

        Args:
            user_query: The user query to process

        Returns:
            ChatResponse object with the generated answer and citations
        """
        start_time = time.time()

        try:
            # Get conversational context if available
            session_context = await history_service.get_recent_context(user_query.session_id)

            # Retrieve relevant chunks based on query scope
            retrieved_chunks = await self._retrieve_content(user_query)

            # Generate response using the agent
            response = await agent_service.generate_response(
                query=user_query.question,
                retrieved_chunks=retrieved_chunks,
                session_context=session_context
            )

            # Set the query and session IDs in the response
            response.query_id = user_query.query_id
            response.session_id = user_query.session_id

            # Calculate response time
            response_time_ms = int((time.time() - start_time) * 1000)
            response.response_time_ms = response_time_ms

            # Validate grounding if required
            if self.grounding_required:
                await self._validate_grounding(response, retrieved_chunks)

            logging.info(f"Processed query {user_query.query_id} in {response_time_ms}ms")
            return response

        except Exception as e:
            logging.error(f"Error processing query {user_query.query_id}: {str(e)}")
            # Return a default response in case of error
            return ChatResponse(
                response_id=f"response-{int(datetime.utcnow().timestamp())}",
                query_id=user_query.query_id,
                session_id=user_query.session_id,
                answer="I'm sorry, but I encountered an error while processing your request. Please try again.",
                citations=[],
                tokens_used=0,
                response_time_ms=int((time.time() - start_time) * 1000)
            )

    async def _retrieve_content(self, user_query: UserQuery) -> List[RetrievedChunk]:
        """
        Retrieve content based on the query scope and mode.

        Args:
            user_query: The user query containing scope information

        Returns:
            List of RetrievedChunk objects
        """
        if user_query.scope == "selected_text" and user_query.selected_text:
            # In selected-text mode, we don't retrieve from the database
            # Instead, we create a virtual chunk from the selected text
            if self.selected_text_strict_mode:
                # Return a single chunk with the selected text
                return [
                    RetrievedChunk(
                        chunk_id=f"virtual-{user_query.query_id}",
                        query_id=user_query.query_id,
                        content=user_query.selected_text,
                        page_url="",
                        module="selected_text",
                        chapter="selected_text",
                        section="selected_text",
                        score=1.0,  # Perfect match since it's the exact selected text
                        position=1,
                        timestamp=datetime.utcnow()
                    )
                ]
            else:
                # If strict mode is disabled, also perform normal retrieval
                # This would be unusual but included for completeness
                pass

        # For other scopes, perform normal retrieval from Qdrant
        # In a real implementation, we would generate embeddings for the query
        # and call the retrieval service. For this example, we'll simulate
        # an empty embedding list since we don't have the embedding generation
        # service implemented yet.
        try:
            # This would be replaced with actual embedding generation
            # For now, we'll return an empty list which will result in no context
            # In a real implementation, you would call a service to generate
            # embeddings from the user_query.question
            embedding = []  # Placeholder - would be generated from query text
            chunks = await retrieval_service.retrieve_chunks(user_query, embedding)
            return chunks
        except Exception as e:
            logging.warning(f"Could not retrieve content for query {user_query.query_id}: {str(e)}")
            return []

    async def _validate_grounding(self, response: ChatResponse, retrieved_chunks: List[RetrievedChunk]):
        """
        Validate that the response is properly grounded in the retrieved content.

        Args:
            response: The generated response
            retrieved_chunks: The chunks that were retrieved for context
        """
        # Basic grounding validation - ensure citations exist if chunks were retrieved
        if retrieved_chunks and not response.citations:
            logging.warning(f"Response for {response.query_id} has no citations despite having retrieved chunks")
            # In a production system, you might want to regenerate the response
            # or flag it for review

        # More sophisticated grounding validation would involve checking if
        # the response content actually references information from the retrieved chunks
        # This would require NLP techniques to compare semantic similarity

    async def process_selected_text_query(
        self,
        selected_text: str,
        question: str,
        session_id: str
    ) -> ChatResponse:
        """
        Process a query specifically about selected text.

        Args:
            selected_text: The text that was selected by the user
            question: The question about the selected text
            session_id: The session ID

        Returns:
            ChatResponse object with the generated answer
        """
        # Create a virtual query for selected text mode
        user_query = UserQuery(
            query_id=f"query-{int(datetime.utcnow().timestamp())}",
            session_id=session_id,
            question=question,
            scope="selected_text",
            selected_text=selected_text,
            timestamp=datetime.utcnow()
        )

        return await self.process_query(user_query)


# Global RAG orchestrator instance
rag_orchestrator = RAGOrchestrator()