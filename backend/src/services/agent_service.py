"""
Agent service for the RAG Chatbot API.

This module provides integration with the OpenAI Agent SDK and
Google Generative AI (Gemini) for response generation.
"""

from typing import List, Dict, Any, Optional
from ..models.retrieved_chunk import RetrievedChunk
from ..models.chat_response import ChatResponse, Citation
from ..config.llm import llm_config
from ..config.settings import settings
import google.generativeai as genai
from google.generativeai.types import GenerationConfig
import logging
from datetime import datetime


class AgentService:
    """
    Service class for handling AI response generation using Gemini via Agent SDK.
    """

    def __init__(self):
        self.model = llm_config.get_model()
        self.gemini_model_name = settings.gemini_model
        self.max_tokens = settings.max_tokens

    async def generate_response(
        self,
        query: str,
        retrieved_chunks: List[RetrievedChunk],
        session_context: Optional[str] = None
    ) -> ChatResponse:
        """
        Generate a response using the Gemini model with retrieved context.

        Args:
            query: The user's question
            retrieved_chunks: List of relevant content chunks
            session_context: Optional conversational context from previous exchanges

        Returns:
            ChatResponse object with the generated answer and citations
        """
        try:
            # Build the prompt with retrieved context
            context = self._build_context(retrieved_chunks)
            prompt = self._build_prompt(query, context, session_context)

            # Configure generation parameters
            generation_config = GenerationConfig(
                max_output_tokens=self.max_tokens,
                temperature=0.7,
                top_p=0.9,
                top_k=40
            )

            # Generate content using Gemini
            response = await self.model.generate_content_async(
                prompt,
                generation_config=generation_config,
                # Safety settings to ensure appropriate responses
                safety_settings=[
                    {"category": "HARM_CATEGORY_HARASSMENT", "threshold": "BLOCK_MEDIUM_AND_ABOVE"},
                    {"category": "HARM_CATEGORY_HATE_SPEECH", "threshold": "BLOCK_MEDIUM_AND_ABOVE"},
                    {"category": "HARM_CATEGORY_SEXUALLY_EXPLICIT", "threshold": "BLOCK_MEDIUM_AND_ABOVE"},
                    {"category": "HARM_CATEGORY_DANGEROUS_CONTENT", "threshold": "BLOCK_MEDIUM_AND_ABOVE"},
                ]
            )

            # Extract the text response
            answer = response.text if response.text else "I couldn't find relevant information to answer your question."

            # Create citations from the retrieved chunks
            citations = self._create_citations(retrieved_chunks)

            # Create and return the ChatResponse object
            chat_response = ChatResponse(
                response_id=f"response-{int(datetime.utcnow().timestamp())}",
                query_id="",  # This will be set by the caller
                session_id="",  # This will be set by the caller
                answer=answer,
                citations=citations,
                tokens_used=self._estimate_tokens(prompt + answer),
                response_time_ms=None  # This will be calculated by the caller
            )

            logging.info(f"Generated response with {len(citations)} citations")
            return chat_response

        except Exception as e:
            logging.error(f"Error generating response: {str(e)}")
            # Return a default response in case of error
            return ChatResponse(
                response_id=f"response-{int(datetime.utcnow().timestamp())}",
                query_id="",  # This will be set by the caller
                session_id="",  # This will be set by the caller
                answer="I'm sorry, but I encountered an error while processing your request. Please try again.",
                citations=[],
                tokens_used=0,
                response_time_ms=None  # This will be calculated by the caller
            )

    def _build_context(self, retrieved_chunks: List[RetrievedChunk]) -> str:
        """
        Build context from retrieved chunks for the prompt.

        Args:
            retrieved_chunks: List of retrieved content chunks

        Returns:
            Formatted context string
        """
        if not retrieved_chunks:
            return "No relevant context found in the textbook."

        context_parts = ["Here is the relevant context from the textbook:"]
        for i, chunk in enumerate(retrieved_chunks, 1):
            context_parts.append(
                f"Source {i} (Module: {chunk.module}, Chapter: {chunk.chapter}, Section: {chunk.section}):\n"
                f"{chunk.content}\n"
            )

        return "\n".join(context_parts)

    def _build_prompt(
        self,
        query: str,
        context: str,
        session_context: Optional[str] = None
    ) -> str:
        """
        Build the complete prompt for the LLM.

        Args:
            query: The user's question
            context: The retrieved context
            session_context: Optional conversational context

        Returns:
            Complete prompt string
        """
        prompt_parts = []

        # System instruction
        prompt_parts.append(
            "You are an AI assistant helping students with a Physical AI & Humanoid Robotics textbook. "
            "Answer the user's question based only on the provided context. "
            "If the context doesn't contain enough information to answer the question, say so clearly. "
            "Always provide source citations for the information you use in your response."
        )

        # Add session context if available
        if session_context:
            prompt_parts.append(f"\nPrevious conversation context:\n{session_context}\n")

        # Add the context
        prompt_parts.append(f"\n{context}\n")

        # Add the user query
        prompt_parts.append(f"\nQuestion: {query}\n")

        # Add instruction about citations
        prompt_parts.append(
            "\nProvide your answer with clear source citations. "
            "If you use information from the provided context, specify which source it came from."
        )

        return "\n".join(prompt_parts)

    def _create_citations(self, retrieved_chunks: List[RetrievedChunk]) -> List[Citation]:
        """
        Create citation objects from retrieved chunks.

        Args:
            retrieved_chunks: List of retrieved content chunks

        Returns:
            List of Citation objects
        """
        citations = []
        for chunk in retrieved_chunks:
            citation = Citation(
                text=chunk.content,
                page_url=chunk.page_url,
                module=chunk.module,
                chapter=chunk.chapter,
                section=chunk.section
            )
            citations.append(citation)

        return citations

    def _estimate_tokens(self, text: str) -> int:
        """
        Estimate the number of tokens in the text.
        This is a rough estimation - 1 token is approximately 4 characters in English.

        Args:
            text: Text to estimate tokens for

        Returns:
            Estimated number of tokens
        """
        return max(1, len(text) // 4)


# Global agent service instance
agent_service = AgentService()