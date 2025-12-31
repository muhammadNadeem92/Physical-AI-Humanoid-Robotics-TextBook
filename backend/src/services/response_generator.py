"""
Response Generator Service

Generates responses using Google Gemini based on user queries and context.
"""
import asyncio
import logging
from typing import List, Dict, Any, AsyncGenerator
from datetime import datetime

from google.generativeai import GenerativeModel
import google.generativeai as genai

from src.models.retrieved_chunk import RetrievedChunk
from src.config.settings import settings


logger = logging.getLogger(__name__)


class ResponseGenerator:
    """
    Generates responses using Google Gemini based on context and user queries.
    """

    def __init__(self, gemini_model: GenerativeModel):
        self.gemini_model = gemini_model

    async def generate_response(
        self,
        question: str,
        context_chunks: List[RetrievedChunk],
        history: List[Dict[str, Any]] = None
    ) -> str:
        """
        Generate a response to the user's question based on context and history.

        Args:
            question: The user's question
            context_chunks: List of relevant content chunks
            history: Conversation history

        Returns:
            Generated response text
        """
        try:
            # Prepare the prompt with context
            prompt = self._build_prompt(question, context_chunks, history)

            # Generate content using Gemini
            response = await self.gemini_model.generate_content_async(
                prompt,
                generation_config={
                    "temperature": settings.gemini.temperature,
                    "max_output_tokens": settings.gemini.max_tokens,
                    "top_p": settings.gemini.top_p,
                    "top_k": settings.gemini.top_k,
                }
            )

            if response.text:
                logger.info(f"Generated response of {len(response.text)} characters")
                return response.text.strip()
            else:
                logger.warning("Gemini returned empty response")
                return "I couldn't generate a response to your question. Please try rephrasing."

        except Exception as e:
            logger.error(f"Error generating response: {e}")
            return "I encountered an error while processing your request. Please try again."

    async def generate_streaming_response(
        self,
        question: str,
        context_chunks: List[RetrievedChunk],
        history: List[Dict[str, Any]] = None
    ) -> AsyncGenerator[str, None]:
        """
        Generate a streaming response to the user's question based on context and history.

        Args:
            question: The user's question
            context_chunks: List of relevant content chunks
            history: Conversation history

        Yields:
            Response chunks as they are generated
        """
        try:
            # Prepare the prompt with context
            prompt = self._build_prompt(question, context_chunks, history)

            # Generate content using Gemini with streaming
            response = await self.gemini_model.generate_content_async(
                prompt,
                generation_config={
                    "temperature": settings.gemini.temperature,
                    "max_output_tokens": settings.gemini.max_tokens,
                    "top_p": settings.gemini.top_p,
                    "top_k": settings.gemini.top_k,
                },
                stream=True
            )

            # Stream the response
            async for chunk in response:
                if chunk.text:
                    yield chunk.text

        except Exception as e:
            logger.error(f"Error generating streaming response: {e}")
            yield "I encountered an error while processing your request. Please try again."

    def _build_prompt(
        self,
        question: str,
        context_chunks: List[RetrievedChunk],
        history: List[Dict[str, Any]] = None
    ) -> str:
        """
        Build the prompt for the LLM with context and history.

        Args:
            question: The user's question
            context_chunks: List of relevant content chunks
            history: Conversation history

        Returns:
            Formatted prompt string
        """
        # Start with system instructions
        prompt_parts = [
            "You are an AI assistant helping users with questions about Physical AI & Humanoid Robotics.",
            "Use the following context to answer the user's question.",
            "Be concise, accurate, and cite sources when possible.",
            "If the context doesn't contain enough information, say so clearly.",
            "\n---\n"
        ]

        # Add conversation history if available
        if history:
            prompt_parts.append("Previous conversation:")
            for interaction in history[-settings.rag.history_window:]:
                prompt_parts.append(f"Q: {interaction.get('question', '')}")
                prompt_parts.append(f"A: {interaction.get('answer', '')}")
            prompt_parts.append("\n---\n")

        # Add context chunks
        if context_chunks:
            prompt_parts.append("Context:")
            for i, chunk in enumerate(context_chunks):
                prompt_parts.append(f"[{i+1}] {chunk.content}")
                # Add source information if available
                source = chunk.metadata.get("source_url") or chunk.metadata.get("source")
                if source:
                    prompt_parts.append(f"Source: {source}")
            prompt_parts.append("\n---\n")

        # Add the user's question
        prompt_parts.append(f"Question: {question}")
        prompt_parts.append("Answer (with citations if applicable):")

        return "\n".join(prompt_parts)

    async def validate_response(
        self,
        response: str,
        context_chunks: List[RetrievedChunk]
    ) -> bool:
        """
        Validate that the response is grounded in the provided context.

        Args:
            response: The generated response
            context_chunks: List of context chunks used to generate the response

        Returns:
            True if response is grounded in context, False otherwise
        """
        try:
            # Simple validation: check if response contains content from context
            context_text = " ".join([chunk.content for chunk in context_chunks])
            response_lower = response.lower()
            context_lower = context_text.lower()

            # Check if response contains key terms from context
            context_words = set(context_lower.split()[:50])  # First 50 words as sample
            response_words = set(response_lower.split())

            # If there's significant overlap in content words, consider it grounded
            overlap = len(context_words.intersection(response_words))
            total_context_words = len(context_words)

            # Consider it grounded if at least 20% of context words appear in response
            # (This is a simple heuristic - more sophisticated validation could be implemented)
            is_likely_justified = (overlap / total_context_words) > 0.2 if total_context_words > 0 else False

            # Additional check: if response contains phrases like "not in context" or "no information",
            # it's likely appropriately indicating lack of information
            if any(phrase in response_lower for phrase in [
                "not in context", "no information", "not mentioned", "not found", "not provided"
            ]):
                return True

            return is_likely_justified

        except Exception as e:
            logger.error(f"Error validating response: {e}")
            return True  # Fail open - don't block responses due to validation errors