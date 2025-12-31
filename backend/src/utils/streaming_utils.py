"""
Streaming Response Utilities

Utilities for creating Server-Sent Events (SSE) responses for chat streaming.
"""
import asyncio
import json
import logging
from typing import AsyncGenerator, Dict, Any, Union
from datetime import datetime

from fastapi import Response
from sse_starlette.sse import ServerSentEvent


logger = logging.getLogger(__name__)


class StreamingResponseUtils:
    """
    Utilities for creating and managing Server-Sent Events (SSE) responses.
    """

    @staticmethod
    async def create_sse_response(
        data_generator: AsyncGenerator[Union[str, Dict[str, Any]], None],
        event_type: str = "message",
        retry_interval: int = 3000  # milliseconds
    ) -> Response:
        """
        Create an SSE response from a data generator.

        Args:
            data_generator: Async generator that yields data chunks
            event_type: Type of SSE event
            retry_interval: Reconnection time for clients in milliseconds

        Returns:
            FastAPI Response object with SSE headers
        """
        async def event_generator():
            try:
                async for chunk in data_generator:
                    if isinstance(chunk, dict):
                        # If chunk is a dict, convert to JSON string
                        data = json.dumps(chunk)
                    else:
                        # If chunk is a string, use as-is
                        data = chunk

                    event = ServerSentEvent(
                        data=data,
                        event=event_type,
                        retry=retry_interval
                    )
                    yield event
            except Exception as e:
                logger.error(f"Error in SSE event generator: {e}")
                # Send error event
                error_event = ServerSentEvent(
                    data=json.dumps({
                        "type": "error",
                        "message": "An error occurred while processing your request",
                        "timestamp": datetime.utcnow().isoformat()
                    }),
                    event="error",
                    retry=retry_interval
                )
                yield error_event

        return Response(
            content=event_generator(),
            media_type="text/event-stream",
            headers={
                "Cache-Control": "no-cache",
                "Connection": "keep-alive",
                "Access-Control-Allow-Origin": "*",
                "Access-Control-Allow-Headers": "Cache-Control",
            }
        )

    @staticmethod
    async def format_chat_chunk(
        content: str,
        chunk_type: str = "text",
        metadata: Dict[str, Any] = None
    ) -> Dict[str, Any]:
        """
        Format a chat response chunk for streaming.

        Args:
            content: The content of the chunk
            chunk_type: Type of chunk ("text", "citation", "source", "end")
            metadata: Additional metadata to include

        Returns:
            Formatted chunk dictionary
        """
        chunk = {
            "type": chunk_type,
            "content": content,
            "timestamp": datetime.utcnow().isoformat()
        }

        if metadata:
            chunk["metadata"] = metadata

        return chunk

    @staticmethod
    async def create_chat_response_stream(
        response_generator: AsyncGenerator[str, None],
        citations: list = None,
        query_id: str = None,
        session_id: str = None
    ) -> AsyncGenerator[Dict[str, Any], None]:
        """
        Create a chat response stream with proper formatting.

        Args:
            response_generator: Generator that yields response chunks
            citations: List of citations to include
            query_id: Query identifier
            session_id: Session identifier

        Yields:
            Formatted chat response chunks
        """
        try:
            # Send start event
            yield await StreamingResponseUtils.format_chat_chunk(
                content="",
                chunk_type="start",
                metadata={
                    "query_id": query_id,
                    "session_id": session_id
                }
            )

            # Stream response text
            async for chunk in response_generator:
                if chunk.strip():  # Only send non-empty chunks
                    yield await StreamingResponseUtils.format_chat_chunk(
                        content=chunk,
                        chunk_type="text"
                    )

            # Send citations if available
            if citations:
                yield await StreamingResponseUtils.format_chat_chunk(
                    content=citations,
                    chunk_type="citations"
                )

            # Send end event
            yield await StreamingResponseUtils.format_chat_chunk(
                content="",
                chunk_type="end",
                metadata={
                    "query_id": query_id,
                    "session_id": session_id
                }
            )
        except Exception as e:
            logger.error(f"Error in chat response stream: {e}")
            yield await StreamingResponseUtils.format_chat_chunk(
                content="An error occurred while streaming the response",
                chunk_type="error"
            )

    @staticmethod
    async def create_heartbeat_stream(
        interval: int = 15
    ) -> AsyncGenerator[Dict[str, Any], None]:
        """
        Create a heartbeat stream to keep the connection alive.

        Args:
            interval: Interval between heartbeats in seconds

        Yields:
            Heartbeat events
        """
        while True:
            try:
                yield await StreamingResponseUtils.format_chat_chunk(
                    content="",
                    chunk_type="heartbeat"
                )
                await asyncio.sleep(interval)
            except asyncio.CancelledError:
                logger.info("Heartbeat stream cancelled")
                break
            except Exception as e:
                logger.error(f"Error in heartbeat stream: {e}")
                break

    @staticmethod
    async def format_error_chunk(
        error_message: str,
        error_code: str = "INTERNAL_ERROR"
    ) -> Dict[str, Any]:
        """
        Format an error chunk for streaming.

        Args:
            error_message: The error message
            error_code: Error code

        Returns:
            Formatted error chunk dictionary
        """
        return {
            "type": "error",
            "error_code": error_code,
            "message": error_message,
            "timestamp": datetime.utcnow().isoformat()
        }

    @staticmethod
    async def create_progress_stream(
        total_steps: int,
        current_step: int = 0,
        message: str = "Processing..."
    ) -> AsyncGenerator[Dict[str, Any], None]:
        """
        Create a progress stream to show processing progress.

        Args:
            total_steps: Total number of steps
            current_step: Current step number
            message: Progress message

        Yields:
            Progress update chunks
        """
        progress_data = {
            "current": current_step,
            "total": total_steps,
            "percentage": (current_step / total_steps) * 100 if total_steps > 0 else 0,
            "message": message
        }

        yield await StreamingResponseUtils.format_chat_chunk(
            content=progress_data,
            chunk_type="progress"
        )

    @staticmethod
    def create_sse_data(content: Union[str, Dict[str, Any]]) -> str:
        """
        Create properly formatted SSE data string.

        Args:
            content: Content to format as SSE data

        Returns:
            Formatted SSE data string
        """
        if isinstance(content, dict):
            data = json.dumps(content)
        else:
            data = content

        return f"data: {data}\n\n"