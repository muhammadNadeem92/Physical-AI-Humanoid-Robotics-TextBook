"""
Chat API endpoints for the RAG Chatbot API.

This module defines the FastAPI endpoints for the chat functionality,
including query, selected text, history, and feedback endpoints.
"""

from fastapi import APIRouter, HTTPException, Depends, Request
from typing import List, Optional
from sse_starlette.sse import EventSourceResponse
import asyncio
import json
from ..models.user_query import UserQuery
from ..models.chat_response import ChatResponse, Citation
from ..services.query_processor import query_processor
from ..services.rag_orchestrator import rag_orchestrator
from ..services.session_service import session_service
from ..services.history_service import history_service
from ..services.feedback_service import feedback_service
from ..config.settings import settings


# Create API router
router = APIRouter(prefix="/chat", tags=["chat"])


@router.post("/query", response_model=ChatResponse)
async def chat_query(
    question: str,
    session_id: str,
    module: Optional[str] = None,
    chapter: Optional[str] = None
):
    """
    Process a chat query against the entire book or specific scope.

    Args:
        question: The question to ask
        session_id: Session identifier for maintaining conversational context
        module: Optional module filter for scoped queries
        chapter: Optional chapter filter for scoped queries

    Returns:
        ChatResponse with the answer and citations
    """
    try:
        # Process the query to create a UserQuery object
        user_query = await query_processor.process_query(
            question=question,
            session_id=session_id,
            module=module,
            chapter=chapter
        )

        # Validate the query
        validation_errors = await query_processor.validate_query(user_query)
        if validation_errors:
            raise HTTPException(status_code=400, detail=validation_errors)

        # Add the query to history
        await history_service.add_user_query(user_query)

        # Process the query through the RAG orchestrator
        response = await rag_orchestrator.process_query(user_query)

        # Add the response to history
        await history_service.add_chat_response(response)

        return response

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")


@router.post("/selected-text", response_model=ChatResponse)
async def chat_selected_text(
    selected_text: str,
    question: str,
    session_id: str
):
    """
    Process a query specifically about selected text.

    Args:
        selected_text: The text selected by the user
        question: The question about the selected text
        session_id: Session identifier for maintaining conversational context

    Returns:
        ChatResponse with the answer (strictly based on selected text)
    """
    try:
        if not selected_text or not selected_text.strip():
            raise HTTPException(status_code=400, detail="Selected text cannot be empty")

        if not question or not question.strip():
            raise HTTPException(status_code=400, detail="Question cannot be empty")

        # Process the selected text query through the RAG orchestrator
        response = await rag_orchestrator.process_selected_text_query(
            selected_text=selected_text,
            question=question,
            session_id=session_id
        )

        # Create a virtual UserQuery for history tracking
        user_query = await query_processor.process_query(
            question=question,
            session_id=session_id,
            selected_text=selected_text
        )

        # Add the query and response to history
        await history_service.add_user_query(user_query)
        await history_service.add_chat_response(response)

        return response

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing selected text query: {str(e)}")


@router.get("/history/{session_id}")
async def get_chat_history(session_id: str):
    """
    Retrieve conversation history for a session.

    Args:
        session_id: Session identifier to retrieve history for

    Returns:
        List of conversation history items
    """
    try:
        history = await history_service.get_history(session_id)
        if not history:
            raise HTTPException(status_code=404, detail="Session not found or no history available")

        return {
            "session_id": session_id,
            "history": history
        }

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving history: {str(e)}")


@router.post("/feedback")
async def submit_feedback(
    session_id: str,
    query_id: str,
    response_id: str,
    feedback_type: str,
    feedback_text: Optional[str] = None
):
    """
    Submit feedback on a chat response.

    Args:
        session_id: Session identifier
        query_id: Query identifier that the feedback refers to
        response_id: Response identifier that the feedback refers to
        feedback_type: Type of feedback ('thumbs_up', 'thumbs_down', 'report_inaccurate')
        feedback_text: Additional feedback text (optional)

    Returns:
        Success status and feedback ID
    """
    try:
        from ..models.feedback import Feedback, FeedbackType

        # Validate feedback type
        try:
            feedback_type_enum = FeedbackType(feedback_type)
        except ValueError:
            raise HTTPException(status_code=400, detail=f"Invalid feedback type: {feedback_type}")

        # Create feedback object
        feedback = Feedback(
            feedback_id=f"feedback-{int(__import__('time').time())}",
            session_id=session_id,
            query_id=query_id,
            response_id=response_id,
            feedback_type=feedback_type_enum,
            feedback_text=feedback_text
        )

        # Submit feedback
        result = await feedback_service.submit_feedback(feedback)

        if not result["success"]:
            raise HTTPException(status_code=400, detail=result.get("error", "Failed to submit feedback"))

        return result

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error submitting feedback: {str(e)}")


@router.post("/query/stream")
async def chat_query_stream(
    request: Request,
    question: str,
    session_id: str,
    module: Optional[str] = None,
    chapter: Optional[str] = None
):
    """
    Process a chat query with streaming response using Server-Sent Events (SSE).
    This endpoint is compatible with ChatKit for real-time responses.

    Args:
        request: FastAPI request object
        question: The question to ask
        session_id: Session identifier for maintaining conversational context
        module: Optional module filter for scoped queries
        chapter: Optional chapter filter for scoped queries

    Returns:
        Streaming response with Server-Sent Events
    """
    async def event_generator():
        try:
            # Process the query to create a UserQuery object
            user_query = await query_processor.process_query(
                question=question,
                session_id=session_id,
                module=module,
                chapter=chapter
            )

            # Validate the query
            validation_errors = await query_processor.validate_query(user_query)
            if validation_errors:
                yield {
                    "event": "error",
                    "data": json.dumps({"error": validation_errors})
                }
                return

            # Add the query to history
            await history_service.add_user_query(user_query)

            # For streaming, we'll simulate a response by breaking it into chunks
            # In a real implementation, this would connect to the LLM's streaming API
            response = await rag_orchestrator.process_query(user_query)

            # Send start event
            yield {
                "event": "start",
                "data": json.dumps({
                    "response_id": response.response_id,
                    "type": "start"
                })
            }

            # Simulate streaming the response content
            content = response.answer
            chunk_size = 20  # characters per chunk

            for i in range(0, len(content), chunk_size):
                chunk = content[i:i + chunk_size]
                yield {
                    "event": "token",
                    "data": json.dumps({
                        "content": chunk,
                        "type": "token"
                    })
                }
                await asyncio.sleep(0.05)  # Simulate delay between tokens

            # Send citations if available
            if response.citations:
                yield {
                    "event": "citations",
                    "data": json.dumps({
                        "citations": [citation.dict() for citation in response.citations],
                        "type": "citations"
                    })
                }

            # Send end event
            yield {
                "event": "end",
                "data": json.dumps({
                    "response_id": response.response_id,
                    "type": "end",
                    "response_time_ms": response.response_time_ms
                })
            }

            # Add the response to history
            await history_service.add_chat_response(response)

        except Exception as e:
            yield {
                "event": "error",
                "data": json.dumps({"error": str(e)})
            }

    return EventSourceResponse(event_generator())


# Health check endpoint
@router.get("/health")
async def health_check():
    """
    Health check endpoint to verify the API is running.
    """
    return {"status": "healthy", "service": "RAG Chatbot API", "version": settings.app_version}