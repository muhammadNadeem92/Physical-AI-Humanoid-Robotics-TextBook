"""
API package initialization for the RAG Chatbot API.
"""
from .chat import router as chat_router

__all__ = ["chat_router"]