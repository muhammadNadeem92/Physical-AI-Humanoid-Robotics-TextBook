"""
LLM configuration for the RAG Chatbot API.

This module handles the Gemini API configuration
and provides centralized access to LLM configuration values.
"""

from typing import Optional
from google.generativeai import configure, GenerativeModel
from .settings import settings


class LLMConfig:
    """
    LLM configuration and model management.
    """

    def __init__(self):
        self.gemini_api_key = settings.gemini_api_key
        self.gemini_model_name = settings.gemini_model
        self.model: Optional[GenerativeModel] = None

    def init_model(self):
        """
        Initialize the Gemini model.
        """
        if not self.gemini_api_key:
            raise ValueError("GEMINI_API_KEY environment variable is not set")

        # Configure the Google Generative AI client
        configure(api_key=self.gemini_api_key)

        # Initialize the model
        self.model = GenerativeModel(self.gemini_model_name)

    def get_model(self) -> GenerativeModel:
        """
        Get the Gemini model instance.
        """
        if not self.model:
            self.init_model()

        return self.model


# Global LLM config instance
llm_config = LLMConfig()