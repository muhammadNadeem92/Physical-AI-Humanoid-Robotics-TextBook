#!/usr/bin/env python3
"""
Hugging Face Spaces entry point for the RAG Chatbot API backend server.

This file serves as the entry point for Hugging Face Spaces deployment.
"""

import uvicorn
import sys
import os

# Add the project root to the path so imports work correctly
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

# Set environment variables for Hugging Face Spaces
os.environ.setdefault("DEBUG", "False")
os.environ.setdefault("ALLOWED_ORIGINS", "*")

from backend.src.config.settings import settings

def main():
    """Start the backend server for Hugging Face Spaces."""
    print(f"Starting {settings.app_name} v{settings.app_version}")
    print(f"Server will run on http://0.0.0.0:7860")
    print("Press CTRL+C to stop the server")

    # Hugging Face Spaces typically uses port 7860
    uvicorn.run(
        "backend.src.main:app",
        host="0.0.0.0",
        port=7860,  # Hugging Face Spaces port
        reload=False,  # Disable reload for production
        log_level="info"
    )

if __name__ == "__main__":
    main()