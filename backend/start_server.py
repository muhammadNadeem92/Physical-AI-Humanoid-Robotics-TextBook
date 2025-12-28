#!/usr/bin/env python3
"""
Start script for the RAG Chatbot API backend server.

Usage:
    python start_server.py
"""

import uvicorn
import sys
import os

# Add the project root to the path so imports work correctly
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from backend.src.config.settings import settings

def main():
    """Start the backend server."""
    print(f"Starting {settings.app_name} v{settings.app_version}")
    print(f"Server will run on http://0.0.0.0:8000")
    print("Press CTRL+C to stop the server")

    uvicorn.run(
        "backend.src.main:app",
        host="0.0.0.0",
        port=8000,
        reload=settings.debug,
        log_level="info" if not settings.debug else "debug"
    )

if __name__ == "__main__":
    main()