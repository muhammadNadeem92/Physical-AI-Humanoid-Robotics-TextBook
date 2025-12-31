#!/usr/bin/env python3
"""
Entry point for Hugging Face Spaces.
"""

import os
import sys
from pathlib import Path

# Add the project root to the path
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root))

# Set environment variables for Hugging Face Spaces
os.environ.setdefault("DEBUG", "False")
os.environ.setdefault("ALLOWED_ORIGINS", "*")

if __name__ == "__main__":
    import uvicorn

    # Import the app after setting up the path
    from backend.src.main import app

    # Run with Hugging Face Spaces configuration
    uvicorn.run(
        app,
        host="0.0.0.0",
        port=7860,
        reload=False
    )