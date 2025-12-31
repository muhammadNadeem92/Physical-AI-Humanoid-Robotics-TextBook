"""
Main application file for the RAG Chatbot API.

This module initializes the FastAPI application and sets up all routes.
"""

import asyncio
from contextlib import asynccontextmanager
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from .api.chat import router as chat_router
from .config.settings import settings
from .services.session_service import session_service
import logging
import sys
import os


# Set up logging
logging.basicConfig(
    level=logging.INFO if not settings.debug else logging.DEBUG,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    handlers=[
        logging.StreamHandler(sys.stdout)
    ]
)
logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Lifespan event handler for the FastAPI application.
    This runs startup and shutdown events.
    """
    logger.info(f"Starting {settings.app_name} v{settings.app_version}")

    # Startup logic
    # In a real application, you might initialize database connections, caches, etc.
    # For this implementation, we'll just log startup
    logger.info("Application started successfully")

    yield  # This is where the application runs

    # Shutdown logic
    # Clean up resources
    await session_service.cleanup_expired_sessions()
    logger.info("Application shutdown completed")


# Create FastAPI app with lifespan
app = FastAPI(
    title=settings.app_name,
    description="RAG Chatbot API for Physical AI & Humanoid Robotics Textbook",
    version=settings.app_version,
    lifespan=lifespan,
    debug=settings.debug
)


# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.allowed_origins.split(",") if settings.allowed_origins != "*" else ["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    # Expose headers for client-side access
    expose_headers=["Access-Control-Allow-Origin"]
)


# Add routes
app.include_router(chat_router)


# Add a root endpoint
@app.get("/")
async def root():
    """
    Root endpoint for the API.
    """
    return {
        "app_name": settings.app_name,
        "version": settings.app_version,
        "description": "RAG Chatbot API for Physical AI & Humanoid Robotics Textbook",
        "docs_url": "/docs",
        "redoc_url": "/redoc"
    }


# Add a global exception handler
@app.exception_handler(Exception)
async def global_exception_handler(request, exc):
    """
    Global exception handler for the API.
    """
    logger.error(f"Unhandled exception: {str(exc)}", exc_info=True)
    return JSONResponse(
        status_code=500,
        content={"detail": "An internal server error occurred"}
    )


# Add a validation exception handler
@app.exception_handler(ValueError)
async def value_error_handler(request, exc):
    """
    Handler for value errors (like validation errors).
    """
    logger.warning(f"Value error: {str(exc)}")
    return JSONResponse(
        status_code=400,
        content={"detail": str(exc)}
    )


if __name__ == "__main__":
    import uvicorn

    # Run the application with uvicorn
    # When running this file directly, the module reference needs to be from the project root
    # So we use the relative path from the project root directory
    uvicorn.run(
        "backend.src.main:app",
        host="0.0.0.0",
        port=8000,
        reload=settings.debug,
        log_level="info" if not settings.debug else "debug"
    )