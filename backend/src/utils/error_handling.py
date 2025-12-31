"""
Error Handling Utilities

Utilities for handling errors and exceptions in the RAG system.
"""
import logging
import traceback
from typing import Dict, Any, Optional
from datetime import datetime
from functools import wraps

from fastapi import HTTPException, Request
from fastapi.responses import JSONResponse


logger = logging.getLogger(__name__)


class RAGError(Exception):
    """
    Base exception class for RAG system errors.
    """
    def __init__(self, message: str, error_code: str = "RAG_ERROR", details: Optional[Dict[str, Any]] = None):
        self.message = message
        self.error_code = error_code
        self.details = details or {}
        self.timestamp = datetime.utcnow()
        super().__init__(self.message)

    def to_dict(self) -> Dict[str, Any]:
        """Convert error to dictionary format."""
        return {
            "error_code": self.error_code,
            "message": self.message,
            "details": self.details,
            "timestamp": self.timestamp.isoformat()
        }


class ContentRetrievalError(RAGError):
    """Exception raised when content retrieval fails."""
    def __init__(self, message: str = "Failed to retrieve content", details: Optional[Dict[str, Any]] = None):
        super().__init__(message, "CONTENT_RETRIEVAL_ERROR", details)


class ResponseGenerationError(RAGError):
    """Exception raised when response generation fails."""
    def __init__(self, message: str = "Failed to generate response", details: Optional[Dict[str, Any]] = None):
        super().__init__(message, "RESPONSE_GENERATION_ERROR", details)


class SessionError(RAGError):
    """Exception raised when session management fails."""
    def __init__(self, message: str = "Session error occurred", details: Optional[Dict[str, Any]] = None):
        super().__init__(message, "SESSION_ERROR", details)


def handle_rag_errors(func):
    """
    Decorator to handle RAG system errors.
    """
    @wraps(func)
    async def wrapper(*args, **kwargs):
        try:
            return await func(*args, **kwargs)
        except RAGError as e:
            logger.error(f"RAG error in {func.__name__}: {e.message}")
            raise
        except HTTPException:
            # Re-raise HTTP exceptions as-is
            raise
        except Exception as e:
            # Log unexpected errors
            error_msg = f"Unexpected error in {func.__name__}: {str(e)}"
            logger.error(error_msg)
            logger.error(traceback.format_exc())

            # Raise as a general RAG error
            raise RAGError(
                message="An unexpected error occurred",
                error_code="UNEXPECTED_ERROR",
                details={
                    "function": func.__name__,
                    "error": str(e),
                    "traceback": traceback.format_exc()
                }
            )
    return wrapper


async def global_exception_handler(request: Request, exc: Exception) -> JSONResponse:
    """
    Global exception handler for the FastAPI application.
    """
    if isinstance(exc, RAGError):
        return JSONResponse(
            status_code=500,
            content=exc.to_dict()
        )
    elif isinstance(exc, HTTPException):
        return JSONResponse(
            status_code=exc.status_code,
            content={
                "error_code": f"HTTP_{exc.status_code}",
                "message": exc.detail,
                "timestamp": datetime.utcnow().isoformat()
            }
        )
    else:
        logger.error(f"Unhandled exception: {exc}")
        logger.error(traceback.format_exc())

        return JSONResponse(
            status_code=500,
            content={
                "error_code": "INTERNAL_ERROR",
                "message": "An internal server error occurred",
                "timestamp": datetime.utcnow().isoformat()
            }
        )


class PerformanceMonitor:
    """
    Utility class for monitoring performance metrics.
    """

    def __init__(self):
        self.metrics = {}

    async def measure_execution_time(self, func_name: str, func, *args, **kwargs):
        """
        Measure execution time of a function.
        """
        import time

        start_time = time.time()
        try:
            result = await func(*args, **kwargs)
            execution_time = time.time() - start_time

            # Store metrics
            if func_name not in self.metrics:
                self.metrics[func_name] = []
            self.metrics[func_name].append(execution_time)

            logger.info(f"{func_name} executed in {execution_time:.4f}s")
            return result
        except Exception as e:
            execution_time = time.time() - start_time
            logger.error(f"{func_name} failed after {execution_time:.4f}s: {e}")
            raise

    def get_average_execution_time(self, func_name: str) -> Optional[float]:
        """
        Get average execution time for a function.
        """
        if func_name in self.metrics and self.metrics[func_name]:
            times = self.metrics[func_name]
            return sum(times) / len(times)
        return None

    def get_metrics_summary(self) -> Dict[str, Any]:
        """
        Get a summary of all collected metrics.
        """
        summary = {}
        for func_name, times in self.metrics.items():
            if times:
                summary[func_name] = {
                    "count": len(times),
                    "average_time": sum(times) / len(times),
                    "min_time": min(times),
                    "max_time": max(times),
                    "total_time": sum(times)
                }
        return summary


# Create a global performance monitor instance
performance_monitor = PerformanceMonitor()


def log_performance(func_name: Optional[str] = None):
    """
    Decorator to log performance metrics for a function.
    """
    def decorator(func):
        name = func_name or func.__name__

        @wraps(func)
        async def wrapper(*args, **kwargs):
            return await performance_monitor.measure_execution_time(name, func, *args, **kwargs)
        return wrapper
    return decorator


def setup_error_handlers(app):
    """
    Setup error handlers for the FastAPI application.
    """
    app.add_exception_handler(Exception, global_exception_handler)

    # Add specific handlers for different exception types if needed
    # app.add_exception_handler(RAGError, specific_rag_error_handler)

    return app