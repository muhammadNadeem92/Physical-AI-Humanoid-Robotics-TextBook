"""
Validation Utilities

Utilities for validating input data, parameters, and responses in the RAG system.
"""
import re
import logging
from typing import Any, Dict, List, Optional, Union
from datetime import datetime
from urllib.parse import urlparse

from pydantic import BaseModel, ValidationError, validator


logger = logging.getLogger(__name__)


class ValidationUtils:
    """
    Utilities for validating various types of input data.
    """

    @staticmethod
    def validate_session_id(session_id: str) -> bool:
        """
        Validate session ID format.

        Args:
            session_id: Session identifier to validate

        Returns:
            True if valid, False otherwise
        """
        if not session_id or not isinstance(session_id, str):
            return False

        # Session ID should be a UUID-like string or at least 8 characters
        if len(session_id.strip()) < 8:
            return False

        # Should not contain special characters that could be used for injection
        if re.search(r'[<>"\'&]', session_id):
            return False

        return True

    @staticmethod
    def validate_query_text(query: str) -> bool:
        """
        Validate user query text.

        Args:
            query: Query text to validate

        Returns:
            True if valid, False otherwise
        """
        if not query or not isinstance(query, str):
            return False

        query = query.strip()

        # Check length
        if len(query) < 3 or len(query) > 1000:
            return False

        # Check for potentially harmful content
        harmful_patterns = [
            r'<script',  # Potential XSS
            r'javascript:',  # Potential XSS
            r'vbscript:',  # Potential XSS
            r'on\w+\s*=',  # Potential XSS
            r'eval\s*\(',  # Potential code injection
            r'exec\s*\(',  # Potential code injection
        ]

        for pattern in harmful_patterns:
            if re.search(pattern, query, re.IGNORECASE):
                return False

        return True

    @staticmethod
    def validate_module_name(module: str) -> bool:
        """
        Validate module name.

        Args:
            module: Module name to validate

        Returns:
            True if valid, False otherwise
        """
        if not module or not isinstance(module, str):
            return False

        module = module.strip()

        # Check length
        if len(module) < 1 or len(module) > 100:
            return False

        # Should not contain path traversal characters
        if '..' in module or '/' in module or '\\' in module:
            return False

        # Should not contain special characters that could be used for injection
        if re.search(r'[<>"\'&]', module):
            return False

        return True

    @staticmethod
    def validate_chapter_name(chapter: str) -> bool:
        """
        Validate chapter name.

        Args:
            chapter: Chapter name to validate

        Returns:
            True if valid, False otherwise
        """
        if not chapter or not isinstance(chapter, str):
            return False

        chapter = chapter.strip()

        # Check length
        if len(chapter) < 1 or len(chapter) > 100:
            return False

        # Should not contain path traversal characters
        if '..' in chapter or '/' in chapter or '\\' in chapter:
            return False

        # Should not contain special characters that could be used for injection
        if re.search(r'[<>"\'&]', chapter):
            return False

        return True

    @staticmethod
    def validate_selected_text(selected_text: str) -> bool:
        """
        Validate selected text.

        Args:
            selected_text: Selected text to validate

        Returns:
            True if valid, False otherwise
        """
        if not selected_text or not isinstance(selected_text, str):
            return False

        selected_text = selected_text.strip()

        # Check length
        if len(selected_text) < 1 or len(selected_text) > 10000:
            return False

        # Check for potentially harmful content
        harmful_patterns = [
            r'<script',  # Potential XSS
            r'javascript:',  # Potential XSS
            r'vbscript:',  # Potential XSS
            r'on\w+\s*=',  # Potential XSS
        ]

        for pattern in harmful_patterns:
            if re.search(pattern, selected_text, re.IGNORECASE):
                return False

        return True

    @staticmethod
    def validate_feedback_type(feedback_type: str) -> bool:
        """
        Validate feedback type.

        Args:
            feedback_type: Feedback type to validate

        Returns:
            True if valid, False otherwise
        """
        if not feedback_type or not isinstance(feedback_type, str):
            return False

        valid_feedback_types = [
            "positive", "negative", "thumbs_up", "thumbs_down",
            "helpful", "unhelpful", "accurate", "inaccurate",
            "relevant", "irrelevant", "clear", "unclear"
        ]

        return feedback_type.lower() in [f.lower() for f in valid_feedback_types]

    @staticmethod
    def validate_url(url: str) -> bool:
        """
        Validate URL format.

        Args:
            url: URL to validate

        Returns:
            True if valid, False otherwise
        """
        if not url or not isinstance(url, str):
            return False

        try:
            result = urlparse(url)
            return all([result.scheme, result.netloc])
        except Exception:
            return False

    @staticmethod
    def validate_email(email: str) -> bool:
        """
        Validate email format.

        Args:
            email: Email to validate

        Returns:
            True if valid, False otherwise
        """
        if not email or not isinstance(email, str):
            return False

        pattern = r'^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$'
        return re.match(pattern, email) is not None

    @staticmethod
    def validate_json(json_str: str) -> bool:
        """
        Validate JSON string.

        Args:
            json_str: JSON string to validate

        Returns:
            True if valid, False otherwise
        """
        if not json_str or not isinstance(json_str, str):
            return False

        try:
            import json
            json.loads(json_str)
            return True
        except ValueError:
            return False

    @staticmethod
    def sanitize_input(input_str: str) -> str:
        """
        Sanitize input string by removing potentially harmful content.

        Args:
            input_str: Input string to sanitize

        Returns:
            Sanitized string
        """
        if not input_str or not isinstance(input_str, str):
            return input_str

        # Remove potentially harmful characters/patterns
        sanitized = input_str

        # Remove script tags
        sanitized = re.sub(r'<script[^>]*>.*?</script>', '', sanitized, flags=re.IGNORECASE | re.DOTALL)

        # Remove event handlers
        sanitized = re.sub(r'on\w+\s*=\s*["\'][^"\']*["\']', '', sanitized, flags=re.IGNORECASE)

        # Remove javascript/vbscript protocols
        sanitized = re.sub(r'(javascript|vbscript):', '', sanitized, flags=re.IGNORECASE)

        return sanitized

    @staticmethod
    def validate_response_content(response_text: str) -> Dict[str, Any]:
        """
        Validate response content for quality and safety.

        Args:
            response_text: Response text to validate

        Returns:
            Dictionary with validation results
        """
        if not response_text or not isinstance(response_text, str):
            return {
                "is_valid": False,
                "errors": ["Response text is empty or not a string"],
                "warnings": []
            }

        errors = []
        warnings = []

        # Check for potential issues
        if len(response_text) > 10000:
            warnings.append("Response is very long")

        if "I don't know" in response_text or "I'm not sure" in response_text:
            warnings.append("Response indicates uncertainty")

        # Check for potential hallucinations (responses that claim to know something not in context)
        hallucination_indicators = [
            r"I think",
            r"I believe",
            r"probably",
            r"possibly",
            r"might be"
        ]

        for indicator in hallucination_indicators:
            if re.search(indicator, response_text, re.IGNORECASE):
                warnings.append(f"Response contains potential hallucination indicator: {indicator}")

        return {
            "is_valid": len(errors) == 0,
            "errors": errors,
            "warnings": warnings
        }

    @staticmethod
    def validate_citations(citations: List[str]) -> Dict[str, Any]:
        """
        Validate citations list.

        Args:
            citations: List of citations to validate

        Returns:
            Dictionary with validation results
        """
        if not isinstance(citations, list):
            return {
                "is_valid": False,
                "errors": ["Citations must be a list"],
                "warnings": []
            }

        errors = []
        warnings = []

        for i, citation in enumerate(citations):
            if not isinstance(citation, str):
                errors.append(f"Citation at index {i} is not a string")
                continue

            if not ValidationUtils.validate_url(citation):
                warnings.append(f"Citation at index {i} is not a valid URL: {citation}")

        return {
            "is_valid": len(errors) == 0,
            "errors": errors,
            "warnings": warnings
        }

    @staticmethod
    def validate_query_params(
        question: str,
        session_id: str,
        module: Optional[str] = None,
        chapter: Optional[str] = None,
        selected_text: Optional[str] = None
    ) -> Dict[str, Any]:
        """
        Validate all query parameters together.

        Args:
            question: User's question
            session_id: Session identifier
            module: Optional module name
            chapter: Optional chapter name
            selected_text: Optional selected text

        Returns:
            Dictionary with validation results
        """
        errors = []
        warnings = []

        # Validate question
        if not ValidationUtils.validate_query_text(question):
            errors.append("Invalid question text")

        # Validate session ID
        if not ValidationUtils.validate_session_id(session_id):
            errors.append("Invalid session ID")

        # Validate module if provided
        if module and not ValidationUtils.validate_module_name(module):
            errors.append("Invalid module name")

        # Validate chapter if provided
        if chapter and not ValidationUtils.validate_chapter_name(chapter):
            errors.append("Invalid chapter name")

        # Validate selected text if provided
        if selected_text and not ValidationUtils.validate_selected_text(selected_text):
            errors.append("Invalid selected text")

        # Additional checks
        if module and chapter and selected_text:
            warnings.append("Both module/chapter and selected_text provided - this might be confusing")

        return {
            "is_valid": len(errors) == 0,
            "errors": errors,
            "warnings": warnings
        }