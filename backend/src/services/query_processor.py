"""
Query processor for the RAG Chatbot API.

This module handles query processing and scope determination
for the RAG system.
"""

from typing import Optional
from ..models.user_query import UserQuery, QueryScope
from ..config.settings import settings
import logging
import re


class QueryProcessor:
    """
    Service class for processing user queries and determining retrieval scope.
    """

    def __init__(self):
        self.max_query_length = settings.max_tokens  # Using max_tokens as a proxy for query length limit
        self.scope_patterns = {
            QueryScope.MODULE: [
                r"module\s+([a-zA-Z0-9_-]+)",
                r"in\s+module\s+([a-zA-Z0-9_-]+)",
                r"about\s+module\s+([a-zA-Z0-9_-]+)"
            ],
            QueryScope.CHAPTER: [
                r"chapter\s+([a-zA-Z0-9_-]+)",
                r"in\s+chapter\s+([a-zA-Z0-9_-]+)",
                r"about\s+chapter\s+([a-zA-Z0-9_-]+)",
                r"section\s+([a-zA-Z0-9_-]+)"  # Treating sections similarly to chapters
            ]
        }

    async def process_query(self, question: str, session_id: str, **kwargs) -> UserQuery:
        """
        Process a user query and determine its scope and filters.

        Args:
            question: The user's question
            session_id: The session ID
            **kwargs: Additional parameters like module, chapter, selected_text

        Returns:
            UserQuery object with processed information
        """
        try:
            # Validate query length
            if len(question) > self.max_query_length:
                raise ValueError(f"Query exceeds maximum length of {self.max_query_length} characters")

            # Determine the scope of the query
            scope, scope_filter = await self.determine_scope(question, **kwargs)

            # Create query ID
            query_id = f"query-{int(self._get_timestamp())}"

            # Create UserQuery object
            user_query = UserQuery(
                query_id=query_id,
                session_id=session_id,
                question=question.strip(),
                scope=scope,
                scope_filter=scope_filter,
                timestamp=self._get_timestamp(),
                selected_text=kwargs.get('selected_text'),
                metadata=kwargs.get('metadata', {})
            )

            logging.info(f"Processed query '{question[:50]}...' with scope {scope} for session {session_id}")
            return user_query

        except Exception as e:
            logging.error(f"Error processing query: {str(e)}")
            raise

    async def determine_scope(self, question: str, **kwargs) -> tuple[QueryScope, Optional[str]]:
        """
        Determine the scope of the query based on the question text and provided filters.

        Args:
            question: The user's question
            **kwargs: Additional parameters like module, chapter

        Returns:
            Tuple of (QueryScope, scope_filter)
        """
        # Check if specific scope was provided as a parameter
        if kwargs.get('module'):
            return QueryScope.MODULE, kwargs['module']
        elif kwargs.get('chapter'):
            return QueryScope.CHAPTER, kwargs['chapter']
        elif kwargs.get('selected_text'):
            return QueryScope.SELECTED_TEXT, None

        # If no specific scope was provided, try to infer from the question text
        question_lower = question.lower()

        # Check for module patterns
        for pattern in self.scope_patterns[QueryScope.MODULE]:
            match = re.search(pattern, question_lower)
            if match:
                return QueryScope.MODULE, match.group(1)

        # Check for chapter patterns
        for pattern in self.scope_patterns[QueryScope.CHAPTER]:
            match = re.search(pattern, question_lower)
            if match:
                return QueryScope.CHAPTER, match.group(1)

        # Default to global scope if no specific scope is determined
        return QueryScope.GLOBAL, None

    async def validate_query(self, user_query: UserQuery) -> list[str]:
        """
        Validate the user query.

        Args:
            user_query: The UserQuery object to validate

        Returns:
            List of validation errors if any
        """
        errors = []

        # Check if question is empty
        if not user_query.question or not user_query.question.strip():
            errors.append("Question cannot be empty")

        # Check if question is too long
        if len(user_query.question) > self.max_query_length:
            errors.append(f"Question exceeds maximum length of {self.max_query_length} characters")

        # Validate scope
        if user_query.scope not in QueryScope.__members__.values():
            errors.append(f"Invalid query scope: {user_query.scope}")

        # Validate scope filter if present
        if user_query.scope_filter:
            # Basic validation - should be alphanumeric with hyphens/underscores
            if not re.match(r'^[a-zA-Z0-9_-]+$', user_query.scope_filter):
                errors.append("Scope filter contains invalid characters")

        # For selected text queries, ensure selected_text is provided
        if user_query.scope == QueryScope.SELECTED_TEXT and not user_query.selected_text:
            errors.append("Selected text is required for selected_text scope")

        return errors

    def _get_timestamp(self) -> int:
        """
        Get current timestamp as integer.

        Returns:
            Current timestamp as integer
        """
        import time
        return int(time.time())


# Global query processor instance
query_processor = QueryProcessor()