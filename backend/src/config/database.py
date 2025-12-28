"""
Database configuration for the RAG Chatbot API.

This module handles the database connection setup for Neon Postgres
and provides centralized access to database configuration values.
"""

from typing import Optional
from .settings import settings
import asyncpg


class DatabaseConfig:
    """
    Database configuration and connection management.
    """

    def __init__(self):
        self.database_url = settings.database_url
        self.pool: Optional[asyncpg.Pool] = None

    async def init_db_pool(self):
        """
        Initialize the database connection pool.
        """
        if not self.database_url:
            raise ValueError("DATABASE_URL environment variable is not set")

        self.pool = await asyncpg.create_pool(
            dsn=self.database_url,
            min_size=2,
            max_size=10,
            command_timeout=60,
        )

    async def get_connection(self):
        """
        Get a database connection from the pool.
        """
        if not self.pool:
            await self.init_db_pool()

        return self.pool.acquire()

    async def close_pool(self):
        """
        Close the database connection pool.
        """
        if self.pool:
            await self.pool.close()


# Global database config instance
db_config = DatabaseConfig()