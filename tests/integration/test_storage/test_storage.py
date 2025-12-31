"""Integration tests for the storage module."""
import pytest
from unittest.mock import patch, MagicMock
from src.ingestion.storage import Storage
from src.models.content_segment import ContentSegment
from src.models.metadata import Metadata
from src.models.ingestion_log import IngestionLog
from datetime import datetime
import uuid


class TestStorageIntegration:
    """Test cases for the Storage class integration."""

    @patch('src.ingestion.storage.qdrant_client.QdrantClient')
    @patch('src.ingestion.storage.psycopg2.connect')
    def test_store_content_segments_with_embeddings(self, mock_db_connect, mock_qdrant_client):
        """Test storing content segments with embeddings."""
        # Setup mocks
        mock_qdrant_instance = MagicMock()
        mock_db_connection = MagicMock()
        mock_db_cursor = MagicMock()

        mock_qdrant_client.return_value = mock_qdrant_instance
        mock_db_connect.return_value = mock_db_connection
        mock_db_connection.cursor.return_value = mock_db_cursor

        # Create storage instance
        storage = Storage(collection_name="test_collection")

        # Create test content segments
        content_segment = ContentSegment(
            id="test_id_1",
            content="This is test content.",
            module="test_module",
            chapter="test_chapter",
            section="test_section",
            page_url="/test/page",
            token_count=10,
            hash="test_hash"
        )
        content_segments = [content_segment]

        # Create test embeddings
        embeddings = [[0.1, 0.2, 0.3, 0.4]]  # Example embedding vector

        # Call the method
        result = storage.store_content_segments_with_embeddings(content_segments, embeddings)

        # Assertions
        assert result["success"] is True
        assert result["stored_count"] == 1

        # Verify that the upsert method was called
        mock_qdrant_instance.upsert.assert_called_once()

    @patch('src.ingestion.storage.qdrant_client.QdrantClient')
    @patch('src.ingestion.storage.psycopg2.connect')
    def test_store_metadata(self, mock_db_connect, mock_qdrant_client):
        """Test storing metadata in Postgres."""
        # Setup mocks
        mock_db_connection = MagicMock()
        mock_db_cursor = MagicMock()

        mock_qdrant_client.return_value = MagicMock()
        mock_db_connect.return_value = mock_db_connection
        mock_db_connection.cursor.return_value = mock_db_cursor
        mock_db_connection.commit = MagicMock()

        # Create storage instance
        storage = Storage()

        # Create test metadata
        metadata = Metadata(
            id="test_metadata_id",
            content_segment_id="test_content_id",
            module="test_module",
            chapter="test_chapter",
            section="test_section",
            page_url="/test/page",
            source_file_path="/path/to/file",
            source_file_hash="test_hash"
        )
        metadata_records = [metadata]

        # Call the method
        result = storage.store_metadata(metadata_records)

        # Assertions
        assert result["success"] is True
        assert result["stored_count"] == 1

        # Verify that the cursor execute method was called
        assert mock_db_cursor.execute.called
        mock_db_connection.commit.assert_called_once()

    @patch('src.ingestion.storage.qdrant_client.QdrantClient')
    @patch('src.ingestion.storage.psycopg2.connect')
    def test_store_ingestion_log(self, mock_db_connect, mock_qdrant_client):
        """Test storing ingestion logs in Postgres."""
        # Setup mocks
        mock_db_connection = MagicMock()
        mock_db_cursor = MagicMock()

        mock_qdrant_client.return_value = MagicMock()
        mock_db_connect.return_value = mock_db_connection
        mock_db_connection.cursor.return_value = mock_db_cursor
        mock_db_connection.commit = MagicMock()

        # Create storage instance
        storage = Storage()

        # Create test ingestion log
        ingestion_log = IngestionLog(
            id="test_log_id",
            run_id="test_run_id",
            source_file_path="/path/to/file",
            status="success",
            content_segments_created=5
        )

        # Call the method
        result = storage.store_ingestion_log(ingestion_log)

        # Assertions
        assert result["success"] is True
        assert result["stored_id"] == "test_log_id"

        # Verify that the cursor execute method was called
        assert mock_db_cursor.execute.called
        mock_db_connection.commit.assert_called_once()

    @patch('src.ingestion.storage.qdrant_client.QdrantClient')
    @patch('src.ingestion.storage.psycopg2.connect')
    def test_create_database_schema(self, mock_db_connect, mock_qdrant_client):
        """Test creating database schema."""
        # Setup mocks
        mock_db_connection = MagicMock()
        mock_db_cursor = MagicMock()

        mock_qdrant_client.return_value = MagicMock()
        mock_db_connect.return_value = mock_db_connection
        mock_db_connection.cursor.return_value = mock_db_cursor
        mock_db_connection.commit = MagicMock()

        # Create storage instance
        storage = Storage()

        # Call the method
        storage.create_database_schema()

        # Verify that the cursor execute method was called for both tables
        assert mock_db_cursor.execute.call_count >= 2  # At least 2 calls for the 2 tables
        mock_db_connection.commit.assert_called_once()