"""Qdrant and Postgres storage logic."""
import qdrant_client
from qdrant_client.http import models
from qdrant_client.models import PointStruct
from typing import List, Dict, Any
import psycopg2
from psycopg2.extras import RealDictCursor
import json
from datetime import datetime
from src.models.content_segment import ContentSegment
from src.models.metadata import Metadata
from src.config.settings import settings
import uuid


class Storage:
    """Class to handle storage of embeddings in Qdrant Cloud and metadata in Neon Postgres."""

    def __init__(self, collection_name: str = settings.QDRANT_COLLECTION_NAME):
        """
        Initialize the storage handler.

        Args:
            collection_name: Name of the Qdrant collection (default: from settings)
        """
        # Initialize Qdrant client
        self.qdrant_client = qdrant_client.QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY,
        )
        self.collection_name = collection_name

        # Initialize Postgres connection
        self.db_connection_string = settings.DATABASE_URL

        # Create collection if it doesn't exist
        self._create_qdrant_collection_if_not_exists()

    def _create_qdrant_collection_if_not_exists(self):
        """Create the Qdrant collection if it doesn't exist."""
        try:
            # Try to get collection info to see if it exists
            self.qdrant_client.get_collection(self.collection_name)
        except Exception:
            # Collection doesn't exist, create it
            # Cohere embeddings for embed-english-v3.0 are 1024 dimensions
            self.qdrant_client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=1024,
                    distance=models.Distance.COSINE
                )
            )

    def store_content_segments_with_embeddings(self, content_segments: List[ContentSegment], embeddings: List[List[float]]) -> Dict[str, Any]:
        """
        Store content segments with their embeddings in Qdrant Cloud.

        Args:
            content_segments: List of ContentSegment objects
            embeddings: List of embedding vectors

        Returns:
            Dictionary with success status and stored count
        """
        if len(content_segments) != len(embeddings):
            raise ValueError("Number of content segments must match number of embeddings")

        try:
            points = []
            for i, segment in enumerate(content_segments):
                # Prepare the payload with metadata
                payload = {
                    "content": segment.content,
                    "module": segment.module,
                    "chapter": segment.chapter,
                    "section": segment.section,
                    "page_url": segment.page_url,
                    "token_count": segment.token_count,
                    "hash": segment.hash,
                    "created_at": segment.created_at.isoformat(),
                    "updated_at": segment.updated_at.isoformat()
                }

                # Create a PointStruct for Qdrant with the actual embedding vector
                point = PointStruct(
                    id=segment.id,
                    vector=embeddings[i],  # Actual embedding vector
                    payload=payload
                )
                points.append(point)

            # Upload points to Qdrant
            self.qdrant_client.upsert(
                collection_name=self.collection_name,
                points=points
            )

            return {
                "success": True,
                "stored_count": len(content_segments)
            }
        except Exception as e:
            print(f"Error storing content segments with embeddings in Qdrant: {e}")
            return {
                "success": False,
                "stored_count": 0,
                "error": str(e)
            }

    def store_embeddings(self, embeddings: List[Dict], content_segments: List[ContentSegment]) -> Dict[str, Any]:
        """
        Store embeddings in Qdrant Cloud.

        Args:
            embeddings: List of embedding data (from Cohere) - each dict should have 'vector' and 'content_segment_id'
            content_segments: Corresponding content segments

        Returns:
            Dictionary with success status and stored count
        """
        try:
            points = []
            for i, embedding_data in enumerate(embeddings):
                if i < len(content_segments):
                    segment = content_segments[i]
                    # Prepare the payload with metadata
                    payload = {
                        "content": segment.content,
                        "module": segment.module,
                        "chapter": segment.chapter,
                        "section": segment.section,
                        "page_url": segment.page_url,
                        "token_count": segment.token_count,
                        "hash": segment.hash,
                        "created_at": segment.created_at.isoformat(),
                        "updated_at": segment.updated_at.isoformat()
                    }

                    # Create a PointStruct for Qdrant with the actual embedding vector
                    point = PointStruct(
                        id=segment.id,  # Use the same ID as the content segment
                        vector=embedding_data['vector'],  # The actual embedding vector
                        payload=payload
                    )
                    points.append(point)

            # Upload points to Qdrant
            self.qdrant_client.upsert(
                collection_name=self.collection_name,
                points=points
            )

            return {
                "success": True,
                "stored_count": len(points)
            }
        except Exception as e:
            print(f"Error storing embeddings in Qdrant: {e}")
            return {
                "success": False,
                "stored_count": 0,
                "error": str(e)
            }

    def store_metadata(self, metadata_records: List[Metadata]) -> Dict[str, Any]:
        """
        Store metadata and ingestion logs in Neon Postgres.

        Args:
            metadata_records: List of Metadata objects

        Returns:
            Dictionary with success status and stored count
        """
        try:
            # Connect to Postgres
            conn = psycopg2.connect(self.db_connection_string)
            cursor = conn.cursor(cursor_factory=RealDictCursor)

            # Create metadata table if it doesn't exist
            create_metadata_table_query = """
            CREATE TABLE IF NOT EXISTS content_metadata (
                id VARCHAR(255) PRIMARY KEY,
                content_segment_id VARCHAR(255) NOT NULL,
                module VARCHAR(255) NOT NULL,
                chapter VARCHAR(255) NOT NULL,
                section VARCHAR(255) NOT NULL,
                page_url TEXT NOT NULL,
                source_file_path TEXT NOT NULL,
                source_file_hash VARCHAR(255) NOT NULL,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            );
            """
            cursor.execute(create_metadata_table_query)

            # Insert metadata records
            insert_query = """
            INSERT INTO content_metadata (
                id, content_segment_id, module, chapter, section,
                page_url, source_file_path, source_file_hash, created_at, updated_at
            ) VALUES (
                %(id)s, %(content_segment_id)s, %(module)s, %(chapter)s, %(section)s,
                %(page_url)s, %(source_file_path)s, %(source_file_hash)s, %(created_at)s, %(updated_at)s
            ) ON CONFLICT (id) DO UPDATE SET
                content_segment_id = EXCLUDED.content_segment_id,
                module = EXCLUDED.module,
                chapter = EXCLUDED.chapter,
                section = EXCLUDED.section,
                page_url = EXCLUDED.page_url,
                source_file_path = EXCLUDED.source_file_path,
                source_file_hash = EXCLUDED.source_file_hash,
                updated_at = EXCLUDED.updated_at;
            """

            for metadata_record in metadata_records:
                cursor.execute(insert_query, {
                    'id': metadata_record.id,
                    'content_segment_id': metadata_record.content_segment_id,
                    'module': metadata_record.module,
                    'chapter': metadata_record.chapter,
                    'section': metadata_record.section,
                    'page_url': metadata_record.page_url,
                    'source_file_path': metadata_record.source_file_path,
                    'source_file_hash': metadata_record.source_file_hash,
                    'created_at': metadata_record.created_at,
                    'updated_at': metadata_record.updated_at
                })

            conn.commit()
            cursor.close()
            conn.close()

            return {
                "success": True,
                "stored_count": len(metadata_records)
            }
        except Exception as e:
            print(f"Error storing metadata in Postgres: {e}")
            if 'conn' in locals():
                conn.rollback()
                cursor.close()
                conn.close()
            return {
                "success": False,
                "stored_count": 0,
                "error": str(e)
            }

    def create_database_schema(self):
        """Create the necessary database schema for metadata and ingestion logs."""
        try:
            # Connect to Postgres
            conn = psycopg2.connect(self.db_connection_string)
            cursor = conn.cursor(cursor_factory=RealDictCursor)

            # Create ingestion logs table
            create_logs_table_query = """
            CREATE TABLE IF NOT EXISTS ingestion_logs (
                id VARCHAR(255) PRIMARY KEY,
                run_id VARCHAR(255) NOT NULL,
                source_file_path TEXT NOT NULL,
                status VARCHAR(50) NOT NULL CHECK (status IN ('success', 'error', 'skipped')),
                error_message TEXT,
                content_segments_created INTEGER DEFAULT 0,
                start_time TIMESTAMP,
                end_time TIMESTAMP,
                duration_ms INTEGER,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            );
            """
            cursor.execute(create_logs_table_query)

            # Create metadata table (if not already created)
            create_metadata_table_query = """
            CREATE TABLE IF NOT EXISTS content_metadata (
                id VARCHAR(255) PRIMARY KEY,
                content_segment_id VARCHAR(255) NOT NULL,
                module VARCHAR(255) NOT NULL,
                chapter VARCHAR(255) NOT NULL,
                section VARCHAR(255) NOT NULL,
                page_url TEXT NOT NULL,
                source_file_path TEXT NOT NULL,
                source_file_hash VARCHAR(255) NOT NULL,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            );
            """
            cursor.execute(create_metadata_table_query)

            conn.commit()
            cursor.close()
            conn.close()

            print("Database schema created successfully")
        except Exception as e:
            print(f"Error creating database schema: {e}")
            if 'conn' in locals():
                conn.rollback()
                cursor.close()
                conn.close()
            raise e

    def store_ingestion_log(self, ingestion_log) -> Dict[str, Any]:
        """
        Store ingestion log in Neon Postgres.

        Args:
            ingestion_log: IngestionLog object

        Returns:
            Dictionary with success status
        """
        try:
            # Connect to Postgres
            conn = psycopg2.connect(self.db_connection_string)
            cursor = conn.cursor(cursor_factory=RealDictCursor)

            # Insert ingestion log record
            insert_query = """
            INSERT INTO ingestion_logs (
                id, run_id, source_file_path, status, error_message,
                content_segments_created, start_time, end_time, duration_ms, created_at
            ) VALUES (
                %(id)s, %(run_id)s, %(source_file_path)s, %(status)s, %(error_message)s,
                %(content_segments_created)s, %(start_time)s, %(end_time)s, %(duration_ms)s, %(created_at)s
            ) ON CONFLICT (id) DO UPDATE SET
                status = EXCLUDED.status,
                error_message = EXCLUDED.error_message,
                content_segments_created = EXCLUDED.content_segments_created,
                start_time = EXCLUDED.start_time,
                end_time = EXCLUDED.end_time,
                duration_ms = EXCLUDED.duration_ms,
                created_at = EXCLUDED.created_at;
            """

            cursor.execute(insert_query, {
                'id': ingestion_log.id,
                'run_id': ingestion_log.run_id,
                'source_file_path': ingestion_log.source_file_path,
                'status': ingestion_log.status,
                'error_message': ingestion_log.error_message,
                'content_segments_created': ingestion_log.content_segments_created,
                'start_time': ingestion_log.start_time,
                'end_time': ingestion_log.end_time,
                'duration_ms': ingestion_log.duration_ms,
                'created_at': ingestion_log.created_at
            })

            conn.commit()
            cursor.close()
            conn.close()

            return {
                "success": True,
                "stored_id": ingestion_log.id
            }
        except Exception as e:
            print(f"Error storing ingestion log in Postgres: {e}")
            if 'conn' in locals():
                conn.rollback()
                cursor.close()
                conn.close()
            return {
                "success": False,
                "error": str(e)
            }


def store_content_segments_with_embeddings_in_qdrant(content_segments: List[ContentSegment], embeddings: List[List[float]], collection_name: str = settings.QDRANT_COLLECTION_NAME) -> Dict[str, Any]:
    """
    Function to store content segments with their embeddings in Qdrant Cloud.

    Args:
        content_segments: List of ContentSegment objects
        embeddings: List of embedding vectors
        collection_name: Name of the Qdrant collection

    Returns:
        Dictionary with success status and stored count
    """
    storage = Storage(collection_name)
    return storage.store_content_segments_with_embeddings(content_segments, embeddings)


def store_embeddings_in_qdrant(embeddings: List[Dict], content_segments: List[ContentSegment], collection_name: str = settings.QDRANT_COLLECTION_NAME) -> Dict[str, Any]:
    """
    Function to store embeddings in Qdrant Cloud.

    Args:
        embeddings: List of embedding data (from Cohere) - each dict should have 'vector' and 'content_segment_id'
        content_segments: Corresponding content segments
        collection_name: Name of the Qdrant collection

    Returns:
        Dictionary with success status and stored count
    """
    storage = Storage(collection_name)
    return storage.store_embeddings(embeddings, content_segments)


def store_metadata_in_postgres(metadata_records: List[Metadata]) -> Dict[str, Any]:
    """
    Function to store metadata and ingestion logs in Neon Postgres.

    Args:
        metadata_records: List of Metadata objects

    Returns:
        Dictionary with success status and stored count
    """
    storage = Storage()
    return storage.store_metadata(metadata_records)


def create_database_schema():
    """Create the necessary database schema for metadata and ingestion logs."""
    storage = Storage()
    storage.create_database_schema()