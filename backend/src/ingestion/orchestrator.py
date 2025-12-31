"""Main pipeline orchestration."""
from typing import List, Dict, Any
from src.ingestion.loader import load_docusaurus_content
from src.ingestion.chunker import chunk_text_content
from src.ingestion.embedder import generate_embeddings_for_texts
from src.ingestion.storage import store_content_segments_with_embeddings_in_qdrant, store_metadata_in_postgres, Storage
from src.models.content_segment import ContentSegment
from src.models.metadata import Metadata
from src.models.ingestion_log import IngestionLog
from src.utils.hash_utils import calculate_content_hash, is_file_changed, get_stored_hash, store_file_hash
from src.config.settings import settings
import uuid
import time
from datetime import datetime


class PipelineOrchestrator:
    """Class to orchestrate the end-to-end ingestion pipeline."""

    def __init__(self):
        """Initialize the pipeline orchestrator."""
        pass

    def run_ingestion_pipeline(self, source_path: str = "/frontend/docs", collection_name: str = settings.QDRANT_COLLECTION_NAME, reprocess_updates: bool = True) -> Dict[str, Any]:
        """
        Run the end-to-end ingestion pipeline: load → parse → chunk → embed → store.

        Args:
            source_path: Path to Docusaurus docs directory
            collection_name: Name of Qdrant collection to use
            reprocess_updates: Whether to reprocess updated content (default: True)

        Returns:
            Dictionary with run_id and summary statistics for the run
        """
        run_id = str(uuid.uuid4())
        start_time = time.time()

        try:
            print(f"Starting ingestion pipeline run {run_id}...")
            print(f"Source path: {source_path}")
            print(f"Collection name: {collection_name}")
            print(f"Reprocess updates: {reprocess_updates}")

            # Step 1: Load content
            print("Step 1: Loading Docusaurus content...")
            content_files = load_docusaurus_content(source_path)
            print(f"Loaded {len(content_files)} content files")

            # Step 2: Process each content file
            all_content_segments = []
            all_metadata_records = []

            for content_file in content_files:
                file_path = content_file['path']
                relative_path = content_file['relative_path']
                content_text = content_file['content']
                metadata = content_file['metadata']
                structural_metadata = content_file['structural_metadata']

                # Check if file has changed if reprocessing is enabled
                should_process = True
                file_start_time = time.time()
                file_error_message = None
                file_segments_created = 0

                if reprocess_updates:
                    stored_hash = get_stored_hash(file_path)
                    should_process = is_file_changed(file_path, stored_hash)
                    if not should_process:
                        print(f"Skipping unchanged file: {relative_path}")
                        # Log the skipped file
                        ingestion_log = IngestionLog(
                            id=str(uuid.uuid4()),
                            run_id=run_id,
                            source_file_path=file_path,
                            status="skipped",
                            error_message=None,
                            content_segments_created=0
                        )
                        storage = Storage()
                        storage.store_ingestion_log(ingestion_log)
                        continue  # Skip unchanged files

                print(f"Processing file: {relative_path}")

                try:
                    # Step 2a: Chunk content
                    content_segments = chunk_text_content(content_text, structural_metadata)
                    file_segments_created = len(content_segments)
                    print(f"Created {len(content_segments)} content segments from {relative_path}")

                    # Step 2b: Create metadata records
                    for segment in content_segments:
                        metadata_record = Metadata(
                            id=str(uuid.uuid4()),
                            content_segment_id=segment.id,
                            module=segment.module,
                            chapter=segment.chapter,
                            section=segment.section,
                            page_url=segment.page_url,
                            source_file_path=file_path,
                            source_file_hash=calculate_content_hash(content_text)
                        )
                        all_metadata_records.append(metadata_record)

                    all_content_segments.extend(content_segments)

                    # Store the hash of the processed file
                    if should_process:
                        file_hash = calculate_content_hash(content_text)
                        store_file_hash(file_path, file_hash)

                except Exception as file_error:
                    file_error_message = str(file_error)
                    print(f"Error processing file {relative_path}: {file_error_message}")

                finally:
                    # Log the file processing result
                    file_end_time = time.time()
                    file_duration_ms = int((file_end_time - file_start_time) * 1000)

                    status = "error" if file_error_message else "success"
                    ingestion_log = IngestionLog(
                        id=str(uuid.uuid4()),
                        run_id=run_id,
                        source_file_path=file_path,
                        status=status,
                        error_message=file_error_message,
                        content_segments_created=file_segments_created,
                        start_time=datetime.fromtimestamp(file_start_time),
                        end_time=datetime.fromtimestamp(file_end_time),
                        duration_ms=file_duration_ms
                    )

                    storage = Storage()
                    storage.store_ingestion_log(ingestion_log)

            # Step 3: Generate embeddings
            if all_content_segments:
                print(f"Generating embeddings for {len(all_content_segments)} content segments...")
                text_chunks = [segment.content for segment in all_content_segments]
                embeddings = generate_embeddings_for_texts(text_chunks)
                print(f"Generated {len(embeddings)} embeddings")

                # Extract vectors from embeddings for storage
                embedding_vectors = [embedding.vector for embedding in embeddings]

                # Step 4: Store content segments with embeddings in Qdrant
                print("Storing content segments with embeddings in Qdrant...")
                qdrant_result = store_content_segments_with_embeddings_in_qdrant(
                    all_content_segments,
                    embedding_vectors,
                    collection_name
                )
                print(f"Qdrant storage result: {qdrant_result}")
            else:
                print("No content segments to process, skipping embedding generation and storage")
                qdrant_result = {"success": True, "stored_count": 0}

            # Step 5: Store metadata in Postgres
            print("Storing metadata in Postgres...")
            postgres_result = store_metadata_in_postgres(all_metadata_records)
            print(f"Postgres storage result: {postgres_result}")

            # Step 6: Calculate summary
            end_time = time.time()
            duration = end_time - start_time

            summary = {
                "run_id": run_id,
                "files_processed": len(content_files),
                "content_segments_created": len(all_content_segments),
                "embeddings_generated": len(all_content_segments) if all_content_segments else 0,  # Only if we processed content
                "qdrant_result": qdrant_result,
                "postgres_result": postgres_result,
                "duration_seconds": duration,
                "start_time": start_time,
                "end_time": end_time,
                "status": "success"
            }

            print(f"Ingestion pipeline completed successfully in {duration:.2f} seconds")
            return summary

        except Exception as e:
            end_time = time.time()
            duration = end_time - start_time

            error_summary = {
                "run_id": run_id,
                "status": "error",
                "error_message": str(e),
                "duration_seconds": duration,
                "start_time": start_time,
                "end_time": end_time
            }

            print(f"Ingestion pipeline failed: {str(e)}")
            return error_summary

    def add_deterministic_processing(self):
        """
        Add deterministic processing to ensure consistent results.
        This is implemented by using consistent hashing and processing order.
        """
        # This is inherently handled by the processing steps which are deterministic
        # given the same input data
        pass


def run_ingestion_pipeline(source_path: str = "/frontend/docs", collection_name: str = settings.QDRANT_COLLECTION_NAME, reprocess_updates: bool = True) -> Dict[str, Any]:
    """
    Function to run the end-to-end ingestion pipeline: load → parse → chunk → embed → store.

    Args:
        source_path: Path to Docusaurus docs directory
        collection_name: Name of Qdrant collection to use
        reprocess_updates: Whether to reprocess updated content (default: True)

    Returns:
        Dictionary with run_id and summary statistics for the run
    """
    orchestrator = PipelineOrchestrator()
    return orchestrator.run_ingestion_pipeline(source_path, collection_name, reprocess_updates)