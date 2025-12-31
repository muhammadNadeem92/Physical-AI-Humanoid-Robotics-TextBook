"""CLI entry point for the ingestion pipeline."""
import argparse
import sys
import os
from typing import Optional
from src.ingestion.orchestrator import run_ingestion_pipeline
from src.config.settings import settings


def main():
    """Main entry point for the CLI."""
    # Validate settings
    validation_errors = settings.validate()
    if validation_errors:
        print("Configuration errors found:")
        for error in validation_errors:
            print(f"  - {error}")
        sys.exit(1)

    parser = argparse.ArgumentParser(description="Content Ingestion Pipeline for RAG Chatbot")
    subparsers = parser.add_subparsers(dest="command", help="Available commands")

    # Ingest command
    ingest_parser = subparsers.add_parser("ingest", help="Run the content ingestion pipeline")
    ingest_parser.add_argument(
        "--source-path",
        type=str,
        default="/frontend/docs",
        help="Path to the Docusaurus docs directory (default: /frontend/docs)"
    )
    ingest_parser.add_argument(
        "--collection-name",
        type=str,
        default=settings.QDRANT_COLLECTION_NAME,
        help=f"Name of the Qdrant collection (default: {settings.QDRANT_COLLECTION_NAME})"
    )
    ingest_parser.add_argument(
        "--no-reprocess",
        action="store_true",
        help="Disable reprocessing of updated content"
    )

    # Parse arguments
    args = parser.parse_args()

    if args.command == "ingest":
        print("Starting ingestion pipeline...")
        print(f"Source path: {args.source_path}")
        print(f"Collection name: {args.collection_name}")
        print(f"Reprocess updates: {not args.no_reprocess}")

        # Run the ingestion pipeline
        result = run_ingestion_pipeline(
            source_path=args.source_path,
            collection_name=args.collection_name,
            reprocess_updates=not args.no_reprocess
        )

        # Print results
        if result.get("status") == "success":
            print("\nIngestion completed successfully!")
            print(f"Run ID: {result.get('run_id')}")
            print(f"Files processed: {result.get('files_processed', 0)}")
            print(f"Content segments created: {result.get('content_segments_created', 0)}")
            print(f"Embeddings generated: {result.get('embeddings_generated', 0)}")
            print(f"Duration: {result.get('duration_seconds', 0):.2f} seconds")
        else:
            print(f"\nIngestion failed: {result.get('error_message')}")
            sys.exit(1)

    else:
        parser.print_help()
        sys.exit(1)


if __name__ == "__main__":
    main()