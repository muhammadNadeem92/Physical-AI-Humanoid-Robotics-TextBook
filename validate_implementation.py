#!/usr/bin/env python3
"""
Validation script to test the basic functionality of the content ingestion pipeline.
"""
import os
import sys
from datetime import datetime
from src.config.settings import settings
from src.models.content_segment import ContentSegment
from src.models.metadata import Metadata
from src.models.ingestion_log import IngestionLog
from src.utils.hash_utils import calculate_content_hash
from src.ingestion.chunker import ContentChunker


def validate_settings():
    """Validate that the settings are properly configured."""
    print("Validating settings...")
    errors = settings.validate()
    if errors:
        print("❌ Configuration errors found:")
        for error in errors:
            print(f"  - {error}")
        return False
    else:
        print("✅ Settings validation passed")
        return True


def validate_models():
    """Validate that the data models work correctly."""
    print("\nValidating models...")

    # Test ContentSegment
    try:
        segment = ContentSegment(
            id="test_id",
            content="This is a test content with sufficient length to meet token requirements.",
            module="test_module",
            chapter="test_chapter",
            section="test_section",
            page_url="/test/page",
            token_count=10,
            hash="test_hash"
        )
        print("✅ ContentSegment model validation passed")
    except Exception as e:
        print(f"❌ ContentSegment model validation failed: {e}")
        return False

    # Test Metadata
    try:
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
        print("✅ Metadata model validation passed")
    except Exception as e:
        print(f"❌ Metadata model validation failed: {e}")
        return False

    # Test IngestionLog
    try:
        log = IngestionLog(
            id="test_log_id",
            run_id="test_run_id",
            source_file_path="/path/to/file",
            status="success",
            content_segments_created=5
        )
        print("✅ IngestionLog model validation passed")
    except Exception as e:
        print(f"❌ IngestionLog model validation failed: {e}")
        return False

    return True


def validate_utilities():
    """Validate that utility functions work correctly."""
    print("\nValidating utilities...")

    # Test hash calculation
    try:
        content = "Test content for hashing"
        hash_value = calculate_content_hash(content)
        assert len(hash_value) == 64  # SHA-256 produces 64-character hex string
        print("✅ Hash utility validation passed")
    except Exception as e:
        print(f"❌ Hash utility validation failed: {e}")
        return False

    # Test chunker
    try:
        chunker = ContentChunker(min_tokens=10, max_tokens=50)
        content = "This is a test sentence. " * 20  # Create content with multiple sentences
        structural_metadata = {
            'module': 'test_module',
            'chapter': 'test_chapter',
            'section': 'test_section',
            'page_url': '/test/page'
        }

        segments = chunker.chunk_content(content, structural_metadata)
        assert len(segments) > 0
        print("✅ Chunker utility validation passed")
    except Exception as e:
        print(f"❌ Chunker utility validation failed: {e}")
        return False

    return True


def validate_imports():
    """Validate that all modules can be imported."""
    print("\nValidating imports...")

    try:
        from src.config.settings import settings
        from src.models.content_segment import ContentSegment
        from src.models.embedding import Embedding
        from src.models.metadata import Metadata
        from src.models.ingestion_log import IngestionLog
        from src.utils.file_utils import find_files
        from src.utils.hash_utils import calculate_content_hash
        from src.ingestion.loader import ContentLoader
        from src.ingestion.chunker import ContentChunker
        from src.ingestion.embedder import Embedder
        from src.ingestion.storage import Storage
        from src.ingestion.orchestrator import PipelineOrchestrator
        from src.cli.main import main

        print("✅ Import validation passed")
        return True
    except ImportError as e:
        print(f"❌ Import validation failed: {e}")
        return False


def main():
    """Main validation function."""
    print("Starting validation of the content ingestion pipeline implementation...")
    print(f"Validation started at: {datetime.now().isoformat()}")

    all_passed = True

    # Run all validations
    all_passed &= validate_imports()
    all_passed &= validate_settings()
    all_passed &= validate_models()
    all_passed &= validate_utilities()

    print(f"\nValidation completed at: {datetime.now().isoformat()}")

    if all_passed:
        print("🎉 All validations passed! The implementation is ready.")
        return 0
    else:
        print("❌ Some validations failed. Please check the implementation.")
        return 1


if __name__ == "__main__":
    sys.exit(main())