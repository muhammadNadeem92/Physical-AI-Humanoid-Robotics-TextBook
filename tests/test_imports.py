"""Test that all modules can be imported correctly."""
import pytest


def test_config_imports():
    """Test that config modules can be imported."""
    from src.config.settings import settings
    assert settings is not None


def test_models_imports():
    """Test that model modules can be imported."""
    from src.models.content_segment import ContentSegment
    from src.models.embedding import Embedding
    from src.models.metadata import Metadata
    from src.models.ingestion_log import IngestionLog

    assert ContentSegment is not None
    assert Embedding is not None
    assert Metadata is not None
    assert IngestionLog is not None


def test_utils_imports():
    """Test that utility modules can be imported."""
    from src.utils.file_utils import find_files, read_file
    from src.utils.hash_utils import calculate_content_hash, calculate_file_hash

    assert find_files is not None
    assert read_file is not None
    assert calculate_content_hash is not None
    assert calculate_file_hash is not None


def test_ingestion_imports():
    """Test that ingestion modules can be imported."""
    from src.ingestion.loader import ContentLoader, load_docusaurus_content
    from src.ingestion.chunker import ContentChunker, chunk_text_content
    from src.ingestion.embedder import Embedder, generate_embeddings_for_texts
    from src.ingestion.storage import Storage
    from src.ingestion.orchestrator import PipelineOrchestrator, run_ingestion_pipeline

    assert ContentLoader is not None
    assert load_docusaurus_content is not None
    assert ContentChunker is not None
    assert chunk_text_content is not None
    assert Embedder is not None
    assert generate_embeddings_for_texts is not None
    assert Storage is not None
    assert PipelineOrchestrator is not None
    assert run_ingestion_pipeline is not None


def test_cli_imports():
    """Test that CLI modules can be imported."""
    from src.cli.main import main

    assert main is not None