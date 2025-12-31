"""Unit tests for the content loader module."""
import pytest
from src.ingestion.loader import ContentLoader


class TestContentLoader:
    """Test cases for the ContentLoader class."""

    def test_extract_structural_metadata_basic_path(self):
        """Test extracting structural metadata from a basic path."""
        loader = ContentLoader()
        relative_path = "module01-introduction/chapter01-basics/01-getting-started.md"

        result = loader._extract_structural_metadata(relative_path)

        assert result['module'] == 'introduction'
        assert result['chapter'] == 'basics'
        assert result['section'] == 'getting-started'
        assert result['page_url'] == '/module01-introduction/chapter01-basics/01-getting-started'

    def test_extract_structural_metadata_nested_path(self):
        """Test extracting structural metadata from a nested path."""
        loader = ContentLoader()
        relative_path = "module02-advanced/section01-embedding/01-advanced-embedding.md"

        result = loader._extract_structural_metadata(relative_path)

        assert result['module'] == 'advanced'
        assert result['chapter'] == 'embedding'
        assert result['section'] == 'advanced-embedding'

    def test_extract_structural_metadata_simple_path(self):
        """Test extracting structural metadata from a simple path."""
        loader = ContentLoader()
        relative_path = "simple-file.md"

        result = loader._extract_structural_metadata(relative_path)

        assert result['module'] == 'unknown'
        assert result['chapter'] == 'unknown'
        assert result['section'] == 'simple-file'
        assert result['page_url'] == '/simple-file'

    def test_validate_markdown_syntax_valid(self):
        """Test validating valid markdown syntax."""
        loader = ContentLoader()
        valid_markdown = "# Test\n\nThis is a test.\n\n- Item 1\n- Item 2"

        result = loader.validate_markdown_syntax(valid_markdown)

        assert result is True

    def test_validate_markdown_syntax_invalid(self):
        """Test validating invalid markdown syntax."""
        loader = ContentLoader()
        invalid_markdown = "<unclosed_tag>This is invalid markdown"

        result = loader.validate_markdown_syntax(invalid_markdown)

        # The current implementation should return False for invalid syntax
        assert result is False