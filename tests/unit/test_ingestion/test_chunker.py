"""Unit tests for the content chunker module."""
import pytest
from src.ingestion.chunker import ContentChunker


class TestContentChunker:
    """Test cases for the ContentChunker class."""

    def test_chunk_content_basic(self):
        """Test basic content chunking functionality."""
        chunker = ContentChunker(min_tokens=10, max_tokens=50, overlap_ratio=0.1)
        content = "This is a sample content for testing. " * 10  # Create content with multiple sentences
        structural_metadata = {
            'module': 'test_module',
            'chapter': 'test_chapter',
            'section': 'test_section',
            'page_url': '/test/page'
        }

        result = chunker.chunk_content(content, structural_metadata)

        # Check that we have at least one segment
        assert len(result) > 0

        # Check that each segment has the required properties
        for segment in result:
            assert segment.module == 'test_module'
            assert segment.chapter == 'test_chapter'
            assert segment.section == 'test_section'
            assert segment.page_url == '/test/page'
            assert len(segment.content) > 0
            assert segment.token_count >= 10  # At least min_tokens

    def test_token_counting(self):
        """Test that token counting works correctly."""
        chunker = ContentChunker()

        # Simple content to test tokenization
        content = "Hello world. This is a test."

        # Count tokens using the tokenizer
        token_count = len(chunker.tokenizer.encode(content))

        assert token_count > 0

    def test_min_max_token_constraints(self):
        """Test that chunks respect min and max token constraints."""
        chunker = ContentChunker(min_tokens=20, max_tokens=100)
        content = "This is a test sentence. " * 50  # Create longer content
        structural_metadata = {
            'module': 'test_module',
            'chapter': 'test_chapter',
            'section': 'test_section',
            'page_url': '/test/page'
        }

        result = chunker.chunk_content(content, structural_metadata)

        # Check token constraints for each segment
        for segment in result:
            assert segment.token_count >= 20  # At least min_tokens
            assert segment.token_count <= 100  # At most max_tokens

    def test_split_into_sentences(self):
        """Test sentence splitting functionality."""
        chunker = ContentChunker()
        text = "First sentence. Second sentence! Third sentence? All done."

        sentences = chunker._split_into_sentences(text)

        assert len(sentences) == 4  # Should split on ., !, ?
        assert sentences[0].endswith(' ')
        assert sentences[1].endswith(' ')
        assert sentences[2].endswith(' ')
        assert sentences[3] == 'All done.'  # Last sentence doesn't need space