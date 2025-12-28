"""Unit tests for the hash utilities module."""
import pytest
from src.utils.hash_utils import (
    calculate_content_hash,
    calculate_file_hash,
    compare_content_hashes,
    is_content_changed,
    is_file_changed
)
import tempfile
import os


class TestHashUtils:
    """Test cases for the hash utilities."""

    def test_calculate_content_hash(self):
        """Test calculating hash for content."""
        content = "This is a test content for hashing."
        hash1 = calculate_content_hash(content)
        hash2 = calculate_content_hash(content)

        # Same content should produce same hash
        assert hash1 == hash2
        assert len(hash1) == 64  # SHA-256 produces 64-character hex string

    def test_calculate_different_content_hashes(self):
        """Test that different content produces different hashes."""
        content1 = "Content one"
        content2 = "Content two"

        hash1 = calculate_content_hash(content1)
        hash2 = calculate_content_hash(content2)

        # Different content should produce different hashes
        assert hash1 != hash2

    def test_calculate_file_hash(self):
        """Test calculating hash for a file."""
        content = "This is a test file content."

        # Create a temporary file
        with tempfile.NamedTemporaryFile(mode='w', delete=False) as temp_file:
            temp_file.write(content)
            temp_file_path = temp_file.name

        try:
            file_hash = calculate_file_hash(temp_file_path)
            expected_hash = calculate_content_hash(content)

            # File hash should match content hash
            assert file_hash == expected_hash
        finally:
            # Clean up the temporary file
            os.unlink(temp_file_path)

    def test_compare_content_hashes_same(self):
        """Test comparing identical hashes."""
        content = "Test content"
        hash1 = calculate_content_hash(content)
        hash2 = calculate_content_hash(content)

        result = compare_content_hashes(hash1, hash2)

        assert result is True

    def test_compare_content_hashes_different(self):
        """Test comparing different hashes."""
        hash1 = calculate_content_hash("Content one")
        hash2 = calculate_content_hash("Content two")

        result = compare_content_hashes(hash1, hash2)

        assert result is False

    def test_is_content_changed_no_stored_hash(self):
        """Test content change detection with no stored hash."""
        content = "Test content"
        result = is_content_changed(content, None)

        # Should return True when no stored hash exists
        assert result is True

    def test_is_content_changed_unchanged(self):
        """Test content change detection for unchanged content."""
        content = "Test content"
        stored_hash = calculate_content_hash(content)

        result = is_content_changed(content, stored_hash)

        # Should return False when content hasn't changed
        assert result is False

    def test_is_content_changed_changed(self):
        """Test content change detection for changed content."""
        original_content = "Original content"
        new_content = "New content"
        stored_hash = calculate_content_hash(original_content)

        result = is_content_changed(new_content, stored_hash)

        # Should return True when content has changed
        assert result is True

    def test_is_file_changed_no_stored_hash(self):
        """Test file change detection with no stored hash."""
        content = "Test content"

        # Create a temporary file
        with tempfile.NamedTemporaryFile(mode='w', delete=False) as temp_file:
            temp_file.write(content)
            temp_file_path = temp_file.name

        try:
            result = is_file_changed(temp_file_path, None)

            # Should return True when no stored hash exists
            assert result is True
        finally:
            # Clean up the temporary file
            os.unlink(temp_file_path)

    def test_is_file_changed_unchanged(self):
        """Test file change detection for unchanged file."""
        content = "Test content"

        # Create a temporary file
        with tempfile.NamedTemporaryFile(mode='w', delete=False) as temp_file:
            temp_file.write(content)
            temp_file_path = temp_file.name

        try:
            stored_hash = calculate_file_hash(temp_file_path)
            result = is_file_changed(temp_file_path, stored_hash)

            # Should return False when file hasn't changed
            assert result is False
        finally:
            # Clean up the temporary file
            os.unlink(temp_file_path)

    def test_is_file_changed_changed(self):
        """Test file change detection for changed file."""
        original_content = "Original content"
        new_content = "New content"

        # Create a temporary file with original content
        with tempfile.NamedTemporaryFile(mode='w', delete=False) as temp_file:
            temp_file.write(original_content)
            temp_file_path = temp_file.name

        try:
            stored_hash = calculate_file_hash(temp_file_path)

            # Update the file with new content
            with open(temp_file_path, 'w') as f:
                f.write(new_content)

            result = is_file_changed(temp_file_path, stored_hash)

            # Should return True when file has changed
            assert result is True
        finally:
            # Clean up the temporary file
            os.unlink(temp_file_path)