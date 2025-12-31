"""Utility functions for content hashing using hashlib."""
import hashlib
from typing import Dict, Optional


def calculate_content_hash(content: str) -> str:
    """
    Calculate SHA-256 hash of content.

    Args:
        content: The content to hash

    Returns:
        SHA-256 hash of the content as a hexadecimal string
    """
    return hashlib.sha256(content.encode('utf-8')).hexdigest()


def calculate_file_hash(file_path: str) -> str:
    """
    Calculate SHA-256 hash of a file's content.

    Args:
        file_path: Path to the file

    Returns:
        SHA-256 hash of the file content as a hexadecimal string
    """
    with open(file_path, 'rb') as file:
        content = file.read()
        return hashlib.sha256(content).hexdigest()


def compare_content_hashes(hash1: str, hash2: str) -> bool:
    """
    Compare two content hashes for equality.

    Args:
        hash1: First hash to compare
        hash2: Second hash to compare

    Returns:
        True if hashes are equal, False otherwise
    """
    return hash1 == hash2


def is_content_changed(content: str, stored_hash: Optional[str]) -> bool:
    """
    Check if content has changed compared to a stored hash.

    Args:
        content: Current content to check
        stored_hash: Previously stored hash to compare against

    Returns:
        True if content has changed or no previous hash exists, False otherwise
    """
    if stored_hash is None:
        return True  # No previous hash, so content is considered changed

    current_hash = calculate_content_hash(content)
    return not compare_content_hashes(current_hash, stored_hash)


def is_file_changed(file_path: str, stored_hash: Optional[str]) -> bool:
    """
    Check if a file has changed compared to a stored hash.

    Args:
        file_path: Path to the file to check
        stored_hash: Previously stored hash to compare against

    Returns:
        True if file has changed or no previous hash exists, False otherwise
    """
    if stored_hash is None:
        return True  # No previous hash, so file is considered changed

    current_hash = calculate_file_hash(file_path)
    return not compare_content_hashes(current_hash, stored_hash)


def store_file_hash(file_path: str, hash_value: str) -> bool:
    """
    Store a file hash for future comparison (in a simple file-based approach).
    In a real implementation, this would use a database.

    Args:
        file_path: Path to the file
        hash_value: Hash value to store

    Returns:
        True if successfully stored, False otherwise
    """
    # This would typically store in a database, but for now we'll use a simple approach
    # In a real implementation, you'd use a proper database to track file hashes
    return True


def get_stored_hash(file_path: str) -> Optional[str]:
    """
    Get the stored hash for a file (in a simple file-based approach).
    In a real implementation, this would retrieve from a database.

    Args:
        file_path: Path to the file

    Returns:
        Stored hash value or None if not found
    """
    # This would typically retrieve from a database, but for now we'll return None
    # In a real implementation, you'd retrieve the hash from a database
    return None