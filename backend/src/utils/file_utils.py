"""Utility functions for file operations and directory traversal."""
import os
import glob
from pathlib import Path
from typing import List, Optional


def find_files(directory: str, patterns: List[str]) -> List[str]:
    """
    Find files in a directory matching the given patterns.

    Args:
        directory: The directory to search in
        patterns: List of glob patterns to match

    Returns:
        List of file paths matching the patterns
    """
    found_files = []
    directory_path = Path(directory)

    for pattern in patterns:
        # Use glob to find files matching the pattern
        matches = glob.glob(str(directory_path / pattern), recursive=True)
        for match in matches:
            if os.path.isfile(match):
                found_files.append(match)

    return found_files


def read_file(file_path: str) -> str:
    """
    Read the content of a file.

    Args:
        file_path: Path to the file to read

    Returns:
        Content of the file as a string
    """
    with open(file_path, 'r', encoding='utf-8') as file:
        return file.read()


def get_file_info(file_path: str) -> dict:
    """
    Get information about a file.

    Args:
        file_path: Path to the file

    Returns:
        Dictionary with file information (path, size, modification time)
    """
    stat = os.stat(file_path)
    return {
        'path': file_path,
        'size': stat.st_size,
        'modified': stat.st_mtime
    }


def ensure_directory_exists(directory: str) -> bool:
    """
    Ensure that a directory exists, creating it if necessary.

    Args:
        directory: Path to the directory

    Returns:
        True if directory exists or was created, False otherwise
    """
    try:
        Path(directory).mkdir(parents=True, exist_ok=True)
        return True
    except Exception:
        return False


def get_relative_path(file_path: str, base_path: str) -> str:
    """
    Get the relative path of a file with respect to a base path.

    Args:
        file_path: Path to the file
        base_path: Base path to calculate relative path from

    Returns:
        Relative path as a string
    """
    return os.path.relpath(file_path, base_path)