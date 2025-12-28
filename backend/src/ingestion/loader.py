"""Load and parse Docusaurus markdown content from /frontend/docs."""
import os
import re
from pathlib import Path
from typing import Dict, List, Optional
from markdown import markdown
import frontmatter  # type: ignore

from src.utils.file_utils import find_files, read_file, get_relative_path
from src.models.content_segment import ContentSegment


class ContentLoader:
    """Class to load and parse Docusaurus markdown content."""

    def __init__(self, source_path: str = "/frontend/docs"):
        """
        Initialize the content loader.

        Args:
            source_path: Path to the Docusaurus docs directory
        """
        self.source_path = source_path

    def load_content(self, file_patterns: Optional[List[str]] = None) -> List[Dict]:
        """
        Load Docusaurus markdown content from the source directory.

        Args:
            file_patterns: Patterns to match markdown files (default: ["**/*.md", "**/*.mdx"])

        Returns:
            List of content files with path, content, and metadata
        """
        if file_patterns is None:
            file_patterns = ["**/*.md", "**/*.mdx"]

        content_files = []
        file_paths = find_files(self.source_path, file_patterns)

        for file_path in file_paths:
            try:
                content = read_file(file_path)
                relative_path = get_relative_path(file_path, self.source_path)

                # Parse frontmatter and content
                post = frontmatter.loads(content)
                metadata = post.metadata
                content_text = post.content

                # Extract structural metadata from the file path
                structural_metadata = self._extract_structural_metadata(relative_path)

                content_file = {
                    'path': file_path,
                    'relative_path': relative_path,
                    'content': content_text,
                    'metadata': metadata,
                    'structural_metadata': structural_metadata
                }

                content_files.append(content_file)
            except Exception as e:
                print(f"Error processing file {file_path}: {str(e)}")
                continue

        return content_files

    def _extract_structural_metadata(self, relative_path: str) -> Dict[str, str]:
        """
        Extract structural metadata (module, chapter, section) from the file path.

        Args:
            relative_path: Relative path of the file from source directory

        Returns:
            Dictionary with module, chapter, section, and page URL
        """
        path_parts = Path(relative_path).parts

        # Default values
        module = "unknown"
        chapter = "unknown"
        section = "unknown"
        page_url = f"/{relative_path.replace('.md', '').replace('.mdx', '')}"

        # Extract based on path structure (e.g., module/chapter/section.md)
        if len(path_parts) >= 1:
            # The last part without extension is the section/page name
            section = Path(path_parts[-1]).stem

        if len(path_parts) >= 2:
            # The second-to-last part is the chapter
            chapter = path_parts[-2]

        if len(path_parts) >= 3:
            # The third-to-last part is the module
            module = path_parts[-3]

        # Special handling for common Docusaurus patterns
        # If the path is like "module01-introduction/chapter01-basics/01-section.md"
        if len(path_parts) >= 1:
            # Remove numeric prefixes from names
            section_clean = re.sub(r'^\d+-', '', Path(path_parts[-1]).stem)
            section = section_clean

        if len(path_parts) >= 2:
            chapter_clean = re.sub(r'^\d+-', '', path_parts[-2])
            chapter = chapter_clean

        if len(path_parts) >= 3:
            module_clean = re.sub(r'^\d+-', '', path_parts[-3])
            module = module_clean

        return {
            'module': module,
            'chapter': chapter,
            'section': section,
            'page_url': page_url
        }

    def validate_markdown_syntax(self, content: str) -> bool:
        """
        Validate markdown syntax to ensure proper parsing.

        Args:
            content: Markdown content to validate

        Returns:
            True if syntax is valid, False otherwise
        """
        try:
            # Try to parse the markdown
            markdown(content)
            return True
        except Exception:
            return False


def load_docusaurus_content(source_path: str = "/frontend/docs", file_patterns: Optional[List[str]] = None) -> List[Dict]:
    """
    Function to load and parse Docusaurus markdown content from /frontend/docs.

    Args:
        source_path: Path to the Docusaurus docs directory
        file_patterns: Patterns to match markdown files (default: ["**/*.md", "**/*.mdx"])

    Returns:
        List of content files with path, content, and metadata
    """
    loader = ContentLoader(source_path)
    return loader.load_content(file_patterns)