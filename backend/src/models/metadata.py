"""Metadata data model."""
from dataclasses import dataclass
from typing import Optional
from datetime import datetime


@dataclass
class Metadata:
    """
    Structural information about the content including module, chapter, section, and page URL.
    """
    id: str
    content_segment_id: str
    module: str
    chapter: str
    section: str
    page_url: str
    source_file_path: str
    source_file_hash: str
    created_at: datetime = datetime.now()
    updated_at: datetime = datetime.now()

    def __post_init__(self):
        """Validate the metadata after initialization."""
        if not self.module or not self.chapter or not self.section:
            raise ValueError("Module, chapter, and section must not be empty")

        if not self.page_url:
            raise ValueError("Page URL must not be empty")

        if not self.source_file_path:
            raise ValueError("Source file path must not be empty")