"""Content segment data model."""
from dataclasses import dataclass
from typing import Optional
from datetime import datetime


@dataclass
class ContentSegment:
    """
    A chunk of text from the textbook content (500-800 tokens) with associated metadata.
    """
    id: str
    content: str
    module: str
    chapter: str
    section: str
    page_url: str
    token_count: int
    hash: str
    created_at: datetime = datetime.now()
    updated_at: datetime = datetime.now()

    def __post_init__(self):
        """Validate the content segment after initialization."""
        if self.token_count < 500 or self.token_count > 800:
            raise ValueError(f"Token count must be between 500-800, got {self.token_count}")

        if not self.module or not self.chapter or not self.section:
            raise ValueError("Module, chapter, and section must not be empty")

        if not self.page_url:
            raise ValueError("Page URL must not be empty")