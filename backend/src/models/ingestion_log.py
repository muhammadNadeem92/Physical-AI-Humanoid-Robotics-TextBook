"""Ingestion log data model."""
from dataclasses import dataclass
from typing import Optional
from datetime import datetime


@dataclass
class IngestionLog:
    """
    Record of ingestion activities including timestamps, status, and error information.
    """
    id: str
    run_id: str
    source_file_path: str
    status: str  # 'success', 'error', 'skipped'
    error_message: Optional[str] = None
    content_segments_created: int = 0
    start_time: Optional[datetime] = None
    end_time: Optional[datetime] = None
    duration_ms: Optional[int] = None
    created_at: datetime = datetime.now()

    def __post_init__(self):
        """Validate the ingestion log after initialization."""
        if self.status not in ['success', 'error', 'skipped']:
            raise ValueError("Status must be one of: 'success', 'error', 'skipped'")

        if self.duration_ms is not None and self.duration_ms < 0:
            raise ValueError("Duration must be non-negative")

        if not self.run_id:
            raise ValueError("Run ID must not be empty")

        if not self.source_file_path:
            raise ValueError("Source file path must not be empty")