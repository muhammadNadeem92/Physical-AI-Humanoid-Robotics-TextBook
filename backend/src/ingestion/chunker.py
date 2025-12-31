"""Text chunking logic with overlap."""
import tiktoken
from typing import List, Dict
from src.models.content_segment import ContentSegment
from src.utils.hash_utils import calculate_content_hash
import uuid


class ContentChunker:
    """Class to chunk text into 500-800 token segments with overlap."""

    def __init__(self, min_tokens: int = 500, max_tokens: int = 800, overlap_ratio: float = 0.2):
        """
        Initialize the content chunker.

        Args:
            min_tokens: Minimum number of tokens per chunk (default: 500)
            max_tokens: Maximum number of tokens per chunk (default: 800)
            overlap_ratio: Ratio of overlap between chunks (default: 0.2)
        """
        self.min_tokens = min_tokens
        self.max_tokens = max_tokens
        self.overlap_ratio = overlap_ratio
        # Use the cl100k_base tokenizer which is used by gpt-4, gpt-3.5-turbo, text-embedding-ada-002
        self.tokenizer = tiktoken.get_encoding("cl100k_base")

    def chunk_content(self, content: str, structural_metadata: Dict) -> List[ContentSegment]:
        """
        Chunk text content into segments with specified token limits and overlap.

        Args:
            content: The content to chunk
            structural_metadata: Metadata including module, chapter, section, page URL

        Returns:
            List of ContentSegment objects
        """
        # First, split content into sentences to maintain context
        sentences = self._split_into_sentences(content)
        chunks = []
        current_chunk = ""
        current_tokens = 0

        for sentence in sentences:
            sentence_tokens = len(self.tokenizer.encode(sentence))

            # Check if adding this sentence would exceed max tokens
            if current_tokens + sentence_tokens > self.max_tokens and current_chunk:
                # If current chunk is within acceptable range, add it
                if current_tokens >= self.min_tokens:
                    chunk_segment = self._create_content_segment(current_chunk, structural_metadata)
                    chunks.append(chunk_segment)
                    # Start new chunk with overlap
                    current_chunk, current_tokens = self._apply_overlap(current_chunk, sentence)
                else:
                    # If chunk is too small, try to add the sentence anyway
                    current_chunk += sentence
                    current_tokens += sentence_tokens
            else:
                current_chunk += sentence
                current_tokens += sentence_tokens

        # Add the final chunk if it meets minimum size requirements
        if current_chunk.strip() and len(self.tokenizer.encode(current_chunk)) >= self.min_tokens:
            chunk_segment = self._create_content_segment(current_chunk, structural_metadata)
            chunks.append(chunk_segment)

        return chunks

    def _split_into_sentences(self, text: str) -> List[str]:
        """
        Split text into sentences while preserving sentence boundaries.

        Args:
            text: Text to split into sentences

        Returns:
            List of sentences
        """
        import re
        # Split on sentence boundaries (., !, ?, etc.) while keeping the punctuation
        sentences = re.split(r'(?<=[.!?])\s+', text)
        # Add the space back to all but the last sentence
        for i in range(len(sentences) - 1):
            sentences[i] += ' '
        return sentences

    def _apply_overlap(self, current_chunk: str, new_sentence: str) -> tuple:
        """
        Apply overlap to the current chunk when starting a new chunk.

        Args:
            current_chunk: Current chunk content
            new_sentence: New sentence to add

        Returns:
            Tuple of (new_chunk_content, new_token_count)
        """
        # Calculate overlap size in tokens
        overlap_size = int(self.max_tokens * self.overlap_ratio)

        # Get the last part of the current chunk to use as overlap
        current_tokens = self.tokenizer.encode(current_chunk)
        if len(current_tokens) > overlap_size:
            overlap_tokens = current_tokens[-overlap_size:]
            overlap_text = self.tokenizer.decode(overlap_tokens)
        else:
            overlap_text = current_chunk

        # Create new chunk with overlap + new sentence
        new_chunk = overlap_text + new_sentence
        new_token_count = len(self.tokenizer.encode(new_chunk))

        return new_chunk, new_token_count

    def _create_content_segment(self, content: str, structural_metadata: Dict) -> ContentSegment:
        """
        Create a ContentSegment object from content and metadata.

        Args:
            content: Content text
            structural_metadata: Metadata dictionary

        Returns:
            ContentSegment object
        """
        token_count = len(self.tokenizer.encode(content))
        content_hash = calculate_content_hash(content)

        return ContentSegment(
            id=str(uuid.uuid4()),
            content=content,
            module=structural_metadata.get('module', 'unknown'),
            chapter=structural_metadata.get('chapter', 'unknown'),
            section=structural_metadata.get('section', 'unknown'),
            page_url=structural_metadata.get('page_url', ''),
            token_count=token_count,
            hash=content_hash
        )


def chunk_text_content(content: str, structural_metadata: Dict, min_tokens: int = 500, max_tokens: int = 800, overlap_ratio: float = 0.2) -> List[ContentSegment]:
    """
    Function to chunk text content into 500-800 token segments with overlap.

    Args:
        content: The content to chunk
        structural_metadata: Metadata including module, chapter, section, page URL
        min_tokens: Minimum number of tokens per chunk (default: 500)
        max_tokens: Maximum number of tokens per chunk (default: 800)
        overlap_ratio: Ratio of overlap between chunks (default: 0.2)

    Returns:
        List of ContentSegment objects
    """
    chunker = ContentChunker(min_tokens, max_tokens, overlap_ratio)
    return chunker.chunk_content(content, structural_metadata)