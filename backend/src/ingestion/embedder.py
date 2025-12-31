"""Cohere embedding generation."""
import cohere
from typing import List
from src.models.embedding import Embedding
from src.config.settings import settings
import uuid
import time


class Embedder:
    """Class to generate embeddings using Cohere API."""

    def __init__(self, model_name: str = settings.COHERE_MODEL_NAME):
        """
        Initialize the embedder.

        Args:
            model_name: Name of the Cohere embedding model (default: from settings)
        """
        self.model_name = model_name
        self.client = cohere.Client(settings.COHERE_API_KEY)

    def generate_embeddings(self, texts: List[str]) -> List[Embedding]:
        """
        Generate embeddings for a list of texts using Cohere API.

        Args:
            texts: List of text chunks to embed

        Returns:
            List of Embedding objects with vectors and metadata

        Raises:
            Exception: If Cohere API call fails
        """
        if not texts:
            return []

        # Cohere has limits on batch size, so we'll process in chunks if needed
        batch_size = 96  # Conservative batch size (Cohere's limit is 96)
        all_embeddings = []

        for i in range(0, len(texts), batch_size):
            batch = texts[i:i + batch_size]
            try:
                response = self.client.embed(
                    texts=batch,
                    model=self.model_name,
                    input_type="search_document"  # Appropriate for document search
                )

                # Create Embedding objects from the response
                for idx, embedding_vector in enumerate(response.embeddings):
                    content_segment_id = f"batch_{i}_text_{idx}"
                    embedding_obj = Embedding(
                        id=str(uuid.uuid4()),
                        content_segment_id=content_segment_id,
                        vector=embedding_vector,
                        model_name=self.model_name
                    )
                    all_embeddings.append(embedding_obj)

            except cohere.CohereAPIError as e:
                print(f"Cohere API error: {e}")
                raise e
            except Exception as e:
                print(f"Error generating embeddings: {e}")
                raise e

        return all_embeddings

    def generate_embedding_for_text(self, text: str, content_segment_id: str) -> Embedding:
        """
        Generate a single embedding for a text.

        Args:
            text: Text to embed
            content_segment_id: ID of the content segment

        Returns:
            Embedding object
        """
        try:
            response = self.client.embed(
                texts=[text],
                model=self.model_name,
                input_type="search_document"
            )

            return Embedding(
                id=str(uuid.uuid4()),
                content_segment_id=content_segment_id,
                vector=response.embeddings[0],
                model_name=self.model_name
            )
        except cohere.CohereAPIError as e:
            print(f"Cohere API error for single text: {e}")
            raise e
        except Exception as e:
            print(f"Error generating embedding for single text: {e}")
            raise e


def generate_embeddings_for_texts(texts: List[str]) -> List[Embedding]:
    """
    Function to generate embeddings using Cohere API.

    Args:
        texts: List of text chunks to embed

    Returns:
        List of Embedding objects with vectors and metadata
    """
    embedder = Embedder()
    return embedder.generate_embeddings(texts)