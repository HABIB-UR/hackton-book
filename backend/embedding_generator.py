import openai
from typing import List, Optional
import logging
from config import Config
from models import Embedding
from errors import EmbeddingGenerationError, handle_error

logger = logging.getLogger(__name__)

def setup_openai_client():
    """Set up the OpenAI client with the API key from config."""
    openai.api_key = Config.OPENAI_API_KEY

def generate_embeddings(texts: List[str], model: str = "text-embedding-ada-002") -> List[List[float]]:
    """
    Generate embeddings for a list of texts using OpenAI API.

    Args:
        texts: List of texts to generate embeddings for
        model: The embedding model to use (default: text-embedding-ada-002)

    Returns:
        List of embedding vectors (each vector is a list of floats)
    """
    try:
        # Set up the OpenAI client
        setup_openai_client()

        # Generate embeddings
        response = openai.embeddings.create(
            input=texts,
            model=model
        )

        # Extract embeddings from response
        embeddings = []
        for item in response.data:
            embeddings.append(item.embedding)

        logger.info(f"Generated embeddings for {len(texts)} texts using model: {model}")
        return embeddings

    except Exception as e:
        error_msg = handle_error(e, f"generating embeddings with model {model}")
        raise EmbeddingGenerationError(error_msg)

def generate_single_embedding(text: str, model: str = "text-embedding-ada-002") -> List[float]:
    """
    Generate a single embedding for a text.

    Args:
        text: The text to generate embedding for
        model: The embedding model to use

    Returns:
        Embedding vector as a list of floats
    """
    try:
        embeddings = generate_embeddings([text], model)
        return embeddings[0]
    except Exception as e:
        error_msg = handle_error(e, f"generating single embedding for text of length {len(text)}")
        raise EmbeddingGenerationError(error_msg)

def validate_embedding_dimensions(embeddings: List[List[float]], expected_size: Optional[int] = None) -> bool:
    """
    Validate that all embeddings have consistent dimensions.

    Args:
        embeddings: List of embedding vectors
        expected_size: Expected size of each embedding (optional)

    Returns:
        True if all embeddings have consistent dimensions, False otherwise
    """
    if not embeddings:
        return False

    first_size = len(embeddings[0])

    if expected_size and first_size != expected_size:
        logger.error(f"Embedding size {first_size} doesn't match expected size {expected_size}")
        return False

    for i, embedding in enumerate(embeddings):
        if len(embedding) != first_size:
            logger.error(f"Embedding at index {i} has size {len(embedding)}, expected {first_size}")
            return False

    logger.info(f"All embeddings have consistent size of {first_size}")
    return True

def create_embedding_instance(
    chunk_id: str,
    vector: List[float],
    source_url: str,
    source_section: str,
    metadata: Optional[dict] = None
) -> Embedding:
    """
    Create an Embedding instance with proper metadata.

    Args:
        chunk_id: ID of the source chunk
        vector: The embedding vector
        source_url: Source URL for the content
        source_section: Source section for the content
        metadata: Additional metadata (optional)

    Returns:
        Embedding instance
    """
    try:
        if metadata is None:
            metadata = {}

        # Ensure required metadata fields are present
        metadata['source_url'] = source_url
        metadata['source_section'] = source_section

        embedding = Embedding(
            id="",
            vector=vector,
            chunk_id=chunk_id,
            metadata=metadata
        )

        return embedding
    except Exception as e:
        error_msg = handle_error(e, f"creating embedding instance for chunk {chunk_id}")
        raise ValueError(error_msg)