import cohere
from typing import List, Optional
import logging
from .config import Config
from .models import Embedding
from .errors import EmbeddingGenerationError, handle_error

logger = logging.getLogger(__name__)

def setup_cohere_client():
    """Set up the Cohere client with the API key from config."""
    # Note: We'll need to get the Cohere API key from the root .env
    import os
    from dotenv import load_dotenv
    load_dotenv()  # Load root .env file

    cohere_api_key = os.getenv('COHERE_API_KEY', Config.OPENAI_API_KEY)  # Fallback to OPENAI_API_KEY
    cohere_client = cohere.Client(cohere_api_key)
    return cohere_client

def generate_cohere_embeddings(texts: List[str], model: str = "embed-english-v3.0") -> List[List[float]]:
    """
    Generate embeddings for a list of texts using Cohere API.

    Args:
        texts: List of texts to generate embeddings for
        model: The embedding model to use (default: embed-english-v3.0)

    Returns:
        List of embedding vectors (each vector is a list of floats)
    """
    try:
        # Set up the Cohere client
        client = setup_cohere_client()

        # Generate embeddings using Cohere
        response = client.embed(
            texts=texts,
            model=model,
            input_type="search_document"  # or "search_query" for queries
        )

        # Extract embeddings from response
        embeddings = response.embeddings

        logger.info(f"Generated embeddings for {len(texts)} texts using Cohere model: {model}")
        return embeddings

    except Exception as e:
        error_msg = handle_error(e, f"generating embeddings with Cohere model {model}")
        raise EmbeddingGenerationError(error_msg)

def generate_single_cohere_embedding(text: str, model: str = "embed-english-v3.0") -> List[float]:
    """
    Generate a single embedding for a text using Cohere.

    Args:
        text: The text to generate embedding for
        model: The embedding model to use

    Returns:
        Embedding vector as a list of floats
    """
    try:
        embeddings = generate_cohere_embeddings([text], model)
        return embeddings[0]
    except Exception as e:
        error_msg = handle_error(e, f"generating single embedding for text of length {len(text)}")
        raise EmbeddingGenerationError(error_msg)