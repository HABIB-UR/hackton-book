from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import Optional
import logging

logger = logging.getLogger(__name__)

def create_qdrant_client(url: str, api_key: str) -> QdrantClient:
    """
    Create and return a Qdrant client instance.

    Args:
        url: Qdrant service URL
        api_key: Qdrant API key for authentication

    Returns:
        QdrantClient instance
    """
    client = QdrantClient(
        url=url,
        api_key=api_key,
        # Timeout for requests
        timeout=10.0
    )
    return client

def setup_qdrant_collection(
    client: QdrantClient,
    collection_name: str,
    vector_size: int = 1536  # Default size for OpenAI embeddings
) -> bool:
    """
    Set up a Qdrant collection with the specified name and vector size.

    Args:
        client: QdrantClient instance
        collection_name: Name of the collection to create
        vector_size: Size of the embedding vectors (default: 1536 for OpenAI)

    Returns:
        True if collection was created or already exists, False otherwise
    """
    try:
        # Check if collection already exists
        collections = client.get_collections()
        collection_names = [collection.name for collection in collections.collections]

        if collection_name in collection_names:
            logger.info(f"Collection '{collection_name}' already exists")
            return True

        # Create the collection
        client.create_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(
                size=vector_size,
                distance=models.Distance.COSINE
            )
        )

        logger.info(f"Created collection '{collection_name}' with vector size {vector_size}")
        return True

    except Exception as e:
        logger.error(f"Failed to create collection '{collection_name}': {str(e)}")
        return False