from typing import List, Dict, Any
import logging
from qdrant_client.http import models
from qdrant_helper import create_qdrant_client, setup_qdrant_collection
from config import Config
from models import Embedding
from errors import QdrantStorageError, handle_error

logger = logging.getLogger(__name__)

def store_embeddings_in_qdrant(
    embeddings: List[Embedding],
    collection_name: str = "documentation_embeddings",
    vector_size: int = 1536  # Default size for OpenAI embeddings
) -> bool:
    """
    Store embeddings in Qdrant vector database with metadata.

    Args:
        embeddings: List of Embedding objects to store
        collection_name: Name of the Qdrant collection to store embeddings in
        vector_size: Size of the embedding vectors

    Returns:
        True if storage was successful, False otherwise
    """
    try:
        # Create Qdrant client
        client = create_qdrant_client(Config.QDRANT_URL, Config.QDRANT_API_KEY)

        # Ensure collection exists
        if not setup_qdrant_collection(client, collection_name, vector_size):
            raise QdrantStorageError(f"Failed to set up collection '{collection_name}'")

        # Prepare points for insertion
        points = []
        for embedding in embeddings:
            point = models.PointStruct(
                id=embedding.id,
                vector=embedding.vector,
                payload={
                    "chunk_id": embedding.chunk_id,
                    "source_url": embedding.metadata.get("source_url", ""),
                    "source_section": embedding.metadata.get("source_section", ""),
                    "created_at": embedding.created_at.isoformat() if embedding.created_at else None,
                    **{k: v for k, v in embedding.metadata.items()
                       if k not in ["source_url", "source_section", "created_at"]}
                }
            )
            points.append(point)

        # Upload points to Qdrant
        client.upload_points(
            collection_name=collection_name,
            points=points,
            wait=True  # Wait for operation to complete
        )

        logger.info(f"Successfully stored {len(embeddings)} embeddings in collection '{collection_name}'")
        return True

    except Exception as e:
        error_msg = handle_error(e, f"storing embeddings in Qdrant collection {collection_name}")
        raise QdrantStorageError(error_msg)

def batch_store_embeddings_in_qdrant(
    embeddings: List[Embedding],
    collection_name: str = "documentation_embeddings",
    batch_size: int = 10,
    vector_size: int = 1536
) -> bool:
    """
    Store embeddings in Qdrant in batches for efficient processing.

    Args:
        embeddings: List of Embedding objects to store
        collection_name: Name of the Qdrant collection to store embeddings in
        batch_size: Number of embeddings to process in each batch
        vector_size: Size of the embedding vectors

    Returns:
        True if storage was successful, False otherwise
    """
    try:
        # Create Qdrant client
        client = create_qdrant_client(Config.QDRANT_URL, Config.QDRANT_API_KEY)

        # Ensure collection exists
        if not setup_qdrant_collection(client, collection_name, vector_size):
            raise QdrantStorageError(f"Failed to set up collection '{collection_name}'")

        # Process embeddings in batches
        total_embeddings = len(embeddings)
        for i in range(0, total_embeddings, batch_size):
            batch = embeddings[i:i + batch_size]

            # Prepare points for this batch
            points = []
            for embedding in batch:
                point = models.PointStruct(
                    id=embedding.id,
                    vector=embedding.vector,
                    payload={
                        "chunk_id": embedding.chunk_id,
                        "source_url": embedding.metadata.get("source_url", ""),
                        "source_section": embedding.metadata.get("source_section", ""),
                        "created_at": embedding.created_at.isoformat() if embedding.created_at else None,
                        **{k: v for k, v in embedding.metadata.items()
                           if k not in ["source_url", "source_section", "created_at"]}
                    }
                )
                points.append(point)

            # Upload batch to Qdrant
            client.upload_points(
                collection_name=collection_name,
                points=points,
                wait=True
            )

            logger.info(f"Stored batch {i//batch_size + 1}/{(total_embeddings-1)//batch_size + 1} ({len(batch)} embeddings)")

        logger.info(f"Successfully stored {total_embeddings} embeddings in collection '{collection_name}' using batches of {batch_size}")
        return True

    except Exception as e:
        error_msg = handle_error(e, f"batch storing embeddings in Qdrant collection {collection_name}")
        raise QdrantStorageError(error_msg)

def validate_embedding_storage(embeddings: List[Embedding], collection_name: str = "documentation_embeddings") -> bool:
    """
    Validate that embeddings were properly stored in Qdrant.

    Args:
        embeddings: List of embeddings that should be stored
        collection_name: Name of the Qdrant collection

    Returns:
        True if validation passes, False otherwise
    """
    try:
        client = create_qdrant_client(Config.QDRANT_URL, Config.QDRANT_API_KEY)

        # Get collection info
        collection_info = client.get_collection(collection_name)

        # Check if the number of stored vectors matches expected count
        if collection_info.points_count < len(embeddings):
            logger.warning(f"Expected {len(embeddings)} embeddings but found {collection_info.points_count} in collection")
            return False

        logger.info(f"Embedding storage validation passed: {collection_info.points_count} points in collection")
        return True

    except Exception as e:
        error_msg = handle_error(e, f"validating embedding storage in {collection_name}")
        logger.error(error_msg)
        return False