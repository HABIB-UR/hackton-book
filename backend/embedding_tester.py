from typing import List, Optional
import logging
from qdrant_client import models
from qdrant_helper import create_qdrant_client
from config import Config
from embedding_generator import generate_single_embedding
from errors import handle_error

logger = logging.getLogger(__name__)

def test_embedding_similarity(
    query_text: str,
    collection_name: str = "documentation_embeddings",
    top_k: int = 5
) -> List[dict]:
    """
    Test embedding similarity by performing a search with a query text.

    Args:
        query_text: The text to use for similarity search
        collection_name: Name of the Qdrant collection to search in
        top_k: Number of top results to return

    Returns:
        List of results with content, source URL, section, and similarity score
    """
    try:
        # Generate embedding for the query text
        query_embedding = generate_single_embedding(query_text)

        # Create Qdrant client
        client = create_qdrant_client(Config.QDRANT_URL, Config.QDRANT_API_KEY)

        # Perform search
        search_results = client.search(
            collection_name=collection_name,
            query_vector=query_embedding,
            limit=top_k,
            with_payload=True
        )

        # Format results
        results = []
        for result in search_results:
            payload = result.payload
            results.append({
                "content": payload.get("content", "")[:200] + "..." if len(payload.get("content", "")) > 200 else payload.get("content", ""),
                "source_url": payload.get("source_url", ""),
                "source_section": payload.get("source_section", ""),
                "similarity_score": result.score
            })

        logger.info(f"Found {len(results)} similar content chunks for query: '{query_text[:50]}...'")
        return results

    except Exception as e:
        error_msg = handle_error(e, f"performing similarity test in collection {collection_name}")
        logger.error(error_msg)
        return []

def verify_embeddings_stored(collection_name: str = "documentation_embeddings") -> bool:
    """
    Verify that embeddings have been stored in the collection.

    Args:
        collection_name: Name of the Qdrant collection to check

    Returns:
        True if embeddings exist in the collection, False otherwise
    """
    try:
        client = create_qdrant_client(Config.QDRANT_URL, Config.QDRANT_API_KEY)

        # Get collection info
        collection_info = client.get_collection(collection_name)

        # Check if there are any points in the collection
        has_embeddings = collection_info.points_count > 0

        if has_embeddings:
            logger.info(f"Verified {collection_info.points_count} embeddings stored in collection '{collection_name}'")
        else:
            logger.warning(f"No embeddings found in collection '{collection_name}'")

        return has_embeddings

    except Exception as e:
        error_msg = handle_error(e, f"verifying embeddings in collection {collection_name}")
        logger.error(error_msg)
        return False

def run_embedding_verification_test(query_text: str = "What is this documentation about?", top_k: int = 3) -> bool:
    """
    Run a comprehensive verification test to check if embeddings are properly stored and searchable.

    Args:
        query_text: Test query to verify search functionality
        top_k: Number of results to return

    Returns:
        True if verification passes, False otherwise
    """
    try:
        logger.info("Running embedding verification test...")

        # Check if embeddings exist
        if not verify_embeddings_stored():
            logger.error("No embeddings found in the collection")
            return False

        # Try to perform a search
        results = test_embedding_similarity(query_text, top_k=top_k)

        if not results:
            logger.error("No results returned from similarity search")
            return False

        logger.info(f"Verification test passed: Found {len(results)} relevant results")
        return True

    except Exception as e:
        error_msg = handle_error(e, "running embedding verification test")
        logger.error(error_msg)
        return False