import time
import logging
from typing import Callable, Any, Type, Tuple
from functools import wraps

logger = logging.getLogger(__name__)

def retry_on_failure(max_attempts: int = 3, delay: float = 1.0, backoff: float = 2.0, exceptions: Tuple[Type[Exception], ...] = (Exception,)):
    """
    Decorator to retry a function on failure.

    Args:
        max_attempts: Maximum number of attempts (default: 3)
        delay: Initial delay between attempts in seconds (default: 1.0)
        backoff: Multiplier for delay after each attempt (default: 2.0)
        exceptions: Tuple of exception types to catch and retry (default: all exceptions)
    """
    def decorator(func: Callable) -> Callable:
        @wraps(func)
        def wrapper(*args, **kwargs) -> Any:
            current_delay = delay

            for attempt in range(max_attempts):
                try:
                    return func(*args, **kwargs)
                except exceptions as e:
                    if attempt == max_attempts - 1:  # Last attempt
                        logger.error(f"Function {func.__name__} failed after {max_attempts} attempts: {str(e)}")
                        raise e
                    else:
                        logger.warning(f"Attempt {attempt + 1} failed for {func.__name__}: {str(e)}. Retrying in {current_delay}s...")
                        time.sleep(current_delay)
                        current_delay *= backoff  # Exponential backoff

            # This should never be reached, but included for type checker
            raise Exception(f"Unexpected error in retry logic for {func.__name__}")

        return wrapper
    return decorator


class RetryHandler:
    """Handles retry logic for various operations in the pipeline."""

    def __init__(self, max_attempts: int = 3, initial_delay: float = 1.0, backoff: float = 2.0):
        self.max_attempts = max_attempts
        self.initial_delay = initial_delay
        self.backoff = backoff

    def execute_with_retry(self, operation: Callable, *args, **kwargs) -> Any:
        """
        Execute an operation with retry logic.

        Args:
            operation: The operation to execute
            *args: Arguments to pass to the operation
            **kwargs: Keyword arguments to pass to the operation

        Returns:
            Result of the operation
        """
        delay = self.initial_delay

        for attempt in range(self.max_attempts):
            try:
                return operation(*args, **kwargs)
            except Exception as e:
                if attempt == self.max_attempts - 1:  # Last attempt
                    logger.error(f"Operation failed after {self.max_attempts} attempts: {str(e)}")
                    raise e
                else:
                    logger.warning(f"Attempt {attempt + 1} failed: {str(e)}. Retrying in {delay}s...")
                    time.sleep(delay)
                    delay *= self.backoff

    def execute_content_extraction_with_retry(self, url: str, max_tokens: int = 512):
        """
        Execute content extraction with retry logic.

        Args:
            url: URL to extract content from
            max_tokens: Maximum tokens per chunk

        Returns:
            Result of content extraction
        """
        from .content_extractor import extract_content_from_url

        return self.execute_with_retry(extract_content_from_url, url, max_tokens)

    def execute_embedding_generation_with_retry(self, texts: list, model: str = "text-embedding-ada-002"):
        """
        Execute embedding generation with retry logic.

        Args:
            texts: List of texts to generate embeddings for
            model: The embedding model to use

        Returns:
            Result of embedding generation
        """
        from .embedding_generator import generate_embeddings

        return self.execute_with_retry(generate_embeddings, texts, model)

    def execute_qdrant_storage_with_retry(self, embeddings: list, collection_name: str = "documentation_embeddings"):
        """
        Execute Qdrant storage with retry logic.

        Args:
            embeddings: List of embeddings to store
            collection_name: Name of the Qdrant collection

        Returns:
            Result of storage operation
        """
        from .embedding_storage import batch_store_embeddings_in_qdrant

        return self.execute_with_retry(batch_store_embeddings_in_qdrant, embeddings, collection_name)