from typing import Optional, List

class RAGException(Exception):
    """Base exception for RAG-related errors."""
    pass

class WebsiteAccessException(RAGException):
    """Raised when there's an issue accessing the website."""
    pass

class ContentExtractionError(RAGException):
    """Raised when content extraction fails."""
    pass

class EmbeddingGenerationError(RAGException):
    """Raised when embedding generation fails."""
    pass

class QdrantStorageError(RAGException):
    """Raised when storing embeddings in Qdrant fails."""
    pass

class ValidationError(RAGException):
    """Raised when validation fails."""
    pass

class ProcessingError(RAGException):
    """Raised when general processing fails."""
    pass

def handle_error(error: Exception, context: str = "", log_error: bool = True) -> str:
    """
    Handle an error by logging it and returning a user-friendly message.

    Args:
        error: The exception that occurred
        context: Context information about where the error occurred
        log_error: Whether to log the error

    Returns:
        A user-friendly error message
    """
    error_msg = f"Error in {context}: {str(error)}" if context else f"Error: {str(error)}"

    if log_error:
        import logging
        logger = logging.getLogger(__name__)
        logger.error(error_msg, exc_info=True)

    return error_msg

def add_error_to_job(job: 'ProcessingJob', error: str) -> None:
    """
    Add an error message to a processing job.

    Args:
        job: The ProcessingJob to add the error to
        error: The error message to add
    """
    if not hasattr(job, 'errors') or job.errors is None:
        job.errors = []
    job.errors.append(error)