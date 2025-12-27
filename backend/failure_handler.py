import logging
from typing import List, Any, Callable, Optional
from .models import ProcessingJob
from .errors import add_error_to_job, handle_error

logger = logging.getLogger(__name__)

class FailureHandler:
    """Handles graceful degradation when partial failures occur in the pipeline."""

    def __init__(self, job: Optional[ProcessingJob] = None):
        self.job = job

    def handle_content_extraction_failure(self, url: str, error: Exception) -> bool:
        """
        Handle failure during content extraction for a specific URL.

        Args:
            url: The URL that failed to extract content from
            error: The error that occurred

        Returns:
            True if we should continue processing, False if we should stop
        """
        error_msg = handle_error(error, f"content extraction from {url}")
        logger.warning(f"Content extraction failed for {url}: {error_msg}")

        if self.job:
            add_error_to_job(self.job, f"Failed to extract content from {url}: {error_msg}")

        # For content extraction, we continue processing other URLs
        return True

    def handle_embedding_generation_failure(self, text: str, error: Exception) -> bool:
        """
        Handle failure during embedding generation for a specific text.

        Args:
            text: The text that failed to generate embedding for
            error: The error that occurred

        Returns:
            True if we should continue processing, False if we should stop
        """
        error_msg = handle_error(error, f"embedding generation for text of length {len(text)}")
        logger.warning(f"Embedding generation failed for text: {error_msg}")

        if self.job:
            add_error_to_job(self.job, f"Failed to generate embedding: {error_msg}")

        # For embedding generation, we continue processing other texts
        return True

    def handle_storage_failure(self, chunk_id: str, error: Exception) -> bool:
        """
        Handle failure during storage of an embedding.

        Args:
            chunk_id: The chunk ID that failed to store
            error: The error that occurred

        Returns:
            True if we should continue processing, False if we should stop
        """
        error_msg = handle_error(error, f"storing embedding for chunk {chunk_id}")
        logger.warning(f"Storage failed for chunk {chunk_id}: {error_msg}")

        if self.job:
            add_error_to_job(self.job, f"Failed to store embedding for chunk {chunk_id}: {error_msg}")

        # For storage, we continue processing other embeddings
        return True

    def handle_url_discovery_failure(self, website_url: str, error: Exception) -> List[str]:
        """
        Handle failure during URL discovery, providing fallback URLs.

        Args:
            website_url: The base website URL
            error: The error that occurred

        Returns:
            List of URLs to process as fallback
        """
        error_msg = handle_error(error, f"URL discovery for {website_url}")
        logger.warning(f"URL discovery failed: {error_msg}")

        if self.job:
            add_error_to_job(self.job, f"URL discovery failed: {error_msg}")

        # As fallback, return just the base URL
        fallback_urls = [website_url]
        logger.info(f"Using fallback: processing base URL only: {website_url}")
        return fallback_urls

    def continue_on_error(self, operation: Callable, *args, **kwargs) -> Any:
        """
        Execute an operation and continue if it fails.

        Args:
            operation: The operation to execute
            *args: Arguments to pass to the operation
            **kwargs: Keyword arguments to pass to the operation

        Returns:
            Result of the operation, or None if it failed
        """
        try:
            return operation(*args, **kwargs)
        except Exception as e:
            error_msg = handle_error(e, f"operation {operation.__name__}")
            logger.warning(f"Operation {operation.__name__} failed: {error_msg}")

            if self.job:
                add_error_to_job(self.job, f"Operation {operation.__name__} failed: {error_msg}")

            return None

    def process_with_degradation(self, items: List[Any], processor: Callable, failure_handler: Callable) -> List[Any]:
        """
        Process a list of items, handling failures gracefully.

        Args:
            items: List of items to process
            processor: Function to process each item
            failure_handler: Function to handle failures for each item

        Returns:
            List of successfully processed items
        """
        results = []
        failed_count = 0

        for i, item in enumerate(items):
            try:
                result = processor(item)
                if result is not None:
                    results.append(result)
                else:
                    failed_count += 1
                    should_continue = failure_handler(item, Exception("Processor returned None"))
                    if not should_continue:
                        break
            except Exception as e:
                failed_count += 1
                should_continue = failure_handler(item, e)
                if not should_continue:
                    break

            # Log progress
            if (i + 1) % 10 == 0:  # Log every 10 items
                logger.info(f"Processed {i + 1}/{len(items)} items, {failed_count} failed")

        logger.info(f"Processing completed: {len(results)} successful, {failed_count} failed out of {len(items)} total")
        return results

    def update_job_status_with_degradation(self) -> None:
        """
        Update the job status considering partial failures.
        """
        if not self.job:
            return

        # If we have errors but also processed some pages, mark as completed with warnings
        if self.job.errors and self.job.pages_processed > 0:
            self.job.status = "completed_with_warnings"
            logger.info("Pipeline completed with warnings due to partial failures")
        elif self.job.errors:
            self.job.status = "failed"
            logger.error("Pipeline failed completely")
        else:
            self.job.status = "completed"
            logger.info("Pipeline completed successfully")