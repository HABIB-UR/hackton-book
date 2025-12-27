from typing import Optional
import logging
from datetime import datetime
from models import ProcessingJob

logger = logging.getLogger(__name__)

class ProgressTracker:
    """Track progress of the content extraction process."""

    def __init__(self, total_items: int, job: Optional[ProcessingJob] = None):
        """
        Initialize the progress tracker.

        Args:
            total_items: Total number of items to process
            job: Optional ProcessingJob to update
        """
        self.total_items = total_items
        self.processed_items = 0
        self.job = job
        self.start_time = datetime.now()

    def update_progress(self, processed: int = 1, description: str = "") -> None:
        """
        Update the progress with the number of items processed.

        Args:
            processed: Number of items processed since last update (default: 1)
            description: Optional description of what was processed
        """
        self.processed_items += processed

        # Calculate percentage
        if self.total_items > 0:
            percentage = (self.processed_items / self.total_items) * 100
        else:
            percentage = 0

        # Update job if provided
        if self.job:
            self.job.pages_processed = self.processed_items

        # Log progress
        if description:
            logger.info(f"Progress: {self.processed_items}/{self.total_items} ({percentage:.1f}%) - {description}")
        else:
            logger.info(f"Progress: {self.processed_items}/{self.total_items} ({percentage:.1f}%)")

    def get_progress_percentage(self) -> float:
        """
        Get the current progress as a percentage.

        Returns:
            Current progress percentage
        """
        if self.total_items > 0:
            return (self.processed_items / self.total_items) * 100
        return 0.0

    def get_elapsed_time(self) -> float:
        """
        Get the elapsed time in seconds since tracking started.

        Returns:
            Elapsed time in seconds
        """
        return (datetime.now() - self.start_time).total_seconds()

    def get_remaining_items(self) -> int:
        """
        Get the number of items remaining to be processed.

        Returns:
            Number of remaining items
        """
        return max(0, self.total_items - self.processed_items)

def track_extraction_progress(urls: list, job: Optional[ProcessingJob] = None) -> ProgressTracker:
    """
    Create a progress tracker for content extraction.

    Args:
        urls: List of URLs to process
        job: Optional ProcessingJob to update

    Returns:
        ProgressTracker instance
    """
    return ProgressTracker(len(urls), job)