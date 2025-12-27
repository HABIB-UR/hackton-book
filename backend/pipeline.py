from typing import List, Tuple
import logging
from .config import Config
from .website_validator import validate_website, discover_all_pages
from .models import ProcessingJob
from .errors import handle_error

logger = logging.getLogger(__name__)

def run_website_validation(website_url: str) -> Tuple[bool, List[str]]:
    """
    Run website validation and page discovery.

    Args:
        website_url: The website URL to validate

    Returns:
        Tuple of (is_valid, list_of_page_urls)
    """
    logger.info(f"Starting website validation for: {website_url}")

    # Validate website accessibility
    is_valid, error_msg = validate_website(website_url)
    if not is_valid:
        logger.error(f"Website validation failed: {error_msg}")
        return False, []

    logger.info("Website is accessible")

    # Discover all pages
    page_urls = discover_all_pages(website_url)
    logger.info(f"Discovered {len(page_urls)} pages")

    return True, page_urls

def add_website_validation_to_pipeline(job: ProcessingJob) -> ProcessingJob:
    """
    Add website validation to the main pipeline.

    Args:
        job: The processing job to update

    Returns:
        Updated processing job
    """
    try:
        is_valid, page_urls = run_website_validation(job.website_url)
        if not is_valid:
            job.status = "failed"
            return job

        job.total_pages = len(page_urls)
        job.status = "running"
        logger.info(f"Pipeline initialized with {job.total_pages} pages to process")
        return job

    except Exception as e:
        error_msg = handle_error(e, "website_validation_pipeline")
        job.status = "failed"
        # Assuming job has an errors attribute - we'll add error to the job
        from .errors import add_error_to_job
        add_error_to_job(job, error_msg)
        return job