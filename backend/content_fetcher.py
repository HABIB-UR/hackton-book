import requests
from typing import Optional
import logging
from errors import ContentExtractionError, handle_error

logger = logging.getLogger(__name__)

def fetch_web_content(url: str, timeout: int = 30) -> Optional[str]:
    """
    Fetch content from a web page using requests.

    Args:
        url: The URL to fetch content from
        timeout: Request timeout in seconds

    Returns:
        The content of the webpage as a string, or None if failed
    """
    try:
        headers = {
            'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36'
        }

        response = requests.get(url, timeout=timeout, headers=headers)
        response.raise_for_status()  # Raise an exception for bad status codes

        # Check if the content is HTML
        content_type = response.headers.get('content-type', '').lower()
        if 'text/html' not in content_type and 'application/xhtml+xml' not in content_type:
            logger.warning(f"URL {url} does not appear to contain HTML content: {content_type}")

        return response.text

    except requests.RequestException as e:
        error_msg = handle_error(e, f"fetching content from {url}")
        raise ContentExtractionError(f"Failed to fetch content from {url}: {error_msg}")
    except Exception as e:
        error_msg = handle_error(e, f"fetching content from {url}")
        raise ContentExtractionError(f"Unexpected error when fetching content from {url}: {error_msg}")