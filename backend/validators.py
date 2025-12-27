import requests
from urllib.parse import urlparse
from typing import Optional
import logging

logger = logging.getLogger(__name__)

def is_valid_url(url: str) -> bool:
    """
    Validate if the given string is a properly formatted URL.

    Args:
        url: The URL string to validate

    Returns:
        True if the URL is valid, False otherwise
    """
    try:
        result = urlparse(url)
        return all([result.scheme, result.netloc])
    except Exception:
        return False

def is_url_accessible(url: str, timeout: int = 30) -> bool:
    """
    Check if the given URL is accessible by making a GET request.

    Args:
        url: The URL to check for accessibility
        timeout: Request timeout in seconds (default: 30)

    Returns:
        True if the URL is accessible, False otherwise
    """
    try:
        response = requests.get(url, timeout=timeout)
        return response.status_code == 200
    except requests.RequestException as e:
        logger.warning(f"URL {url} is not accessible: {str(e)}")
        return False

def validate_website_accessibility(website_url: str, timeout: int = 30) -> tuple[bool, Optional[str]]:
    """
    Validate that the website URL is properly formatted and accessible.

    Args:
        website_url: The website URL to validate
        timeout: Request timeout in seconds (default: 30)

    Returns:
        Tuple of (is_valid, error_message). is_valid is True if URL is valid and accessible.
    """
    if not is_valid_url(website_url):
        return False, f"Invalid URL format: {website_url}"

    if not is_url_accessible(website_url, timeout):
        return False, f"Website is not accessible: {website_url}"

    return True, None