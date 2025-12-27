import requests
from urllib.parse import urljoin, urlparse
from bs4 import BeautifulSoup
from typing import List, Optional, Tuple
import logging
from validators import validate_website_accessibility

logger = logging.getLogger(__name__)

def validate_website(website_url: str, timeout: int = 30) -> Tuple[bool, Optional[str]]:
    """
    Validate that the website URL is accessible and properly formatted.

    Args:
        website_url: The website URL to validate
        timeout: Request timeout in seconds

    Returns:
        Tuple of (is_valid, error_message)
    """
    return validate_website_accessibility(website_url, timeout)

def discover_sitemap(website_url: str) -> Optional[str]:
    """
    Try to discover the sitemap.xml URL for the website.

    Args:
        website_url: The base website URL

    Returns:
        URL to sitemap.xml if found, None otherwise
    """
    sitemap_url = urljoin(website_url, "sitemap.xml")

    try:
        response = requests.get(sitemap_url, timeout=10)
        if response.status_code == 200:
            # Verify it's actually an XML sitemap
            content_type = response.headers.get('content-type', '').lower()
            if 'xml' in content_type or response.text.startswith('<?xml'):
                logger.info(f"Found sitemap at {sitemap_url}")
                return sitemap_url
    except requests.RequestException:
        logger.debug(f"No sitemap found at {sitemap_url}")

    return None

def discover_robots_txt_sitemap(website_url: str) -> Optional[str]:
    """
    Try to discover sitemap from robots.txt file.

    Args:
        website_url: The base website URL

    Returns:
        URL to sitemap.xml if found in robots.txt, None otherwise
    """
    robots_url = urljoin(website_url, "robots.txt")

    try:
        response = requests.get(robots_url, timeout=10)
        if response.status_code == 200:
            # Look for sitemap entries in robots.txt
            for line in response.text.splitlines():
                line = line.strip().lower()
                if line.startswith('sitemap:'):
                    sitemap_url = line[8:].strip()  # Remove 'sitemap:' prefix
                    # If it's a relative URL, make it absolute
                    if sitemap_url.startswith('/'):
                        sitemap_url = urljoin(website_url, sitemap_url)
                    elif not sitemap_url.startswith('http'):
                        sitemap_url = urljoin(website_url, sitemap_url)

                    logger.info(f"Found sitemap from robots.txt: {sitemap_url}")
                    return sitemap_url
    except requests.RequestException:
        logger.debug(f"Could not access robots.txt at {robots_url}")

    return None

def parse_sitemap(sitemap_url: str) -> List[str]:
    """
    Parse a sitemap.xml file and extract all URLs.

    Args:
        sitemap_url: URL to the sitemap.xml file

    Returns:
        List of URLs found in the sitemap
    """
    try:
        response = requests.get(sitemap_url, timeout=30)
        response.raise_for_status()

        soup = BeautifulSoup(response.text, 'xml')
        urls = []

        # Look for <url><loc>...</loc></url> elements
        for url_elem in soup.find_all('url'):
            loc_elem = url_elem.find('loc')
            if loc_elem:
                urls.append(loc_elem.text.strip())

        logger.info(f"Discovered {len(urls)} URLs from sitemap")
        return urls

    except Exception as e:
        logger.error(f"Failed to parse sitemap {sitemap_url}: {str(e)}")
        return []

def discover_all_pages(website_url: str) -> List[str]:
    """
    Discover all pages on the website by checking sitemap and crawling if needed.

    Args:
        website_url: The base website URL

    Returns:
        List of discovered page URLs
    """
    logger.info(f"Discovering pages for website: {website_url}")

    # First try to find pages via sitemap.xml
    sitemap_url = discover_sitemap(website_url)
    if not sitemap_url:
        # Try to find sitemap from robots.txt
        sitemap_url = discover_robots_txt_sitemap(website_url)

    if sitemap_url:
        urls = parse_sitemap(sitemap_url)
        if urls:
            return urls

    # If no sitemap or sitemap parsing failed, we could implement basic crawling
    # For now, we'll return just the base URL as a starting point
    # In a full implementation, we would add web crawling logic here
    logger.warning(f"No sitemap found for {website_url}, returning base URL only")
    return [website_url]