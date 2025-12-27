from bs4 import BeautifulSoup
from typing import Optional
import logging
from errors import ContentExtractionError

logger = logging.getLogger(__name__)

def parse_html_content(html_content: str) -> BeautifulSoup:
    """
    Parse HTML content using BeautifulSoup.

    Args:
        html_content: The raw HTML content to parse

    Returns:
        BeautifulSoup object representing the parsed HTML
    """
    try:
        soup = BeautifulSoup(html_content, 'html.parser')
        return soup
    except Exception as e:
        logger.error(f"Failed to parse HTML content: {str(e)}")
        raise ContentExtractionError(f"HTML parsing failed: {str(e)}")

def extract_clean_text(soup: BeautifulSoup) -> str:
    """
    Extract clean text content from parsed HTML, removing HTML tags and scripts.

    Args:
        soup: BeautifulSoup object representing parsed HTML

    Returns:
        Clean text content extracted from the HTML
    """
    try:
        # Remove script and style elements
        for script in soup(["script", "style", "nav", "header", "footer", "aside"]):
            script.decompose()

        # Get text content
        text = soup.get_text()

        # Clean up the text
        lines = (line.strip() for line in text.splitlines())
        chunks = (phrase.strip() for line in lines for phrase in line.split("  "))
        text = ' '.join(chunk for chunk in chunks if chunk)

        return text

    except Exception as e:
        logger.error(f"Failed to extract clean text: {str(e)}")
        raise ContentExtractionError(f"Text extraction failed: {str(e)}")

def extract_title(soup: BeautifulSoup) -> Optional[str]:
    """
    Extract the title from the parsed HTML.

    Args:
        soup: BeautifulSoup object representing parsed HTML

    Returns:
        The title text, or None if no title found
    """
    try:
        title_tag = soup.find('title')
        if title_tag:
            return title_tag.get_text().strip()

        # If no title tag, try h1
        h1_tag = soup.find('h1')
        if h1_tag:
            return h1_tag.get_text().strip()

        return None
    except Exception as e:
        logger.error(f"Failed to extract title: {str(e)}")
        return None

def extract_section_info(soup: BeautifulSoup) -> str:
    """
    Extract section information from the parsed HTML.

    Args:
        soup: BeautifulSoup object representing parsed HTML

    Returns:
        Section information (e.g., H1, H2 tags)
    """
    try:
        # Look for main heading tags
        section_parts = []

        # Check for H1
        h1 = soup.find('h1')
        if h1:
            section_parts.append(h1.get_text().strip())

        # Check for H2
        h2 = soup.find('h2')
        if h2:
            section_parts.append(h2.get_text().strip())

        # Return the first meaningful section header found
        if section_parts:
            return " > ".join(section_parts[:2])  # Limit to first 2 levels

        return "General"
    except Exception as e:
        logger.error(f"Failed to extract section info: {str(e)}")
        return "General"