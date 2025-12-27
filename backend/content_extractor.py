from typing import List, Tuple
import logging
from content_fetcher import fetch_web_content
from html_parser import parse_html_content, extract_clean_text, extract_title, extract_section_info
from chunker import chunk_content
from models import DocumentationContent, ContentChunk
from errors import ContentExtractionError, handle_error

logger = logging.getLogger(__name__)

def extract_content_from_url(url: str, max_tokens: int = 512) -> Tuple[DocumentationContent, List[ContentChunk]]:
    """
    Extract content from a single URL and chunk it.

    Args:
        url: The URL to extract content from
        max_tokens: Maximum tokens per chunk

    Returns:
        Tuple of (DocumentationContent, List of ContentChunks)
    """
    try:
        # Fetch the web content
        html_content = fetch_web_content(url)
        if not html_content:
            raise ContentExtractionError(f"Failed to fetch content from {url}")

        # Parse the HTML
        soup = parse_html_content(html_content)

        # Extract clean text
        clean_text = extract_clean_text(soup)

        # Extract title
        title = extract_title(soup) or "Untitled"

        # Extract section info
        section = extract_section_info(soup)

        # Create DocumentationContent
        doc_content = DocumentationContent(
            url=url,
            title=title,
            content=clean_text,
            section=section
        )

        # Chunk the content
        chunks = chunk_content(clean_text, max_tokens)

        # Update chunk source information
        for chunk in chunks:
            chunk.source_url = url
            chunk.source_section = section

        logger.info(f"Successfully extracted and chunked content from {url}")
        return doc_content, chunks

    except Exception as e:
        error_msg = handle_error(e, f"extracting content from {url}")
        raise ContentExtractionError(error_msg)

def extract_content_from_urls(urls: List[str], max_tokens: int = 512) -> Tuple[List[DocumentationContent], List[ContentChunk]]:
    """
    Extract content from multiple URLs.

    Args:
        urls: List of URLs to extract content from
        max_tokens: Maximum tokens per chunk

    Returns:
        Tuple of (List of DocumentationContents, List of all ContentChunks)
    """
    all_doc_contents = []
    all_chunks = []

    for i, url in enumerate(urls):
        try:
            logger.info(f"Processing URL {i+1}/{len(urls)}: {url}")
            doc_content, chunks = extract_content_from_url(url, max_tokens)
            all_doc_contents.append(doc_content)
            all_chunks.extend(chunks)
        except Exception as e:
            logger.error(f"Failed to extract content from {url}: {str(e)}")
            # Continue with other URLs
            continue

    logger.info(f"Extraction completed: {len(all_doc_contents)} documents, {len(all_chunks)} chunks")
    return all_doc_contents, all_chunks