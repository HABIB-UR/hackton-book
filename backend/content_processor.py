from typing import List, Tuple
import logging
from models import DocumentationContent, ContentChunk
from content_extractor import extract_content_from_urls
from chunker import validate_chunk
from errors import add_error_to_job, handle_error

logger = logging.getLogger(__name__)

def create_documentation_content(url: str, title: str, content: str, section: str) -> DocumentationContent:
    """
    Create a DocumentationContent instance with validation.

    Args:
        url: Source URL
        title: Document title
        content: Document content
        section: Document section

    Returns:
        DocumentationContent instance
    """
    try:
        doc_content = DocumentationContent(
            url=url,
            title=title,
            content=content,
            section=section
        )
        return doc_content
    except Exception as e:
        error_msg = handle_error(e, f"creating DocumentationContent for {url}")
        raise ValueError(error_msg)

def create_content_chunk(content: str, source_url: str, source_section: str, chunk_index: int, token_count: int) -> ContentChunk:
    """
    Create a ContentChunk instance with validation.

    Args:
        content: Chunk content
        source_url: Source URL
        source_section: Source section
        chunk_index: Index of the chunk
        token_count: Number of tokens in the chunk

    Returns:
        ContentChunk instance
    """
    try:
        chunk = ContentChunk(
            id="",
            content=content,
            source_url=source_url,
            source_section=source_section,
            chunk_index=chunk_index,
            token_count=token_count
        )

        # Validate the chunk
        if not validate_chunk(chunk):
            raise ValueError(f"Invalid chunk created for URL: {source_url}, index: {chunk_index}")

        return chunk
    except Exception as e:
        error_msg = handle_error(e, f"creating ContentChunk for {source_url}, index: {chunk_index}")
        raise ValueError(error_msg)

def process_content_extraction(urls: List[str], max_tokens: int = 512) -> Tuple[List[DocumentationContent], List[ContentChunk]]:
    """
    Process content extraction for multiple URLs and create model instances.

    Args:
        urls: List of URLs to process
        max_tokens: Maximum tokens per chunk

    Returns:
        Tuple of (List of DocumentationContent, List of ContentChunk)
    """
    try:
        # Extract content from all URLs
        doc_contents, all_chunks = extract_content_from_urls(urls, max_tokens)

        # Validate and return the results
        logger.info(f"Processed {len(doc_contents)} documentation contents and {len(all_chunks)} content chunks")
        return doc_contents, all_chunks

    except Exception as e:
        error_msg = handle_error(e, "content extraction process")
        raise Exception(error_msg)

def validate_content_extraction_results(doc_contents: List[DocumentationContent], chunks: List[ContentChunk]) -> Tuple[bool, List[str]]:
    """
    Validate the results of content extraction.

    Args:
        doc_contents: List of DocumentationContent objects
        chunks: List of ContentChunk objects

    Returns:
        Tuple of (is_valid, list of error messages)
    """
    errors = []

    # Check if we have content
    if not doc_contents:
        errors.append("No documentation content extracted")

    # Check if we have chunks
    if not chunks:
        errors.append("No content chunks created")

    # Validate individual chunks
    for i, chunk in enumerate(chunks):
        if not validate_chunk(chunk):
            errors.append(f"Invalid chunk at index {i}")

    # Check for empty content
    for i, doc in enumerate(doc_contents):
        if not doc.content.strip():
            errors.append(f"Empty content for document at index {i}")

    is_valid = len(errors) == 0
    return is_valid, errors