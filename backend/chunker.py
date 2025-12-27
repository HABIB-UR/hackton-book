import tiktoken
from typing import List
import logging
from models import ContentChunk

logger = logging.getLogger(__name__)

def chunk_content(content: str, max_tokens: int = 512) -> List[ContentChunk]:
    """
    Chunk content based on token limits using tiktoken.

    Args:
        content: The content to chunk
        max_tokens: Maximum number of tokens per chunk (default: 512)

    Returns:
        List of ContentChunk objects
    """
    try:
        # Initialize the encoding (using cl100k_base which is used by text-embedding-ada-002)
        encoding = tiktoken.get_encoding("cl100k_base")

        # Split content into sentences or paragraphs to maintain context
        paragraphs = content.split('\n\n')

        chunks = []
        current_chunk = ""
        current_token_count = 0
        chunk_index = 0

        for paragraph in paragraphs:
            # Encode the paragraph to get token count
            paragraph_tokens = encoding.encode(paragraph)
            paragraph_token_count = len(paragraph_tokens)

            # If a single paragraph is too large, split it into sentences
            if paragraph_token_count > max_tokens:
                sentences = paragraph.split('. ')
                current_sentence_chunk = ""

                for sentence in sentences:
                    sentence_with_dot = sentence + '. '
                    sentence_tokens = encoding.encode(sentence_with_dot)
                    sentence_token_count = len(sentence_tokens)

                    if current_token_count + sentence_token_count > max_tokens:
                        # Finalize current chunk and start a new one
                        if current_chunk.strip():
                            chunk = ContentChunk(
                                id="",
                                content=current_chunk.strip(),
                                source_url="",
                                source_section="",
                                chunk_index=chunk_index,
                                token_count=current_token_count
                            )
                            chunks.append(chunk)
                            chunk_index += 1
                            current_chunk = sentence_with_dot
                            current_token_count = sentence_token_count
                        else:
                            # If the sentence itself is too long, we'll include it anyway
                            chunk = ContentChunk(
                                id="",
                                content=sentence_with_dot.strip(),
                                source_url="",
                                source_section="",
                                chunk_index=chunk_index,
                                token_count=sentence_token_count
                            )
                            chunks.append(chunk)
                            chunk_index += 1
                            current_chunk = ""
                            current_token_count = 0
                    else:
                        current_chunk += sentence_with_dot
                        current_token_count += sentence_token_count
            else:
                # Check if adding this paragraph would exceed the limit
                if current_token_count + paragraph_token_count > max_tokens:
                    # Finalize current chunk and start a new one
                    if current_chunk.strip():
                        chunk = ContentChunk(
                            id="",
                            content=current_chunk.strip(),
                            source_url="",
                            source_section="",
                            chunk_index=chunk_index,
                            token_count=current_token_count
                        )
                        chunks.append(chunk)
                        chunk_index += 1

                    # Start new chunk with this paragraph
                    current_chunk = paragraph + '\n\n'
                    current_token_count = paragraph_token_count
                else:
                    current_chunk += paragraph + '\n\n'
                    current_token_count += paragraph_token_count

        # Add the final chunk if it has content
        if current_chunk.strip():
            chunk = ContentChunk(
                id="",
                content=current_chunk.strip(),
                source_url="",
                source_section="",
                chunk_index=chunk_index,
                token_count=current_token_count
            )
            chunks.append(chunk)

        logger.info(f"Content chunked into {len(chunks)} chunks")
        return chunks

    except Exception as e:
        logger.error(f"Failed to chunk content: {str(e)}")
        raise e

def validate_chunk(chunk: ContentChunk) -> bool:
    """
    Validate a content chunk meets requirements.

    Args:
        chunk: The ContentChunk to validate

    Returns:
        True if valid, False otherwise
    """
    try:
        if not chunk.content or not chunk.content.strip():
            return False
        if chunk.token_count < 0:
            return False
        if chunk.chunk_index < 0:
            return False
        return True
    except Exception:
        return False