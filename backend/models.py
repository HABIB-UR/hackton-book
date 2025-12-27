from dataclasses import dataclass
from typing import Optional, List, Dict, Any
from datetime import datetime
import uuid

@dataclass
class DocumentationContent:
    """
    Represents extracted text content from a documentation website page.
    """
    url: str
    title: str
    content: str
    section: str
    created_at: datetime = None

    def __post_init__(self):
        if self.created_at is None:
            self.created_at = datetime.now()

        # Validation
        if not self.url or not self.url.strip():
            raise ValueError("URL must be a non-empty string")
        if not self.title or not self.title.strip():
            raise ValueError("Title must be a non-empty string")
        if not self.content or not self.content.strip():
            raise ValueError("Content must be a non-empty string")

@dataclass
class ContentChunk:
    """
    Represents a segmented portion of documentation content suitable for embedding.
    """
    id: str
    content: str
    source_url: str
    source_section: str
    chunk_index: int
    token_count: int

    def __post_init__(self):
        if not self.id or not self.id.strip():
            self.id = str(uuid.uuid4())
        if self.chunk_index < 0:
            raise ValueError("Chunk index must be non-negative")
        if self.token_count < 0:
            raise ValueError("Token count must be non-negative")
        if len(self.content) > 100000:  # Arbitrary limit to prevent oversized chunks
            raise ValueError("Content exceeds maximum allowed size")

@dataclass
class Embedding:
    """
    Vector representation of content chunk with associated metadata.
    """
    id: str
    vector: List[float]
    chunk_id: str
    metadata: Dict[str, Any]
    created_at: datetime = None

    def __post_init__(self):
        if not self.id or not self.id.strip():
            self.id = str(uuid.uuid4())
        if self.created_at is None:
            self.created_at = datetime.now()

        # Validate vector dimensions are consistent
        if not self.vector:
            raise ValueError("Vector must not be empty")
        if not isinstance(self.vector, list):
            raise ValueError("Vector must be a list of floats")
        if not all(isinstance(v, (int, float)) for v in self.vector):
            raise ValueError("All vector values must be numbers")

        # Validate metadata contains required fields
        if 'source_url' not in self.metadata or 'source_section' not in self.metadata:
            raise ValueError("Metadata must include source_url and source_section")

@dataclass
class ProcessingJob:
    """
    Represents a complete run of the ingestion pipeline.
    """
    id: str
    website_url: str
    status: str  # "pending", "running", "completed", "failed"
    pages_processed: int = 0
    total_pages: int = 0
    start_time: datetime = None
    end_time: datetime = None
    errors: List[str] = None

    def __post_init__(self):
        if not self.id or not self.id.strip():
            self.id = str(uuid.uuid4())
        if self.start_time is None:
            self.start_time = datetime.now()
        if self.errors is None:
            self.errors = []

        # Validate required fields
        if not self.website_url or not self.website_url.strip():
            raise ValueError("Website URL must be a non-empty string")
        if self.status not in ["pending", "running", "completed", "failed"]:
            raise ValueError("Status must be one of: pending, running, completed, failed")
        if self.pages_processed > self.total_pages:
            raise ValueError("Pages processed cannot exceed total pages")