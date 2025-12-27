# Data Model: RAG Website Embedding

## Entity: DocumentationContent

**Description**: Represents extracted text content from a documentation website page

**Fields**:
- `url` (string): The source URL of the documentation page
- `title` (string): The title of the documentation page
- `content` (string): The clean, extracted text content
- `section` (string): The section or chapter identifier
- `created_at` (timestamp): When the content was extracted

**Validation rules**:
- URL must be a valid HTTP/HTTPS URL
- Content must not be empty
- Title must not be empty

## Entity: ContentChunk

**Description**: Represents a segmented portion of documentation content suitable for embedding

**Fields**:
- `id` (string): Unique identifier for the chunk
- `content` (string): The text content of the chunk
- `source_url` (string): Reference to the original page URL
- `source_section` (string): Reference to the section/heading
- `chunk_index` (integer): Position of the chunk within the original content
- `token_count` (integer): Number of tokens in the chunk

**Validation rules**:
- Content must not exceed maximum token limit (e.g., 1024 tokens)
- Source URL must be valid
- Chunk index must be non-negative

## Entity: Embedding

**Description**: Vector representation of content chunk with associated metadata

**Fields**:
- `id` (string): Unique identifier for the embedding
- `vector` (array of floats): The embedding vector values
- `chunk_id` (string): Reference to the source content chunk
- `metadata` (object): Additional metadata including source_url, source_section
- `created_at` (timestamp): When the embedding was generated

**Validation rules**:
- Vector must have consistent dimensions across all embeddings
- Chunk ID must reference an existing content chunk
- Metadata must include source_url and source_section

## Entity: ProcessingJob

**Description**: Represents a complete run of the ingestion pipeline

**Fields**:
- `id` (string): Unique identifier for the job
- `website_url` (string): The target website being processed
- `status` (string): Current status (pending, running, completed, failed)
- `pages_processed` (integer): Number of pages successfully processed
- `total_pages` (integer): Total number of pages identified
- `start_time` (timestamp): When the job started
- `end_time` (timestamp): When the job completed
- `errors` (array): List of any errors encountered during processing

**Validation rules**:
- Website URL must be valid
- Status must be one of the allowed values
- Pages processed must not exceed total pages