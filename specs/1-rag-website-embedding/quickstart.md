# Quickstart: RAG Website Embedding

## Prerequisites

- Python 3.11 or higher
- `uv` package manager installed
- Access to a Qdrant Cloud instance (or local Qdrant server)
- OpenAI API key (or compatible embedding service API key)

## Setup

1. **Create the backend directory and initialize project:**

```bash
mkdir backend
cd backend
uv init
```

2. **Install required dependencies:**

```bash
uv pip install requests beautifulsoup4 openai qdrant-client python-dotenv tiktoken
```

3. **Create environment file:**

```bash
cp .env.example .env
```

4. **Configure environment variables in `.env`:**

```env
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
OPENAI_API_KEY=your_openai_api_key  # or compatible embedding service key
WEBSITE_URL=the_documentation_website_to_process
```

## Usage

1. **Run the full pipeline:**

```bash
python main.py
```

This will:
- Fetch all pages from the configured website
- Extract clean text content from each page
- Chunk the content appropriately
- Generate embeddings for each chunk
- Store embeddings in Qdrant with metadata

2. **For development/testing, you can also run with specific options:**

```bash
# Process a specific URL
python main.py --url "https://specific-page.com"

# Process with specific chunk size
python main.py --chunk-size 512
```

## Verification

After running the pipeline, verify that:
- Embeddings are stored in your Qdrant collection
- Metadata includes proper source URLs and sections
- Test queries return relevant content chunks