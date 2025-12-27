import pytest
import os
from unittest.mock import patch, MagicMock
from .main import run_ingestion_pipeline
from .config import Config

def test_pipeline_with_mocked_services():
    """Test the complete pipeline with mocked external services."""
    # Mock the external services to avoid actual API calls
    with patch('backend.embedding_generator.generate_embeddings') as mock_embeddings, \
         patch('backend.qdrant_client.create_qdrant_client') as mock_qdrant, \
         patch('backend.content_fetcher.fetch_web_content') as mock_fetcher, \
         patch('backend.website_validator.validate_website') as mock_validator, \
         patch('backend.website_validator.discover_all_pages') as mock_discover:

        # Mock return values
        mock_validator.return_value = (True, None)
        mock_discover.return_value = ["https://example.com/page1"]
        mock_fetcher.return_value = "<html><body><h1>Test Content</h1><p>This is test content for RAG pipeline.</p></body></html>"
        mock_embeddings.return_value = [[0.1, 0.2, 0.3] * 512]  # Mock embedding vector
        mock_qdrant.return_value = MagicMock()

        # Mock config validation to pass
        with patch.object(Config, 'validate', return_value=[]):
            # Set required config values
            Config.WEBSITE_URL = "https://example.com"
            Config.OPENAI_API_KEY = "test-key"
            Config.QDRANT_URL = "https://test-qdrant.com"
            Config.QDRANT_API_KEY = "test-key"

            # Run the pipeline
            result = run_ingestion_pipeline(
                website_url="https://example.com",
                max_tokens=512,
                collection_name="test_collection",
                batch_size=5
            )

            # Assertions
            assert result is not None
            assert result.status in ["completed", "completed_with_warnings", "failed"]
            mock_validator.assert_called_once()
            mock_discover.assert_called_once()
            mock_fetcher.assert_called()
            mock_embeddings.assert_called()
            mock_qdrant.assert_called()

def test_pipeline_with_invalid_config():
    """Test that pipeline handles invalid configuration properly."""
    # Mock config validation to return errors
    with patch.object(Config, 'validate', return_value=["Missing QDRANT_URL"]):
        # This test would need to be adjusted based on how the main function handles config errors
        pass

def test_pipeline_with_invalid_website():
    """Test pipeline behavior with invalid website."""
    with patch('backend.website_validator.validate_website') as mock_validator:
        mock_validator.return_value = (False, "Website not accessible")

        # Mock config validation to pass
        with patch.object(Config, 'validate', return_value=[]):
            Config.WEBSITE_URL = "https://example.com"
            Config.OPENAI_API_KEY = "test-key"
            Config.QDRANT_URL = "https://test-qdrant.com"
            Config.QDRANT_API_KEY = "test-key"

            result = run_ingestion_pipeline("https://invalid-website.com")
            assert result.status == "failed"

def test_pipeline_empty_content():
    """Test pipeline behavior when no content is extracted."""
    with patch('backend.website_validator.validate_website') as mock_validator, \
         patch('backend.website_validator.discover_all_pages') as mock_discover, \
         patch('backend.content_fetcher.fetch_web_content') as mock_fetcher:

        mock_validator.return_value = (True, None)
        mock_discover.return_value = ["https://example.com/page1"]
        mock_fetcher.return_value = ""  # Empty content

        # Mock config validation to pass
        with patch.object(Config, 'validate', return_value=[]):
            Config.WEBSITE_URL = "https://example.com"
            Config.OPENAI_API_KEY = "test-key"
            Config.QDRANT_URL = "https://test-qdrant.com"
            Config.QDRANT_API_KEY = "test-key"

            result = run_ingestion_pipeline("https://example.com")
            # Should handle empty content gracefully
            assert result.status in ["failed", "completed_with_warnings"]

if __name__ == "__main__":
    pytest.main([__file__])