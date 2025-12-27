import os
from dotenv import load_dotenv
from typing import Optional

# Load environment variables from .env file
load_dotenv()

class Config:
    """Configuration class to manage environment variables and settings."""

    # Qdrant configuration
    QDRANT_URL: str = os.getenv("QDRANT_URL", "")
    QDRANT_API_KEY: str = os.getenv("QDRANT_API_KEY", "")

    # OpenAI configuration
    OPENAI_API_KEY: str = os.getenv("OPENAI_API_KEY", "")

    # Website configuration
    WEBSITE_URL: str = os.getenv("WEBSITE_URL", "")

    # Default settings
    CHUNK_SIZE: int = int(os.getenv("CHUNK_SIZE", "512"))
    BATCH_SIZE: int = int(os.getenv("BATCH_SIZE", "10"))
    REQUEST_TIMEOUT: int = int(os.getenv("REQUEST_TIMEOUT", "30"))

    @classmethod
    def validate(cls) -> list[str]:
        """Validate required configuration values."""
        errors = []

        if not cls.QDRANT_URL:
            errors.append("QDRANT_URL is required")
        if not cls.QDRANT_API_KEY:
            errors.append("QDRANT_API_KEY is required")
        if not cls.OPENAI_API_KEY:
            errors.append("OPENAI_API_KEY is required")
        if not cls.WEBSITE_URL:
            errors.append("WEBSITE_URL is required")

        return errors