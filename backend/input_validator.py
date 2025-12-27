import re
from typing import Any, List, Dict
import logging

logger = logging.getLogger(__name__)

class InputValidator:
    """Validates user-provided parameters for the pipeline."""

    @staticmethod
    def validate_url(url: str) -> bool:
        """
        Validate if the given string is a properly formatted URL.

        Args:
            url: The URL string to validate

        Returns:
            True if the URL is valid, False otherwise
        """
        if not url or not isinstance(url, str):
            return False

        # URL regex pattern
        url_pattern = re.compile(
            r'^https?://'  # http:// or https://
            r'(?:(?:[A-Z0-9](?:[A-Z0-9-]{0,61}[A-Z0-9])?\.)+[A-Z]{2,6}\.?|'  # domain...
            r'localhost|'  # localhost...
            r'\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})'  # ...or ip
            r'(?::\d+)?'  # optional port
            r'(?:/?|[/?]\S+)$', re.IGNORECASE)

        return url_pattern.match(url) is not None

    @staticmethod
    def validate_positive_integer(value: Any, min_value: int = 1) -> bool:
        """
        Validate if the value is a positive integer.

        Args:
            value: The value to validate
            min_value: Minimum allowed value (default: 1)

        Returns:
            True if valid, False otherwise
        """
        try:
            int_value = int(value)
            return int_value >= min_value
        except (ValueError, TypeError):
            return False

    @staticmethod
    def validate_chunk_size(chunk_size: Any) -> bool:
        """
        Validate chunk size parameter.

        Args:
            chunk_size: The chunk size to validate

        Returns:
            True if valid, False otherwise
        """
        if not InputValidator.validate_positive_integer(chunk_size, min_value=10):
            return False

        # Reasonable upper limit for chunk size
        return int(chunk_size) <= 2048

    @staticmethod
    def validate_batch_size(batch_size: Any) -> bool:
        """
        Validate batch size parameter.

        Args:
            batch_size: The batch size to validate

        Returns:
            True if valid, False otherwise
        """
        if not InputValidator.validate_positive_integer(batch_size, min_value=1):
            return False

        # Reasonable upper limit for batch size
        return int(batch_size) <= 100

    @staticmethod
    def validate_collection_name(name: str) -> bool:
        """
        Validate Qdrant collection name.

        Args:
            name: The collection name to validate

        Returns:
            True if valid, False otherwise
        """
        if not name or not isinstance(name, str):
            return False

        # Collection names should match certain rules
        # Only allow alphanumeric, underscore, hyphen, and dot
        pattern = re.compile(r'^[a-zA-Z0-9_-]+$')
        return bool(pattern.match(name)) and 3 <= len(name) <= 63

    @staticmethod
    def validate_embedding_model(model: str) -> bool:
        """
        Validate embedding model name.

        Args:
            model: The model name to validate

        Returns:
            True if valid, False otherwise
        """
        if not model or not isinstance(model, str):
            return False

        # Basic validation for model names
        valid_pattern = re.compile(r'^[a-zA-Z0-9_-]+$')
        return bool(valid_pattern.match(model))

    @staticmethod
    def validate_log_level(level: str) -> bool:
        """
        Validate logging level.

        Args:
            level: The log level to validate

        Returns:
            True if valid, False otherwise
        """
        if not level or not isinstance(level, str):
            return False

        valid_levels = {'DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL'}
        return level.upper() in valid_levels

    @staticmethod
    def validate_top_k(top_k: Any) -> bool:
        """
        Validate top_k parameter for similarity search.

        Args:
            top_k: The top_k value to validate

        Returns:
            True if valid, False otherwise
        """
        if not InputValidator.validate_positive_integer(top_k, min_value=1):
            return False

        # Reasonable upper limit for top_k
        return int(top_k) <= 100

    @staticmethod
    def validate_percentage(percentage: Any) -> bool:
        """
        Validate percentage value.

        Args:
            percentage: The percentage to validate

        Returns:
            True if valid, False otherwise
        """
        try:
            value = float(percentage)
            return 0 <= value <= 100
        except (ValueError, TypeError):
            return False

    @classmethod
    def validate_all_parameters(
        cls,
        website_url: str,
        chunk_size: int,
        batch_size: int,
        collection_name: str,
        log_level: str,
        top_k: int
    ) -> Dict[str, List[str]]:
        """
        Validate all parameters and return any errors.

        Args:
            website_url: Website URL to validate
            chunk_size: Chunk size to validate
            batch_size: Batch size to validate
            collection_name: Collection name to validate
            log_level: Log level to validate
            top_k: Top K value to validate

        Returns:
            Dictionary with validation results and errors
        """
        errors = []

        if not cls.validate_url(website_url):
            errors.append(f"Invalid website URL: {website_url}")

        if not cls.validate_chunk_size(chunk_size):
            errors.append(f"Invalid chunk size: {chunk_size}. Must be between 10 and 2048.")

        if not cls.validate_batch_size(batch_size):
            errors.append(f"Invalid batch size: {batch_size}. Must be between 1 and 100.")

        if not cls.validate_collection_name(collection_name):
            errors.append(f"Invalid collection name: {collection_name}. Must be 3-63 chars, alphanumeric, underscore, or hyphen only.")

        if not cls.validate_log_level(log_level):
            errors.append(f"Invalid log level: {log_level}. Must be one of DEBUG, INFO, WARNING, ERROR, CRITICAL.")

        if not cls.validate_top_k(top_k):
            errors.append(f"Invalid top_k: {top_k}. Must be between 1 and 100.")

        return {
            "is_valid": len(errors) == 0,
            "errors": errors
        }

def validate_input_parameters(args) -> bool:
    """
    Validate command-line arguments.

    Args:
        args: Parsed command-line arguments

    Returns:
        True if all parameters are valid, False otherwise
    """
    validation_result = InputValidator.validate_all_parameters(
        args.website_url or "",
        args.chunk_size,
        args.batch_size,
        args.collection_name,
        args.log_level,
        getattr(args, 'top_k', 5)  # top_k might not be in all args
    )

    if not validation_result["is_valid"]:
        logger.error("Input validation failed:")
        for error in validation_result["errors"]:
            logger.error(f"  - {error}")
        return False

    return True