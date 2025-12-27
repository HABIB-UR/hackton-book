import os
import shutil
import tempfile
import logging
from typing import List, Optional

logger = logging.getLogger(__name__)

class CleanupManager:
    """Manages cleanup of temporary resources and files."""

    def __init__(self):
        self.temporary_files: List[str] = []
        self.temporary_dirs: List[str] = []

    def register_temp_file(self, file_path: str) -> str:
        """
        Register a temporary file for cleanup.

        Args:
            file_path: Path to the temporary file

        Returns:
            The same file path for convenience
        """
        self.temporary_files.append(file_path)
        return file_path

    def register_temp_dir(self, dir_path: str) -> str:
        """
        Register a temporary directory for cleanup.

        Args:
            dir_path: Path to the temporary directory

        Returns:
            The same directory path for convenience
        """
        self.temporary_dirs.append(dir_path)
        return dir_path

    def cleanup_temp_files(self) -> None:
        """Clean up all registered temporary files."""
        for file_path in self.temporary_files:
            try:
                if os.path.exists(file_path):
                    os.remove(file_path)
                    logger.debug(f"Removed temporary file: {file_path}")
            except Exception as e:
                logger.warning(f"Failed to remove temporary file {file_path}: {str(e)}")

        self.temporary_files.clear()

    def cleanup_temp_dirs(self) -> None:
        """Clean up all registered temporary directories."""
        for dir_path in self.temporary_dirs:
            try:
                if os.path.exists(dir_path):
                    shutil.rmtree(dir_path)
                    logger.debug(f"Removed temporary directory: {dir_path}")
            except Exception as e:
                logger.warning(f"Failed to remove temporary directory {dir_path}: {str(e)}")

        self.temporary_dirs.clear()

    def cleanup_all(self) -> None:
        """Clean up all temporary resources."""
        logger.info("Starting cleanup of temporary resources...")
        self.cleanup_temp_files()
        self.cleanup_temp_dirs()
        logger.info("Cleanup completed")

    def create_temp_file(self, prefix: str = "rag_", suffix: str = "") -> str:
        """
        Create a temporary file and register it for cleanup.

        Args:
            prefix: Prefix for the temporary file name
            suffix: Suffix for the temporary file name

        Returns:
            Path to the created temporary file
        """
        temp_file = tempfile.mktemp(prefix=prefix, suffix=suffix)
        return self.register_temp_file(temp_file)

    def create_temp_dir(self, prefix: str = "rag_", suffix: str = "") -> str:
        """
        Create a temporary directory and register it for cleanup.

        Args:
            prefix: Prefix for the temporary directory name
            suffix: Suffix for the temporary directory name

        Returns:
            Path to the created temporary directory
        """
        temp_dir = tempfile.mkdtemp(prefix=prefix, suffix=suffix)
        return self.register_temp_dir(temp_dir)


# Global cleanup manager instance
cleanup_manager = CleanupManager()


def cleanup_temp_resources() -> None:
    """Global function to cleanup temporary resources."""
    cleanup_manager.cleanup_all()


def register_temp_resource(resource_path: str, is_file: bool = True) -> str:
    """
    Register a temporary resource for cleanup.

    Args:
        resource_path: Path to the temporary resource
        is_file: True if it's a file, False if it's a directory

    Returns:
        The same resource path for convenience
    """
    if is_file:
        return cleanup_manager.register_temp_file(resource_path)
    else:
        return cleanup_manager.register_temp_dir(resource_path)


def get_cleanup_manager() -> CleanupManager:
    """Get the global cleanup manager instance."""
    return cleanup_manager