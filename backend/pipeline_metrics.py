import time
import logging
from datetime import datetime
from typing import Dict, Any, List
from dataclasses import dataclass

logger = logging.getLogger(__name__)

@dataclass
class PipelineMetrics:
    """Metrics for tracking pipeline performance and status."""
    start_time: float
    end_time: float = 0
    total_pages: int = 0
    processed_pages: int = 0
    total_chunks: int = 0
    processed_chunks: int = 0
    total_embeddings: int = 0
    generated_embeddings: int = 0
    errors: List[str] = None
    success_rate: float = 0.0
    processing_time: float = 0.0

    def __post_init__(self):
        if self.errors is None:
            self.errors = []

    def calculate_metrics(self):
        """Calculate derived metrics after pipeline completion."""
        self.processing_time = self.end_time - self.start_time
        if self.total_pages > 0:
            self.success_rate = (self.processed_pages / self.total_pages) * 100

    def to_dict(self) -> Dict[str, Any]:
        """Convert metrics to dictionary for reporting."""
        return {
            "start_time": datetime.fromtimestamp(self.start_time).isoformat(),
            "end_time": datetime.fromtimestamp(self.end_time).isoformat() if self.end_time > 0 else None,
            "processing_time_seconds": self.processing_time,
            "total_pages": self.total_pages,
            "processed_pages": self.processed_pages,
            "total_chunks": self.total_chunks,
            "processed_chunks": self.processed_chunks,
            "total_embeddings": self.total_embeddings,
            "generated_embeddings": self.generated_embeddings,
            "success_rate_percentage": self.success_rate,
            "error_count": len(self.errors),
            "errors": self.errors
        }

class MetricsCollector:
    """Collects and reports metrics for the pipeline."""

    def __init__(self):
        self.metrics = PipelineMetrics(start_time=time.time())

    def start_collection(self):
        """Start collecting metrics."""
        self.metrics = PipelineMetrics(start_time=time.time())
        logger.info("Started metrics collection")

    def update_pages_processed(self, total: int, processed: int):
        """Update page processing metrics."""
        self.metrics.total_pages = total
        self.metrics.processed_pages = processed
        logger.debug(f"Updated pages processed: {processed}/{total}")

    def update_chunks_processed(self, total: int, processed: int):
        """Update chunk processing metrics."""
        self.metrics.total_chunks = total
        self.metrics.processed_chunks = processed
        logger.debug(f"Updated chunks processed: {processed}/{total}")

    def update_embeddings_processed(self, total: int, generated: int):
        """Update embedding processing metrics."""
        self.metrics.total_embeddings = total
        self.metrics.generated_embeddings = generated
        logger.debug(f"Updated embeddings processed: {generated}/{total}")

    def add_error(self, error: str):
        """Add an error to the metrics."""
        self.metrics.errors.append(error)
        logger.warning(f"Added error to metrics: {error}")

    def finalize(self):
        """Finalize metrics collection."""
        self.metrics.end_time = time.time()
        self.metrics.calculate_metrics()
        logger.info("Finalized metrics collection")

    def get_metrics_report(self) -> str:
        """Generate a human-readable metrics report."""
        if self.metrics.end_time == 0:
            self.finalize()

        report = f"""
Pipeline Metrics Report
=======================
Start Time: {datetime.fromtimestamp(self.metrics.start_time).strftime('%Y-%m-%d %H:%M:%S')}
End Time: {datetime.fromtimestamp(self.metrics.end_time).strftime('%Y-%m-%d %H:%M:%S')}
Processing Time: {self.metrics.processing_time:.2f} seconds

Pages: {self.metrics.processed_pages}/{self.metrics.total_pages} ({self.metrics.success_rate:.2f}%)
Chunks: {self.metrics.processed_chunks}/{self.metrics.total_chunks}
Embeddings: {self.metrics.generated_embeddings}/{self.metrics.total_embeddings}

Errors: {len(self.metrics.errors)}
        """

        if self.metrics.errors:
            report += "\nErrors:\n"
            for i, error in enumerate(self.metrics.errors, 1):
                report += f"  {i}. {error}\n"

        return report

    def get_metrics_dict(self) -> Dict[str, Any]:
        """Get metrics as a dictionary."""
        if self.metrics.end_time == 0:
            self.finalize()

        return self.metrics.to_dict()

    def log_performance_metrics(self):
        """Log performance metrics to logger."""
        if self.metrics.end_time == 0:
            self.finalize()

        logger.info(f"Pipeline completed in {self.metrics.processing_time:.2f}s")
        logger.info(f"Pages processed: {self.metrics.processed_pages}/{self.metrics.total_pages} ({self.metrics.success_rate:.2f}%)")
        logger.info(f"Embeddings generated: {self.metrics.generated_embeddings}/{self.metrics.total_embeddings}")
        logger.info(f"Total errors: {len(self.metrics.errors)}")