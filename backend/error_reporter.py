import logging
from typing import List, Dict, Any
from .models import ProcessingJob

class ErrorReporter:
    """Handles comprehensive error reporting and logging for the pipeline."""

    def __init__(self):
        self.logger = logging.getLogger(__name__)

    def report_error(self, error: Exception, context: str = "", details: Dict[str, Any] = None) -> str:
        """
        Report an error with context and details.

        Args:
            error: The exception that occurred
            context: Context information about where the error occurred
            details: Additional details about the error

        Returns:
            A formatted error message
        """
        error_msg = f"Error in {context}: {str(error)}" if context else f"Error: {str(error)}"

        if details:
            error_msg += f" (Details: {details})"

        self.logger.error(error_msg, exc_info=True)
        return error_msg

    def report_pipeline_errors(self, job: ProcessingJob) -> str:
        """
        Generate a comprehensive error report for a processing job.

        Args:
            job: The ProcessingJob to report on

        Returns:
            Formatted error report
        """
        if not job.errors:
            return "No errors reported for this job."

        report = f"Pipeline Job Report (ID: {job.id})\n"
        report += f"Website URL: {job.website_url}\n"
        report += f"Status: {job.status}\n"
        report += f"Pages Processed: {job.pages_processed}/{job.total_pages}\n"
        report += f"Errors Encountered: {len(job.errors)}\n\n"
        report += "Error Details:\n"

        for i, error in enumerate(job.errors, 1):
            report += f"  {i}. {error}\n"

        self.logger.info(report)
        return report

    def log_pipeline_summary(self, job: ProcessingJob, start_time: float, end_time: float) -> None:
        """
        Log a summary of the pipeline execution.

        Args:
            job: The ProcessingJob that was executed
            start_time: Start time of the pipeline
            end_time: End time of the pipeline
        """
        execution_time = end_time - start_time
        summary = (
            f"Pipeline Summary:\n"
            f"  Website: {job.website_url}\n"
            f"  Status: {job.status}\n"
            f"  Pages: {job.pages_processed}/{job.total_pages}\n"
            f"  Execution Time: {execution_time:.2f}s\n"
            f"  Errors: {len(job.errors) if job.errors else 0}"
        )
        self.logger.info(summary)

    def generate_error_statistics(self, jobs: List[ProcessingJob]) -> Dict[str, Any]:
        """
        Generate statistics about errors across multiple jobs.

        Args:
            jobs: List of ProcessingJobs to analyze

        Returns:
            Dictionary with error statistics
        """
        stats = {
            "total_jobs": len(jobs),
            "successful_jobs": 0,
            "failed_jobs": 0,
            "total_errors": 0,
            "error_types": {},
            "average_processing_time": 0
        }

        for job in jobs:
            if job.status == "completed":
                stats["successful_jobs"] += 1
            else:
                stats["failed_jobs"] += 1

            if job.errors:
                stats["total_errors"] += len(job.errors)
                for error in job.errors:
                    error_type = error.split(":")[0] if ":" in error else "Unknown"
                    stats["error_types"][error_type] = stats["error_types"].get(error_type, 0) + 1

        return stats