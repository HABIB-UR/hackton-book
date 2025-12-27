#!/usr/bin/env python3
"""
RAG Website Embedding Pipeline

This script implements a complete RAG ingestion pipeline that:
1. Validates a documentation website
2. Extracts content from all pages
3. Chunks the content appropriately
4. Generates embeddings
5. Stores embeddings in Qdrant vector database
"""

import argparse
import sys
import logging
from typing import List, Tuple
import time

import config
import logging_config
import models
import website_validator
import content_processor
from embedding_generator import generate_embeddings, create_embedding_instance, validate_embedding_dimensions
from embedding_storage import batch_store_embeddings_in_qdrant
from embedding_tester import run_embedding_verification_test
from progress_tracker import track_extraction_progress
from errors import RAGException, handle_error, add_error_to_job
from cleanup import cleanup_temp_resources


def create_processing_job(website_url: str) -> models.ProcessingJob:
    """Create a new processing job."""
    return models.ProcessingJob(
        id="",
        website_url=website_url,
        status="pending",
        pages_processed=0,
        total_pages=0
    )


def run_ingestion_pipeline(
    website_url: str,
    max_tokens: int = 512,
    collection_name: str = "documentation_embeddings",
    batch_size: int = 10
) -> models.ProcessingJob:
    """
    Run the complete RAG ingestion pipeline.

    Args:
        website_url: URL of the website to process
        max_tokens: Maximum tokens per content chunk
        collection_name: Name of the Qdrant collection to store embeddings
        batch_size: Size of batches for processing

    Returns:
        ProcessingJob with the results
    """
    # Import error handling function
    from errors import add_error_to_job

    # Create processing job
    job = create_processing_job(website_url)
    logging.info(f"Starting ingestion pipeline for: {website_url}")

    try:
        # Phase 1: Website validation
        logging.info("Phase 1: Validating website...")
        is_valid, page_urls = website_validator.validate_website(website_url)
        if not is_valid:
            job.status = "failed"
            add_error_to_job(job, f"Website validation failed: {page_urls}")  # page_urls contains error message when validation fails
            return job

        # Handle the case where page_urls might be None
        if page_urls is None:
            page_urls = []

        job.total_pages = len(page_urls)
        job.status = "running"
        logging.info(f"Found {len(page_urls)} pages to process")

        # Phase 2: Content extraction
        logging.info("Phase 2: Extracting content...")
        doc_contents, all_chunks = content_processor.process_content_extraction(page_urls, max_tokens)

        # Validate extraction results
        is_valid, validation_errors = content_processor.validate_content_extraction_results(doc_contents, all_chunks)
        if not is_valid:
            for error in validation_errors:
                add_error_to_job(job, error)
            job.status = "failed"
            return job

        logging.info(f"Extracted {len(doc_contents)} documentation contents and {len(all_chunks)} chunks")

        # Phase 3: Embedding generation
        logging.info("Phase 3: Generating embeddings...")
        # Extract content from chunks for embedding generation
        texts_to_embed = [chunk.content for chunk in all_chunks]

        # Generate embeddings in batches to handle large numbers efficiently
        all_embeddings = []
        embedding_batch_size = 20  # OpenAI has limits on batch sizes

        for i in range(0, len(texts_to_embed), embedding_batch_size):
            batch_texts = texts_to_embed[i:i + embedding_batch_size]
            batch_embeddings = generate_embeddings(batch_texts)

            # Create embedding instances with proper metadata
            for idx, (chunk, vector) in enumerate(zip(all_chunks[i:i + embedding_batch_size], batch_embeddings)):
                embedding = create_embedding_instance(
                    chunk_id=chunk.id,
                    vector=vector,
                    source_url=chunk.source_url,
                    source_section=chunk.source_section
                )
                all_embeddings.append(embedding)

        # Validate embedding dimensions
        if not validate_embedding_dimensions([emb.vector for emb in all_embeddings]):
            add_error_to_job(job, "Embedding dimension validation failed")
            job.status = "failed"
            return job

        logging.info(f"Generated {len(all_embeddings)} embeddings")

        # Phase 4: Storage
        logging.info("Phase 4: Storing embeddings in Qdrant...")
        success = batch_store_embeddings_in_qdrant(
            all_embeddings,
            collection_name=collection_name,
            batch_size=batch_size
        )

        if not success:
            add_error_to_job(job, "Failed to store embeddings in Qdrant")
            job.status = "failed"
            return job

        logging.info("Successfully stored embeddings in Qdrant")

        # Phase 5: Verification
        logging.info("Phase 5: Verifying stored embeddings...")
        verification_success = run_embedding_verification_test()
        if not verification_success:
            add_error_to_job(job, "Embedding verification failed")
            job.status = "completed_with_warnings"
        else:
            job.status = "completed"
            logging.info("Pipeline completed successfully")

    except RAGException as e:
        error_msg = handle_error(e, "ingestion pipeline")
        add_error_to_job(job, error_msg)
        job.status = "failed"
        logging.error(f"Pipeline failed: {error_msg}")
    except Exception as e:
        error_msg = handle_error(e, "ingestion pipeline")
        add_error_to_job(job, error_msg)
        job.status = "failed"
        logging.error(f"Unexpected error in pipeline: {error_msg}")

    return job


def main():
    """Main function to run the RAG ingestion pipeline."""
    parser = argparse.ArgumentParser(description="RAG Website Embedding Pipeline")
    parser.add_argument("--website-url", type=str, help="Website URL to process (overrides config)")
    parser.add_argument("--chunk-size", type=int, default=512, help="Maximum tokens per content chunk")
    parser.add_argument("--collection-name", type=str, default="documentation_embeddings", help="Qdrant collection name")
    parser.add_argument("--batch-size", type=int, default=10, help="Batch size for processing")
    parser.add_argument("--log-level", type=str, default="INFO", help="Logging level")

    args = parser.parse_args()

    # Set up logging
    logging_config.setup_logging(args.log_level)

    # Validate configuration
    config_errors = config.Config.validate()
    if config_errors:
        logging.error("Configuration validation failed:")
        for error in config_errors:
            logging.error(f"  - {error}")
        sys.exit(1)

    # Use command line argument if provided, otherwise use config
    website_url = args.website_url or config.Config.WEBSITE_URL

    if not website_url:
        logging.error("No website URL provided. Please set WEBSITE_URL in config or use --website-url argument.")
        sys.exit(1)

    logging.info(f"Starting RAG ingestion pipeline for: {website_url}")

    # Run the pipeline
    start_time = time.time()
    job = run_ingestion_pipeline(
        website_url=website_url,
        max_tokens=args.chunk_size,
        collection_name=args.collection_name,
        batch_size=args.batch_size
    )
    end_time = time.time()

    # Report results
    logging.info(f"Pipeline completed with status: {job.status}")
    logging.info(f"Processing time: {end_time - start_time:.2f} seconds")
    logging.info(f"Pages processed: {job.pages_processed}/{job.total_pages}")

    if job.errors:
        logging.info(f"Errors encountered: {len(job.errors)}")
        for error in job.errors:
            logging.error(f"  - {error}")

    # Cleanup temporary resources
    cleanup_temp_resources()

    # Exit with appropriate code
    if job.status == "failed":
        sys.exit(1)
    elif job.status == "completed_with_warnings":
        logging.warning("Pipeline completed with warnings")
        sys.exit(0)  # Still consider it successful but with warnings
    else:
        logging.info("Pipeline completed successfully!")
        sys.exit(0)


if __name__ == "__main__":
    main()