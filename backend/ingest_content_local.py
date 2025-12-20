"""
Script to ingest the Physical AI & Humanoid Robotics book content into the vector database
using local embeddings to avoid API costs
"""
import asyncio
import os
import sys
from pathlib import Path

# Add the app directory to the path so we can import modules
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from app.api.ingest import IngestRequest
from app.config import settings


async def run_ingestion():
    """
    Run the document ingestion process using local embeddings
    """
    print("Starting book content ingestion with local embeddings...")
    print(f"Source: docusaurus/docs/")
    print(f"Qdrant Collection: book_content_chunks")
    print(f"Qdrant Connection: {settings.QDRANT_URL}")

    # Create the ingestion request
    request = IngestRequest(
        book_path="../docusaurus/docs",  # Path to the Docusaurus book content
        chunk_size=800,  # 800 token chunks
        force_reprocess=False
    )

    try:
        print("\nScanning documentation directory...")

        # Check if the book path exists
        if not os.path.exists(request.book_path):
            print(f"Error: Book path does not exist: {request.book_path}")
            return False

        print(f"Found book path: {request.book_path}")

        # Import the required modules for ingestion
        from app.ingestion.file_scanner import FileScanner
        from app.ingestion.chunker import chunk_documents
        from app.embeddings.minimal_embedding_generator import minimal_embedding_generator
        from app.vector_store.vector_repository import vector_repository
        from app.database.repositories import BookContentRepository
        from app.database.database import get_db

        # Initialize components
        file_scanner = FileScanner(request.book_path)

        print("Scanning markdown files...")

        # Scan and parse documents
        documents = file_scanner.scan_and_parse_documents()
        print(f"Found {len(documents)} documents")

        if not documents:
            print("No documents found to process")
            return False

        # Validate documents
        valid_documents = [doc for doc in documents if file_scanner.validate_document(doc)]
        print(f"Valid documents: {len(valid_documents)}")

        if not valid_documents:
            print("No valid documents after validation")
            return False

        # Show document stats
        stats = file_scanner.get_document_stats(valid_documents)
        print(f"Document Statistics:")
        print(f"   - Total documents: {stats['total_documents']}")
        print(f"   - Total characters: {stats['total_characters']:,}")
        print(f"   - Unique chapters: {stats['unique_chapters']}")
        print(f"   - Average length: {stats['average_length']:,} chars")

        print("\nChunking documents...")

        # Chunk documents
        all_chunks = chunk_documents(valid_documents)
        print(f"Created {len(all_chunks)} chunks")

        if not all_chunks:
            print("No chunks were created")
            return False

        print("\nGenerating local embeddings...")

        # Generate embeddings for chunks using simple model
        chunks_with_metadata = []
        processed_count = 0
        total_chunks = len(all_chunks)

        # Process in batches to manage memory
        batch_size = 10
        for i in range(0, len(all_chunks), batch_size):
            batch = all_chunks[i:i + batch_size]
            batch_texts = [chunk.content for chunk in batch]

            print(f"   Processing batch {i//batch_size + 1}/{(len(all_chunks)-1)//batch_size + 1}...")

            # Generate embeddings using minimal approach
            batch_embeddings = minimal_embedding_generator.generate_embeddings(batch_texts)

            for j, chunk in enumerate(batch):
                embedding = batch_embeddings[j]
                chunk_data = {
                    'id': chunk.id,
                    'content': chunk.content,
                    'title': chunk.title,
                    'chapter': chunk.chapter,
                    'section': chunk.section,
                    'page_reference': chunk.page_reference,
                    'token_count': chunk.token_count,
                    'embedding': embedding
                }
                chunks_with_metadata.append(chunk_data)
                processed_count += 1

            print(f"   Processed {processed_count}/{total_chunks} chunks")

        print(f"\nGenerated embeddings for {len(chunks_with_metadata)} chunks")

        if not chunks_with_metadata:
            print("No chunks with embeddings to store")
            return False

        print("\nStoring embeddings in Qdrant...")

        # Store embeddings in vector database
        vector_repository.store_document_chunks(chunks_with_metadata)
        print(f"Stored {len(chunks_with_metadata)} embeddings in Qdrant")

        print("\nStoring metadata in PostgreSQL...")

        # Store document metadata in SQL database
        db_gen = get_db()
        db = next(db_gen)
        try:
            content_repo = BookContentRepository(db)

            for chunk_data in chunks_with_metadata:
                # Create or update document in SQL database
                existing_doc = content_repo.get_document_by_id(chunk_data['id'])
                if not existing_doc:
                    content_repo.create_document(chunk_data)

            print(f"Stored {len(chunks_with_metadata)} document metadata entries")
        finally:
            next(db_gen, None)  # Close the db session

        print(f"\nIngestion completed successfully!")
        print(f"Summary:")
        print(f"   - Documents processed: {len(valid_documents)}")
        print(f"   - Chunks created: {len(all_chunks)}")
        print(f"   - Embeddings stored: {len(chunks_with_metadata)}")
        print(f"   - Content ready for RAG queries!")

        # Get final collection stats
        final_stats = vector_repository.get_collection_stats()
        print(f"\nFinal Qdrant Collection Stats:")
        print(f"   - Total vectors: {final_stats.get('point_count', 0)}")
        print(f"   - Vector size: {final_stats.get('vector_size', 'unknown')}")

        return True

    except Exception as e:
        print(f"\nError during ingestion: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    print("RAG Chatbot Content Ingestion Tool (Local Embeddings)")
    print("=" * 50)

    success = asyncio.run(run_ingestion())

    if success:
        print("\nBook content successfully ingested into your Qdrant cluster!")
        print("You can now start asking questions about the Physical AI & Humanoid Robotics book")
        print("The RAG chatbot is ready to use with real book content")
    else:
        print("\nContent ingestion failed. Please check the error messages above.")
        sys.exit(1)