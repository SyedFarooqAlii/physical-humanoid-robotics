from fastapi import APIRouter, HTTPException, BackgroundTasks, UploadFile, File, Form
from typing import Dict, Any, List, Optional
from pydantic import BaseModel
from datetime import datetime
import asyncio
import os
from pathlib import Path

from app.ingestion.file_scanner import FileScanner
from app.ingestion.chunker import TextChunker, chunk_documents
from app.embeddings.minimal_embedding_generator import minimal_embedding_generator
from app.vector_store.vector_repository import vector_repository
from app.database.repositories import BookContentRepository
from app.database.database import get_db
from app.config import settings


router = APIRouter()

# Initialize components
chunker = TextChunker()
file_scanner = FileScanner()


class IngestRequest(BaseModel):
    book_path: str = "docusaurus/docs"  # Default path for Docusaurus docs
    chunk_size: int = 800
    force_reprocess: bool = False


class IngestResponse(BaseModel):
    status: str
    message: str
    documents_processed: int
    chunks_created: int
    embeddings_generated: int
    timestamp: str


class IngestStatusResponse(BaseModel):
    status: str
    progress: float
    message: str
    details: Dict[str, Any]


# In-memory storage for tracking ingestion jobs (in production, use a proper task queue)
ingestion_jobs: Dict[str, Dict[str, Any]] = {}


@router.post("/ingest", response_model=IngestResponse)
async def ingest_documents(request: IngestRequest, background_tasks: BackgroundTasks):
    """
    Ingest book content from markdown files, process, and store in vector database
    """
    try:
        # Validate the book path exists
        if not os.path.exists(request.book_path):
            raise HTTPException(
                status_code=400,
                detail=f"Book path does not exist: {request.book_path}"
            )

        # Update chunker settings if provided
        if request.chunk_size:
            chunker.max_tokens = request.chunk_size

        # Scan and parse documents
        documents = file_scanner.scan_and_parse_documents()

        if not documents:
            raise HTTPException(
                status_code=400,
                detail=f"No markdown documents found in path: {request.book_path}"
            )

        # Validate documents
        valid_documents = [doc for doc in documents if file_scanner.validate_document(doc)]
        if not valid_documents:
            raise HTTPException(
                status_code=400,
                detail="No valid documents found after validation"
            )

        # Chunk documents
        all_chunks = chunk_documents(valid_documents)

        if not all_chunks:
            raise HTTPException(
                status_code=500,
                detail="No chunks were created from documents"
            )

        # Generate embeddings for chunks
        chunks_with_metadata = []
        for chunk in all_chunks:
            # Generate embedding using minimal embedding generator
            embedding = minimal_embedding_generator.encode_query(chunk.content)
            if embedding:
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
            else:
                # Skip chunks that couldn't generate embeddings
                continue

        # Store embeddings in vector database
        if chunks_with_metadata:
            vector_repository.store_document_chunks(chunks_with_metadata)

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
        finally:
            next(db_gen, None)  # Close the db session

        return IngestResponse(
            status="success",
            message=f"Successfully ingested {len(valid_documents)} documents, "
                   f"created {len(all_chunks)} chunks, "
                   f"generated {len(chunks_with_metadata)} embeddings",
            documents_processed=len(valid_documents),
            chunks_created=len(all_chunks),
            embeddings_generated=len(chunks_with_metadata),
            timestamp=datetime.utcnow().isoformat()
        )

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Error during ingestion: {str(e)}"
        )


@router.post("/ingest-from-content")
async def ingest_from_content(content: str = Form(...), title: str = Form(...), chapter: str = Form("Unknown"), section: str = Form("Unknown")):
    """
    Ingest content directly from provided text
    """
    try:
        # Create a mock document from the provided content
        document = {
            'title': title,
            'content': content,
            'chapter': chapter,
            'section': section,
            'file_path': f"api_upload_{datetime.utcnow().strftime('%Y%m%d_%H%M%S')}",
            'metadata': {},
            'structure': []
        }

        # Validate document
        if not document['content'].strip():
            raise HTTPException(status_code=400, detail="Content cannot be empty")

        # Chunk the document
        all_chunks = chunk_documents([document])

        if not all_chunks:
            raise HTTPException(status_code=500, detail="No chunks were created from content")

        # Generate embeddings for chunks
        chunks_with_metadata = []
        for chunk in all_chunks:
            # Generate embedding using minimal embedding generator
            embedding = minimal_embedding_generator.encode_query(chunk.content)
            if embedding:
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
            else:
                # Skip chunks that couldn't generate embeddings
                continue

        # Store embeddings in vector database
        if chunks_with_metadata:
            vector_repository.store_document_chunks(chunks_with_metadata)

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
        finally:
            next(db_gen, None)  # Close the db session

        return {
            "status": "success",
            "message": f"Successfully ingested content, created {len(all_chunks)} chunks",
            "chunks_created": len(all_chunks),
            "timestamp": datetime.utcnow().isoformat()
        }

    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Error during content ingestion: {str(e)}"
        )


@router.post("/ingest-file")
async def ingest_from_file(file: UploadFile = File(...), title: str = Form(None), chapter: str = Form("Unknown")):
    """
    Ingest content from an uploaded file
    """
    try:
        # Read the uploaded file
        content = await file.read()
        content_str = content.decode('utf-8')

        # Use filename as title if not provided
        if not title:
            title = Path(file.filename).stem

        # Ingest the content
        return await ingest_from_content(
            content=content_str,
            title=title,
            chapter=chapter,
            section=Path(file.filename).stem
        )

    except UnicodeDecodeError:
        raise HTTPException(
            status_code=400,
            detail="File must be a UTF-8 encoded text file"
        )
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Error processing uploaded file: {str(e)}"
        )


@router.get("/ingest-status/{job_id}")
async def get_ingest_status(job_id: str):
    """
    Get the status of an ingestion job
    """
    if job_id not in ingestion_jobs:
        raise HTTPException(status_code=404, detail="Job not found")

    return IngestStatusResponse(
        status=ingestion_jobs[job_id]["status"],
        progress=ingestion_jobs[job_id]["progress"],
        message=ingestion_jobs[job_id]["message"],
        details=ingestion_jobs[job_id]["details"]
    )


@router.get("/ingest-stats")
async def get_ingest_stats():
    """
    Get ingestion statistics
    """
    # Get vector store stats
    vector_stats = vector_repository.get_collection_stats()

    # Get database stats
    db_gen = get_db()
    db = next(db_gen)
    try:
        content_repo = BookContentRepository(db)
        all_docs = content_repo.get_all_documents()
        total_docs = len(all_docs)

        # Group by chapter
        chapters = {}
        for doc in all_docs:
            chapter = doc.chapter
            if chapter not in chapters:
                chapters[chapter] = 0
            chapters[chapter] += 1
    finally:
        next(db_gen, None)

    return {
        "vector_store": vector_stats,
        "database": {
            "total_documents": total_docs,
            "documents_by_chapter": chapters
        },
        "timestamp": datetime.utcnow().isoformat()
    }