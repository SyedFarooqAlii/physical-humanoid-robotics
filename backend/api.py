import os
import asyncio
from fastapi import FastAPI, HTTPException, Header
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional, Dict
from dotenv import load_dotenv
import logging

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Import the RAG agent functionality from your project
from agent import RAGAgent
from app.config import settings

# Create FastAPI app
app = FastAPI(
    title="Physical AI & Humanoid Robotics RAG API",
    description="API for RAG-based question answering for Physical AI & Humanoid Robotics book",
    version="1.0.0"
)

# Add CORS middleware for development
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Pydantic models
class QueryRequest(BaseModel):
    query: str
    message: str
    session_id: str
    selected_text: Optional[str] = None
    query_type: str = "global"
    top_k: int = 5

class MatchedChunk(BaseModel):
    content: str
    url: Optional[str] = None
    position: Optional[int] = None
    similarity_score: Optional[float] = None
    document_id: Optional[str] = None
    title: Optional[str] = None
    chapter: Optional[str] = None
    section: Optional[str] = None

class QueryResponse(BaseModel):
    response: str
    citations: List[Dict[str, str]]
    session_id: str
    query_type: str
    timestamp: str
    sources: List[str] = []
    matched_chunks: List[MatchedChunk] = []
    error: Optional[str] = None
    status: str  # "success", "error", "empty"
    query_time_ms: Optional[float] = None
    confidence: Optional[str] = None

class HealthResponse(BaseModel):
    status: str
    message: str

# Global RAG agent instance
rag_agent = None

@app.on_event("startup")
async def startup_event():
    """Initialize the RAG system on startup"""
    global rag_agent
    logger.info("Initializing RAG System...")
    try:
        rag_agent = RAGAgent()
        logger.info("RAG System initialized successfully")
    except Exception as e:
        logger.error(f"Failed to initialize RAG System: {e}")
        raise

@app.post("/api", response_model=QueryResponse)
async def chat_endpoint(
    request: QueryRequest,
    x_api_key: str = Header(None)
):
    """
    Main chat endpoint that handles conversation with RAG capabilities
    """
    logger.info(f"Processing query: {request.query[:50]}...")

    try:
        # Validate input
        if not request.query or len(request.query.strip()) == 0:
            raise HTTPException(status_code=400, detail="Query cannot be empty")

        if not request.session_id or len(request.session_id.strip()) == 0:
            raise HTTPException(status_code=400, detail="Session ID cannot be empty")

        if len(request.query) > 2000:
            raise HTTPException(status_code=400, detail="Query too long, maximum 2000 characters")

        # Optional API key validation using your project's existing settings
        if settings.BACKEND_API_KEY and x_api_key != settings.BACKEND_API_KEY:
            raise HTTPException(status_code=401, detail="Invalid API key")

        # Validate query type
        if request.query_type not in ["global", "selection"]:
            raise HTTPException(
                status_code=400,
                detail="query_type must be either 'global' or 'selection'"
            )

        # Process query through the global RAG agent
        response_data = rag_agent.query_agent(
            query_text=request.query,
            session_id=request.session_id,
            query_type=request.query_type,
            selected_text=request.selected_text
        )

        # Format response
        from datetime import datetime
        timestamp = datetime.utcnow().isoformat()

        formatted_response = QueryResponse(
            response=response_data.get("answer", ""),
            citations=response_data.get("citations", []),
            session_id=request.session_id,
            query_type=request.query_type,
            timestamp=timestamp,
            sources=response_data.get("sources", []),
            matched_chunks=[
                MatchedChunk(
                    content=chunk.get('content', ''),
                    document_id=chunk.get('document_id', ''),
                    title=chunk.get('title', ''),
                    chapter=chunk.get('chapter', ''),
                    section=chunk.get('section', ''),
                    similarity_score=chunk.get('similarity_score', 0.0)
                )
                for chunk in response_data.get("matched_chunks", [])
            ],
            error=response_data.get("error"),
            status="error" if response_data.get("error") else "success",
            query_time_ms=response_data.get("query_time_ms"),
            confidence=response_data.get("confidence")
        )

        logger.info(f"Query processed successfully for session: {request.session_id}")
        return formatted_response

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error processing query: {e}")
        from datetime import datetime
        return QueryResponse(
            response="",
            citations=[],
            session_id=request.session_id,
            query_type="global",
            timestamp=datetime.utcnow().isoformat(),
            sources=[],
            matched_chunks=[],
            error=str(e),
            status="error",
            query_time_ms=None,
            confidence=None
        )

@app.get("/health", response_model=HealthResponse)
async def health_check():
    """
    Health check endpoint
    """
    return HealthResponse(
        status="healthy",
        message="Physical AI & Humanoid Robotics RAG API is running"
    )

# For running with uvicorn
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=int(os.getenv("PORT", 8081)))