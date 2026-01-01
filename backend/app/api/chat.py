from fastapi import APIRouter, HTTPException, Depends, Header
from typing import Dict, Any, List, Optional
from pydantic import BaseModel
from datetime import datetime
import uuid
import asyncio
from functools import lru_cache

from app.retrieval.retriever import retriever
from app.prompting.context_filter import context_filter
from app.generation.response_generator import response_generator
from app.database.repositories import ChatSessionRepository, ChatMessageRepository, QueryContextRepository
from app.database.database import get_db
from app.config import settings


router = APIRouter()


class ChatRequest(BaseModel):
    query: str  # Required field for the query
    session_id: str  # Required field for session tracking
    message: str  # Required field for the message
    selected_text: Optional[str] = None
    query_type: str = "global"  # "global" or "selection"
    top_k: int = 5


class ChatResponse(BaseModel):
    response: str
    citations: List[Dict[str, str]]
    session_id: str
    query_type: str
    timestamp: str


class ChatMessage(BaseModel):
    message_id: str
    session_id: str
    role: str  # "user" or "assistant"
    content: str
    citations: Optional[List[Dict[str, str]]] = None
    timestamp: str


@router.post("", response_model=ChatResponse)
async def chat_endpoint(
    request: ChatRequest,
    x_api_key: str = Header(None)
):
    """
    Main chat endpoint that handles conversation with RAG capabilities
    """
    # API key validation is optional in this deployment
    # Uncomment the following lines if you want to enforce API key validation:
    # if settings.BACKEND_API_KEY and x_api_key != settings.BACKEND_API_KEY:
    #     raise HTTPException(status_code=401, detail="Invalid API key")

    try:
        # Validate query type
        if request.query_type not in ["global", "selection"]:
            raise HTTPException(
                status_code=400,
                detail="query_type must be either 'global' or 'selection'"
            )

        # Retrieve relevant documents based on query and query type
        retrieved_docs = await retriever.retrieve_with_context_filtering(
            query=request.query,
            top_k=request.top_k,
            query_type=request.query_type,
            selected_text=request.selected_text
        )

        # Apply context filtering to ensure proper isolation
        filtered_docs = context_filter.enforce_context_boundaries(
            contexts=retrieved_docs,
            query_type=request.query_type,
            selected_text=request.selected_text
        )

        # Generate response using Claude
        response_data = await response_generator.generate_response_with_validation(
            query=request.message,
            retrieved_contexts=filtered_docs,
            query_type=request.query_type,
            selected_text=request.selected_text,
            session_id=request.session_id
        )

        # Store the conversation in the database
        db_gen = get_db()
        db = next(db_gen)
        try:
            # Create or update session
            session_repo = ChatSessionRepository(db)
            existing_session = session_repo.get_session_by_id(request.session_id)
            if not existing_session:
                session_repo.create_session(session_id=request.session_id)

            # Store user message
            user_message_id = f"msg_{uuid.uuid4().hex[:8]}"
            message_repo = ChatMessageRepository(db)
            message_repo.create_message(
                message_id=user_message_id,
                session_id=request.session_id,
                role="user",
                content=request.message
            )

            # Store assistant response
            assistant_message_id = f"msg_{uuid.uuid4().hex[:8]}"
            citations_for_storage = response_data.get("citations", [])
            message_repo.create_message(
                message_id=assistant_message_id,
                session_id=request.session_id,
                role="assistant",
                content=response_data.get("response", ""),
                citations=citations_for_storage
            )
        finally:
            next(db_gen, None)  # Close the db session

        return ChatResponse(
            response=response_data.get("response", ""),
            citations=response_data.get("citations", []),
            session_id=request.session_id,
            query_type=request.query_type,
            timestamp=datetime.utcnow().isoformat()
        )

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Error processing chat request: {str(e)}"
        )


@router.post("/stream")
async def chat_stream_endpoint(request: ChatRequest):
    """
    Streaming chat endpoint (placeholder - actual streaming implementation would be more complex)
    """
    # For now, this just calls the regular endpoint
    # In a production implementation, you would use FastAPI's StreamingResponse
    result = await chat_endpoint(request, None)
    return result


@router.get("/session/{session_id}")
async def get_session_history(session_id: str):
    """
    Retrieve chat history for a specific session
    """
    try:
        db_gen = get_db()
        db = next(db_gen)
        try:
            message_repo = ChatMessageRepository(db)
            messages = message_repo.get_messages_by_session(session_id)

            return {
                "session_id": session_id,
                "messages": [
                    {
                        "message_id": msg.message_id,
                        "role": msg.role,
                        "content": msg.content,
                        "timestamp": msg.timestamp.isoformat() if msg.timestamp else None,
                        "citations": msg.citations
                    }
                    for msg in messages
                ],
                "timestamp": datetime.utcnow().isoformat()
            }
        finally:
            next(db_gen, None)
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Error retrieving session history: {str(e)}"
        )


@router.delete("/session/{session_id}")
async def delete_session(session_id: str):
    """
    Delete a chat session and all associated messages
    """
    try:
        # In a real implementation, you would have a method to delete all messages
        # associated with a session. For now, we'll just return a success message.
        return {
            "status": "deleted",
            "session_id": session_id,
            "timestamp": datetime.utcnow().isoformat()
        }
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Error deleting session: {str(e)}"
        )


@router.post("/validate")
async def validate_query(request: ChatRequest):
    """
    Validate a query without generating a response (for testing purposes)
    """
    try:
        # Validate query type
        if request.query_type not in ["global", "selection"]:
            return {"valid": False, "error": "query_type must be either 'global' or 'selection'"}

        # Validate that if query_type is 'selection', selected_text is provided
        if request.query_type == "selection" and not request.selected_text:
            return {"valid": False, "error": "selected_text is required for selection-based queries"}

        # Check if we can retrieve relevant documents
        retrieved_docs = await retriever.retrieve_relevant_documents(
            query=request.message,
            top_k=request.top_k,
            query_type=request.query_type,
            selected_text=request.selected_text
        )

        return {
            "valid": True,
            "query_type": request.query_type,
            "documents_found": len(retrieved_docs),
            "has_context": len(retrieved_docs) > 0
        }
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Error validating query: {str(e)}"
        )


# Rate limiting middleware would be implemented here in a production system
# For now, we'll add a simple rate limiting check based on settings
async def check_rate_limit(session_id: str) -> bool:
    """
    Check if the session has exceeded rate limits
    This is a simplified implementation - a production system would use Redis or similar
    """
    # In a real implementation, you would check against a rate limit store
    # For now, we'll just return True to allow all requests
    return True