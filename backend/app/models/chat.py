from pydantic import BaseModel
from typing import List, Dict, Optional
from datetime import datetime


class ChatSession(BaseModel):
    """
    Model for chat session data
    """
    session_id: str
    user_id: Optional[str] = None
    created_at: datetime
    updated_at: datetime
    metadata: Optional[Dict] = None


class ChatMessage(BaseModel):
    """
    Model for chat message data
    """
    message_id: str
    session_id: str
    role: str  # "user" or "assistant"
    content: str
    citations: Optional[List[Dict[str, str]]] = None
    query_context_id: Optional[str] = None
    timestamp: datetime


class QueryContext(BaseModel):
    """
    Model for query context data
    """
    context_id: str
    session_id: str
    selected_text: Optional[str] = None
    query_type: str  # "global" or "selection"
    created_at: datetime


class ChatRequest(BaseModel):
    """
    Model for chat API request
    """
    session_id: str
    message: str
    selected_text: Optional[str] = None
    query_type: str = "global"  # "global" or "selection"
    top_k: int = 5


class ChatResponse(BaseModel):
    """
    Model for chat API response
    """
    response: str
    citations: List[Dict[str, str]]
    session_id: str
    query_type: str
    timestamp: str


class ChatHistoryResponse(BaseModel):
    """
    Model for chat history API response
    """
    session_id: str
    messages: List[ChatMessage]
    timestamp: str