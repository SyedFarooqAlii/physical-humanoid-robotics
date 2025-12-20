from typing import Dict, Any, List, Optional
from datetime import datetime
import uuid

from app.retrieval.retriever import Retriever
from app.prompting.context_filter import ContextFilter
from app.generation.response_generator import ResponseGenerator
from app.database.repositories import ChatSessionRepository, ChatMessageRepository, QueryContextRepository
from app.database.database import get_db


class ChatService:
    """
    Service class that orchestrates the chat functionality
    """

    def __init__(self):
        self.retriever = Retriever()
        self.context_filter = ContextFilter()
        self.response_generator = ResponseGenerator()

    async def process_chat_message(
        self,
        session_id: str,
        message: str,
        query_type: str = "global",
        selected_text: Optional[str] = None,
        top_k: int = 5
    ) -> Dict[str, Any]:
        """
        Process a chat message through the full RAG pipeline
        """
        # Retrieve relevant documents
        retrieved_docs = await self.retriever.retrieve_with_context_filtering(
            query=message,
            top_k=top_k,
            query_type=query_type,
            selected_text=selected_text
        )

        # Apply context filtering to ensure proper isolation
        filtered_docs = self.context_filter.enforce_context_boundaries(
            contexts=retrieved_docs,
            query_type=query_type,
            selected_text=selected_text
        )

        # Generate response
        response_data = await self.response_generator.generate_response_with_validation(
            query=message,
            retrieved_contexts=filtered_docs,
            query_type=query_type,
            selected_text=selected_text,
            session_id=session_id
        )

        # Store conversation in database
        await self._store_conversation(session_id, message, response_data)

        return response_data

    async def _store_conversation(self, session_id: str, user_message: str, response_data: Dict[str, Any]):
        """
        Store the conversation in the database
        """
        db_gen = get_db()
        db = next(db_gen)
        try:
            # Create or update session
            session_repo = ChatSessionRepository(db)
            existing_session = session_repo.get_session_by_id(session_id)
            if not existing_session:
                session_repo.create_session(session_id=session_id)

            # Store user message
            user_message_id = f"msg_{uuid.uuid4().hex[:8]}"
            message_repo = ChatMessageRepository(db)
            message_repo.create_message(
                message_id=user_message_id,
                session_id=session_id,
                role="user",
                content=user_message
            )

            # Store assistant response
            assistant_message_id = f"msg_{uuid.uuid4().hex[:8]}"
            citations_for_storage = response_data.get("citations", [])
            message_repo.create_message(
                message_id=assistant_message_id,
                session_id=session_id,
                role="assistant",
                content=response_data.get("response", ""),
                citations=citations_for_storage
            )
        finally:
            next(db_gen, None)  # Close the db session

    async def get_chat_history(self, session_id: str) -> List[Dict[str, Any]]:
        """
        Retrieve chat history for a session
        """
        db_gen = get_db()
        db = next(db_gen)
        try:
            message_repo = ChatMessageRepository(db)
            messages = message_repo.get_messages_by_session(session_id)

            return [
                {
                    "message_id": msg.message_id,
                    "role": msg.role,
                    "content": msg.content,
                    "timestamp": msg.timestamp.isoformat() if msg.timestamp else None,
                    "citations": msg.citations
                }
                for msg in messages
            ]
        finally:
            next(db_gen, None)

    def validate_query_params(
        self,
        query_type: str,
        selected_text: Optional[str] = None
    ) -> Dict[str, Any]:
        """
        Validate query parameters
        """
        errors = []

        if query_type not in ["global", "selection"]:
            errors.append("query_type must be either 'global' or 'selection'")

        if query_type == "selection" and not selected_text:
            errors.append("selected_text is required for selection-based queries")

        return {
            "is_valid": len(errors) == 0,
            "errors": errors
        }


# Global instance
chat_service = ChatService()