from typing import List, Optional, Dict, Any
from sqlalchemy.orm import Session
from sqlalchemy import and_, or_
from app.database.models import BookContentDocument, ChatSession, ChatMessage, QueryContext
from app.ingestion.chunker import TextChunk


class BookContentRepository:
    """
    Repository for managing book content documents
    """

    def __init__(self, db: Session):
        self.db = db

    def create_document(self, chunk: Dict[str, Any]) -> BookContentDocument:
        """
        Create a new book content document
        """
        db_document = BookContentDocument(
            document_id=chunk.get('id'),
            title=chunk.get('title', ''),
            content=chunk.get('content', ''),
            chapter=chunk.get('chapter', ''),
            section=chunk.get('section', ''),
            page_reference=chunk.get('page_reference', ''),
            embedding_vector=chunk.get('embedding'),
            token_count=chunk.get('token_count', 0)
        )
        self.db.add(db_document)
        self.db.commit()
        self.db.refresh(db_document)
        return db_document

    def get_document_by_id(self, document_id: str) -> Optional[BookContentDocument]:
        """
        Get a document by its ID
        """
        return self.db.query(BookContentDocument).filter(
            BookContentDocument.document_id == document_id
        ).first()

    def get_documents_by_chapter(self, chapter: str) -> List[BookContentDocument]:
        """
        Get all documents for a specific chapter
        """
        return self.db.query(BookContentDocument).filter(
            BookContentDocument.chapter == chapter
        ).all()

    def get_all_documents(self) -> List[BookContentDocument]:
        """
        Get all documents
        """
        return self.db.query(BookContentDocument).all()

    def update_document(self, document_id: str, **kwargs) -> Optional[BookContentDocument]:
        """
        Update a document
        """
        document = self.get_document_by_id(document_id)
        if document:
            for key, value in kwargs.items():
                setattr(document, key, value)
            self.db.commit()
            self.db.refresh(document)
        return document

    def delete_document(self, document_id: str) -> bool:
        """
        Delete a document
        """
        document = self.get_document_by_id(document_id)
        if document:
            self.db.delete(document)
            self.db.commit()
            return True
        return False


class ChatSessionRepository:
    """
    Repository for managing chat sessions
    """

    def __init__(self, db: Session):
        self.db = db

    def create_session(self, session_id: str, user_id: Optional[str] = None, metadata: Optional[Dict] = None) -> ChatSession:
        """
        Create a new chat session
        """
        db_session = ChatSession(
            session_id=session_id,
            user_id=user_id,
            session_metadata=metadata
        )
        self.db.add(db_session)
        self.db.commit()
        self.db.refresh(db_session)
        return db_session

    def get_session_by_id(self, session_id: str) -> Optional[ChatSession]:
        """
        Get a session by its ID
        """
        return self.db.query(ChatSession).filter(
            ChatSession.session_id == session_id
        ).first()

    def update_session(self, session_id: str, **kwargs) -> Optional[ChatSession]:
        """
        Update a session
        """
        session = self.get_session_by_id(session_id)
        if session:
            for key, value in kwargs.items():
                setattr(session, key, value)
            self.db.commit()
            self.db.refresh(session)
        return session


class ChatMessageRepository:
    """
    Repository for managing chat messages
    """

    def __init__(self, db: Session):
        self.db = db

    def create_message(
        self,
        message_id: str,
        session_id: str,
        role: str,
        content: str,
        citations: Optional[List[Dict]] = None,
        query_context_id: Optional[str] = None
    ) -> ChatMessage:
        """
        Create a new chat message
        """
        db_message = ChatMessage(
            message_id=message_id,
            session_id=session_id,
            role=role,
            content=content,
            citations=citations,
            query_context_id=query_context_id
        )
        self.db.add(db_message)
        self.db.commit()
        self.db.refresh(db_message)
        return db_message

    def get_messages_by_session(self, session_id: str) -> List[ChatMessage]:
        """
        Get all messages for a session
        """
        return self.db.query(ChatMessage).filter(
            ChatMessage.session_id == session_id
        ).order_by(ChatMessage.timestamp).all()

    def get_message_by_id(self, message_id: str) -> Optional[ChatMessage]:
        """
        Get a message by its ID
        """
        return self.db.query(ChatMessage).filter(
            ChatMessage.message_id == message_id
        ).first()


class QueryContextRepository:
    """
    Repository for managing query contexts
    """

    def __init__(self, db: Session):
        self.db = db

    def create_query_context(
        self,
        context_id: str,
        session_id: str,
        selected_text: Optional[str] = None,
        query_type: str = "global"
    ) -> QueryContext:
        """
        Create a new query context
        """
        db_context = QueryContext(
            context_id=context_id,
            session_id=session_id,
            selected_text=selected_text,
            query_type=query_type
        )
        self.db.add(db_context)
        self.db.commit()
        self.db.refresh(db_context)
        return db_context

    def get_context_by_id(self, context_id: str) -> Optional[QueryContext]:
        """
        Get a query context by its ID
        """
        return self.db.query(QueryContext).filter(
            QueryContext.context_id == context_id
        ).first()

    def get_contexts_by_session(self, session_id: str) -> List[QueryContext]:
        """
        Get all query contexts for a session
        """
        return self.db.query(QueryContext).filter(
            QueryContext.session_id == session_id
        ).all()