from sqlalchemy import create_engine, Column, Integer, String, Text, DateTime, JSON, Index
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from datetime import datetime
import uuid
from app.config import settings


Base = declarative_base()


class BookContentDocument(Base):
    """
    Model for storing book content document metadata
    """
    __tablename__ = "book_content_documents"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    document_id = Column(String, unique=True, nullable=False, index=True)  # The ID from our chunking process
    title = Column(String, nullable=False)
    content = Column(Text, nullable=False)  # Content summary or the actual content
    chapter = Column(String, nullable=False)
    section = Column(String, nullable=False)
    page_reference = Column(String, nullable=True)
    embedding_vector = Column(JSON, nullable=True)  # Store as JSON for flexibility
    token_count = Column(Integer, nullable=True)
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow, nullable=False)

    # Indexes for better query performance
    # Note: Indexes are handled separately to avoid creation conflicts
    # __table_args__ = (
    #     Index('idx_chapter_section', 'chapter', 'section'),
    #     Index('idx_document_id', 'document_id'),
    # )


class ChatSession(Base):
    """
    Model for storing chat session information
    """
    __tablename__ = "chat_sessions"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    session_id = Column(String, unique=True, nullable=False, index=True)
    user_id = Column(String, nullable=True)  # Optional user identifier
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow, nullable=False)
    session_metadata = Column(JSON, nullable=True)  # Additional session metadata

    # Note: Indexes are handled separately to avoid creation conflicts
    # __table_args__ = (
    #     Index('idx_session_id', 'session_id'),
    #     Index('idx_user_id', 'user_id'),
    # )


class ChatMessage(Base):
    """
    Model for storing individual chat messages
    """
    __tablename__ = "chat_messages"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    message_id = Column(String, unique=True, nullable=False, index=True)
    session_id = Column(String, nullable=False, index=True)  # References chat_sessions.session_id
    role = Column(String, nullable=False)  # 'user' or 'assistant'
    content = Column(Text, nullable=False)
    citations = Column(JSON, nullable=True)  # List of document IDs used in response
    query_context_id = Column(String, nullable=True, index=True)  # References query_context.context_id
    timestamp = Column(DateTime, default=datetime.utcnow, nullable=False)

    # Note: Indexes are handled separately to avoid creation conflicts
    # __table_args__ = (
    #     Index('idx_session_id', 'session_id'),
    #     Index('idx_message_id', 'message_id'),
    #     Index('idx_query_context_id', 'query_context_id'),
    # )


class QueryContext(Base):
    """
    Model for storing query context information
    """
    __tablename__ = "query_contexts"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    context_id = Column(String, unique=True, nullable=False, index=True)
    session_id = Column(String, nullable=False, index=True)  # References chat_sessions.session_id
    selected_text = Column(Text, nullable=True)  # Text selected by user (for selection-based queries)
    query_type = Column(String, nullable=False)  # 'global' or 'selection'
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)

    # Note: Indexes are handled separately to avoid creation conflicts
    # __table_args__ = (
    #     Index('idx_context_id', 'context_id'),
    #     Index('idx_session_id', 'session_id'),
    # )


# Create engine and session
engine = create_engine(settings.NEON_DATABASE_URL, pool_pre_ping=True)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)


def drop_tables():
    """
    Drop all database tables
    """
    # Drop all tables with CASCADE to handle foreign key dependencies
    # First drop any dependent objects, then drop tables
    from sqlalchemy import text
    with engine.connect() as conn:
        # Drop all tables in the schema with CASCADE
        conn.execute(text("DROP SCHEMA public CASCADE"))
        conn.execute(text("CREATE SCHEMA public"))
        conn.execute(text("GRANT ALL ON SCHEMA public TO public"))
        conn.execute(text("GRANT ALL ON SCHEMA public TO neondb_owner"))
        conn.commit()


def create_tables():
    """
    Create all database tables
    """
    # Create tables with checkfirst to avoid errors if they already exist
    Base.metadata.create_all(bind=engine, checkfirst=True)


def get_db():
    """
    Dependency for getting database session
    """
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()