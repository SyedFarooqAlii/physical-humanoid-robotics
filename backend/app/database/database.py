from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from app.config import settings


# Create database engine with connection pooling
engine = create_engine(
    settings.NEON_DATABASE_URL,
    pool_pre_ping=True,  # Verify connections before use
    pool_recycle=300,    # Recycle connections after 5 minutes
    pool_size=10,        # Number of connection to keep open
    max_overflow=20,     # Additional connections beyond pool_size
    echo=False           # Set to True for SQL query logging
)

# Create session factory
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

# Base class for models
Base = declarative_base()


def get_db():
    """
    Dependency for getting database session
    """
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()


def create_tables():
    """
    Create all database tables based on models
    This should be called during application startup
    """
    from .models import Base
    Base.metadata.create_all(bind=engine)


def get_engine():
    """
    Return the database engine (useful for direct access)
    """
    return engine


# Global database instance for easy access
db_instance = {
    'engine': engine,
    'SessionLocal': SessionLocal,
    'Base': Base
}