from pydantic_settings import BaseSettings
from typing import Optional
from functools import lru_cache


class Settings(BaseSettings):
    # OpenRouter API
    OPENROUTER_API_KEY: str
    OPENROUTER_BASE_URL: str = "https://openrouter.ai/api/v1"

    # Qdrant Vector Database
    QDRANT_URL: str
    QDRANT_API_KEY: str
    QDRANT_CLUSTER_ID: str

    # Neon PostgreSQL Database
    NEON_DATABASE_URL: str

    # Cohere API (if needed)
    COHERE_API_KEY: Optional[str] = None

    # Google Gemini API
    GEMINI_API_KEY: Optional[str] = None

    # Backend API
    BACKEND_API_KEY: str

    # Application settings
    DEBUG: bool = False
    LOG_LEVEL: str = "INFO"
    MAX_CONTENT_LENGTH: int = 5000
    RATE_LIMIT_REQUESTS: int = 100
    RATE_LIMIT_WINDOW: int = 60  # in seconds

    class Config:
        env_file = ".env"
        case_sensitive = False


@lru_cache()
def get_settings():
    return Settings()


settings = get_settings()