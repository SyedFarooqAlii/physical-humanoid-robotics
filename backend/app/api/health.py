from fastapi import APIRouter, HTTPException, Depends
from typing import Dict, Any
from datetime import datetime
import time
import asyncio

from app.config import settings
from app.vector_store.qdrant_client import qdrant_client
from app.database.models import create_tables, get_db
from app.database.database import get_db
from sqlalchemy.orm import Session


router = APIRouter()


@router.get("/health", response_model=Dict[str, Any])
async def health_check():
    """
    Health check endpoint that verifies the system is operational
    """
    # Basic system status
    health_status = {
        "status": "healthy",
        "timestamp": datetime.utcnow().isoformat(),
        "service": "RAG Chatbot API",
        "version": "1.0.0",
        "checks": {
            "database": {"status": "unknown", "message": ""},
            "vector_store": {"status": "unknown", "message": ""},
            "api_config": {"status": "ok", "message": "API configuration loaded"},
            "external_services": {"status": "unknown", "message": ""}
        }
    }

    # Check database connection
    try:
        # Try to access the database
        from app.database.database import engine
        with engine.connect() as conn:
            # Simple query to test connection using SQLAlchemy 2.0 syntax
            from sqlalchemy import text
            result = conn.execute(text("SELECT 1"))
            health_status["checks"]["database"]["status"] = "ok"
            health_status["checks"]["database"]["message"] = "Database connection successful"
    except Exception as e:
        health_status["checks"]["database"]["status"] = "error"
        health_status["checks"]["database"]["message"] = f"Database connection failed: {str(e)}"

    # Check vector store connection
    try:
        # Try to get collection info as a simple connectivity test
        collection_info = qdrant_client.get_collection_info()
        health_status["checks"]["vector_store"]["status"] = "ok"
        health_status["checks"]["vector_store"]["message"] = f"Vector store connected, {collection_info.get('point_count', 0)} points"
    except Exception as e:
        health_status["checks"]["vector_store"]["status"] = "error"
        health_status["checks"]["vector_store"]["message"] = f"Vector store connection failed: {str(e)}"

    # Check external services (API keys)
    try:
        if not settings.GEMINI_API_KEY or settings.GEMINI_API_KEY == "your_gemini_api_key_here":
            health_status["checks"]["external_services"]["status"] = "warning"
            health_status["checks"]["external_services"]["message"] = "API key not configured or using default value"
        else:
            health_status["checks"]["external_services"]["status"] = "ok"
            health_status["checks"]["external_services"]["message"] = "External services configured"
    except Exception as e:
        health_status["checks"]["external_services"]["status"] = "error"
        health_status["checks"]["external_services"]["message"] = f"External service config error: {str(e)}"

    # Overall status based on individual checks
    overall_status = "healthy"
    for check_name, check in health_status["checks"].items():
        if check["status"] == "error":
            overall_status = "error"
            break
        elif check["status"] == "warning" and overall_status != "error":
            overall_status = "warning"

    health_status["status"] = overall_status

    return health_status


@router.get("/ready", response_model=Dict[str, str])
async def readiness_check():
    """
    Readiness check - verifies the service is ready to accept traffic
    """
    # For readiness, we mainly check if critical services are available
    try:
        # Test database connection
        from app.database.database import engine
        from sqlalchemy import text
        with engine.connect() as conn:
            conn.execute(text("SELECT 1"))

        # Test vector store connection
        qdrant_client.get_collection_info()

        return {"status": "ready"}
    except Exception:
        raise HTTPException(status_code=503, detail="Service not ready")


@router.get("/live", response_model=Dict[str, str])
async def liveness_check():
    """
    Liveness check - verifies the service is alive and responding
    """
    return {"status": "alive", "timestamp": datetime.utcnow().isoformat()}


@router.get("/stats", response_model=Dict[str, Any])
async def get_service_stats():
    """
    Get detailed service statistics
    """
    stats = {
        "timestamp": datetime.utcnow().isoformat(),
        "uptime": "tracking needed",  # This would need to be implemented with a global start time
        "database": {},
        "vector_store": {},
        "api_usage": {}
    }

    # Database stats
    try:
        from app.database.database import engine
        with engine.connect() as conn:
            # Get basic database info using SQLAlchemy 2.0 syntax
            from sqlalchemy import text
            result = conn.execute(text("SELECT COUNT(*) FROM book_content_documents"))
            doc_count = result.scalar()
            stats["database"] = {
                "documents_count": doc_count,
                "status": "connected"
            }
    except Exception as e:
        stats["database"] = {"status": "error", "error": str(e)}

    # Vector store stats
    try:
        collection_info = qdrant_client.get_collection_info()
        stats["vector_store"] = {
            "point_count": collection_info.get('point_count', 0),
            "vector_size": collection_info.get('vector_size', 0),
            "status": "connected"
        }
    except Exception as e:
        stats["vector_store"] = {"status": "error", "error": str(e)}

    # API usage would be tracked separately in a production system
    stats["api_usage"] = {
        "requests_served": 0,  # This would be implemented with actual tracking
        "avg_response_time": 0.0  # This would be calculated from actual metrics
    }

    return stats