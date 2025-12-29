"""
Script to run the RAG Chatbot API server
"""
import uvicorn
import os
from app.main import app


def run_server():
    """
    Run the FastAPI server
    """
    print("Starting RAG Chatbot API Server...")
    print("Serving Physical AI & Humanoid Robotics book content")
    print("Qdrant vector database connected")
    print("Neon PostgreSQL database connected")
    print("Google Gemini API configured")
    print("\nServer will be available at: http://localhost:8081")
    print("API documentation at: http://localhost:8081/docs")

    uvicorn.run(
        "app.main:app",
        host="0.0.0.0",
        port=8081,
        reload=True,  # Set to False in production
        log_level="info"
    )


if __name__ == "__main__":
    run_server()