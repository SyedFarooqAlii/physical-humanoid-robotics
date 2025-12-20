"""
Basic test to verify the RAG chatbot backend components are working
"""
import asyncio
import os
from app.config import settings
from app.ingestion.file_scanner import FileScanner
from app.ingestion.chunker import TextChunker, chunk_documents
from app.embeddings.minimal_embedding_generator import minimal_embedding_generator
from app.vector_store.qdrant_client import qdrant_client
from app.retrieval.retriever import retriever
from app.prompting.prompt_builder import prompt_builder
from app.generation.response_generator import response_generator


async def test_basic_functionality():
    """
    Test basic functionality of the RAG system
    """
    print("Testing RAG Chatbot Backend Components...")

    # Test 1: Configuration loading
    print("\n1. Testing configuration...")
    if settings.GEMINI_API_KEY and settings.GEMINI_API_KEY != "your_gemini_api_key_here":
        print("✓ Configuration loaded successfully")
    else:
        print("⚠ API key not configured (using placeholder)")

    # Test 2: Qdrant connection
    print("\n2. Testing Qdrant connection...")
    try:
        info = qdrant_client.get_collection_info()
        print(f"✓ Qdrant connection successful - Collection: {info['name']}, Points: {info['point_count']}")
    except Exception as e:
        print(f"✗ Qdrant connection failed: {e}")
        return False

    # Test 3: Embedding generation (basic check)
    print("\n3. Testing embedding generation...")
    try:
        test_text = "This is a test sentence for embedding generation."
        embedding = minimal_embedding_generator.encode_query(test_text)
        if embedding and len(embedding) > 0:
            print(f"✓ Embedding generation works - Vector size: {len(embedding)}")
        else:
            print("✗ Embedding generation returned empty results")
            return False
    except Exception as e:
        print(f"✗ Embedding generation failed: {e}")
        return False

    # Test 4: Prompt building
    print("\n4. Testing prompt building...")
    try:
        test_query = "What are the main concepts in humanoid robotics?"
        test_contexts = [{
            'id': 'test-1',
            'content': 'Humanoid robotics involves creating robots that resemble humans in form and behavior.',
            'title': 'Introduction to Humanoid Robotics',
            'chapter': 'Chapter 1',
            'section': '1.1',
            'page_reference': 'page_1.md',
            'score': 0.9
        }]

        global_prompt = prompt_builder.build_global_query_prompt(test_query, test_contexts)
        selection_prompt = prompt_builder.build_selection_based_prompt(
            test_query,
            "This is selected text about humanoid robotics",
            test_contexts
        )

        if len(global_prompt) > 100 and len(selection_prompt) > 100:
            print("✓ Prompt building works for both query types")
        else:
            print("✗ Prompt building returned unexpectedly short prompts")
            return False
    except Exception as e:
        print(f"✗ Prompt building failed: {e}")
        return False

    # Test 5: Response generation (without actually calling the API to save costs)
    print("\n5. Testing response generator setup...")
    try:
        if response_generator.openrouter_client:
            print("✓ Response generator initialized successfully")
        else:
            print("✗ Response generator not properly initialized")
            return False
    except Exception as e:
        print(f"✗ Response generator test failed: {e}")
        return False

    print("\n✓ All basic functionality tests passed!")
    print("\nBackend is ready for full RAG operations.")
    print("\nTo start the API server, run:")
    print("cd backend")
    print("pip install -r requirements.txt")
    print("uvicorn app.main:app --reload")

    return True


if __name__ == "__main__":
    success = asyncio.run(test_basic_functionality())
    if success:
        print("\n🎉 Backend implementation is complete and ready for use!")
    else:
        print("\n❌ Some tests failed. Please check the implementation.")