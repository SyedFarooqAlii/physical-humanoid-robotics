import os
import json
import logging
from typing import Dict, List, Any, Optional
from dotenv import load_dotenv
import asyncio
import time

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Import your existing RAG components from your project
from app.retrieval.retriever import Retriever
from app.generation.response_generator import ResponseGenerator
from app.config import settings


def retrieve_information(query: str, top_k: int = 5, query_type: str = "global", selected_text: Optional[str] = None) -> Dict:
    """
    Retrieve information from the knowledge base based on a query using your project's retriever
    """
    try:
        # Initialize the retriever
        retriever = Retriever()

        # Call the existing retrieve method from your RAG system
        retrieved_docs = asyncio.run(
            retriever.retrieve_with_context_filtering(
                query=query,
                top_k=top_k,
                query_type=query_type,
                selected_text=selected_text
            )
        )

        # Format the results for the assistant
        formatted_results = []
        for doc in retrieved_docs:
            formatted_results.append({
                'content': doc.get('content', ''),
                'document_id': doc.get('id', ''),
                'title': doc.get('title', ''),
                'chapter': doc.get('chapter', ''),
                'section': doc.get('section', ''),
                'page_reference': doc.get('page_reference', ''),
                'similarity_score': doc.get('score', 0.0)
            })

        return {
            'query': query,
            'retrieved_chunks': formatted_results,
            'total_results': len(formatted_results),
            'query_type': query_type
        }
    except Exception as e:
        logger.error(f"Error in retrieve_information: {e}")
        return {
            'query': query,
            'retrieved_chunks': [],
            'total_results': 0,
            'error': str(e),
            'query_type': query_type
        }


class RAGAgent:
    def __init__(self):
        # Initialize with your existing project components
        self.response_generator = ResponseGenerator()
        self.retriever = Retriever()

        logger.info("RAG Agent initialized with existing project components")

    def query_agent(self, query_text: str, session_id: Optional[str] = None, query_type: str = "global", selected_text: Optional[str] = None) -> Dict:
        """
        Process a query through the RAG agent and return structured response using your project's components
        """
        start_time = time.time()

        logger.info(f"Processing query through RAG agent: '{query_text[:50]}...'")

        try:
            # Retrieve relevant documents using your project's retriever
            retrieval_result = retrieve_information(query_text, top_k=5, query_type=query_type, selected_text=selected_text)

            if retrieval_result.get('error'):
                return {
                    "answer": "Sorry, I encountered an error retrieving information.",
                    "sources": [],
                    "matched_chunks": [],
                    "error": retrieval_result['error'],
                    "query_time_ms": (time.time() - start_time) * 1000,
                    "session_id": session_id,
                    "query_type": query_type
                }

            # Generate response using your project's response generator
            response_data = asyncio.run(
                self.response_generator.generate_response_with_validation(
                    query=query_text,
                    retrieved_contexts=retrieval_result['retrieved_chunks'],
                    query_type=query_type,
                    selected_text=selected_text,
                    session_id=session_id
                )
            )

            # Calculate query time
            query_time_ms = (time.time() - start_time) * 1000

            # Format the response
            response = {
                "answer": response_data.get("response", ""),
                "sources": [citation.get("title", "") for citation in response_data.get("citations", []) if citation.get("title")],
                "matched_chunks": retrieval_result['retrieved_chunks'],
                "citations": response_data.get("citations", []),
                "query_time_ms": query_time_ms,
                "session_id": session_id,
                "query_type": query_type,
                "confidence": self._calculate_confidence(retrieval_result['retrieved_chunks']),
                "error": response_data.get("error")
            }

            logger.info(f"Query processed in {query_time_ms:.2f}ms")
            return response

        except Exception as e:
            logger.error(f"Error processing query: {e}")
            return {
                "answer": "Sorry, I encountered an error processing your request.",
                "sources": [],
                "matched_chunks": [],
                "citations": [],
                "error": str(e),
                "query_time_ms": (time.time() - start_time) * 1000,
                "session_id": session_id,
                "query_type": query_type
            }

    def _calculate_confidence(self, matched_chunks: List[Dict]) -> str:
        """
        Calculate confidence level based on similarity scores and number of matches using your project's logic
        """
        if not matched_chunks:
            return "low"

        # Calculate average similarity score
        scores = [chunk.get('similarity_score', 0.0) for chunk in matched_chunks if chunk.get('similarity_score') is not None]
        if not scores:
            return "low"

        avg_score = sum(scores) / len(scores)

        if avg_score >= 0.7:
            return "high"
        elif avg_score >= 0.4:
            return "medium"
        else:
            return "low"

    async def query_agent_async(self, query_text: str, session_id: Optional[str] = None, query_type: str = "global", selected_text: Optional[str] = None) -> Dict:
        """
        Async version of the query agent method
        """
        return self.query_agent(query_text, session_id, query_type, selected_text)


def query_agent(query_text: str, session_id: Optional[str] = None, query_type: str = "global", selected_text: Optional[str] = None) -> Dict:
    """
    Convenience function to query the RAG agent using your project's components
    """
    agent = RAGAgent()
    return agent.query_agent(query_text, session_id, query_type, selected_text)


def run_agent_sync(query_text: str, session_id: Optional[str] = None, query_type: str = "global", selected_text: Optional[str] = None) -> Dict:
    """
    Synchronous function to run the agent for direct usage with your project's components
    """
    agent = RAGAgent()
    return agent.query_agent(query_text, session_id, query_type, selected_text)


def main():
    """
    Main function to demonstrate the RAG agent functionality using your project's components
    """
    logger.info("Initializing RAG Agent with project components...")

    # Initialize the agent
    agent = RAGAgent()

    # Example queries to test the system
    test_queries = [
        "What is ROS2?",
        "Explain humanoid design principles",
        "How does VLA work?",
        "What are simulation techniques?",
        "Explain AI control systems"
    ]

    print("RAG Agent - Testing Queries")
    print("=" * 50)

    for i, query in enumerate(test_queries, 1):
        print(f"\nQuery {i}: {query}")
        print("-" * 30)

        # Process query through agent using your project's components
        response = agent.query_agent(query, session_id=f"test-session-{i}")

        # Print formatted results
        print(f"Answer: {response['answer']}")

        if response.get('sources'):
            print(f"Sources: {len(response['sources'])} documents")
            for source in response['sources'][:3]:  # Show first 3 sources
                print(f"  - {source}")

        if response.get('matched_chunks'):
            print(f"Matched chunks: {len(response['matched_chunks'])}")
            for j, chunk in enumerate(response['matched_chunks'][:2], 1):  # Show first 2 chunks
                content_preview = chunk['content'][:100] + "..." if len(chunk['content']) > 100 else chunk['content']
                print(f"  Chunk {j}: {content_preview}")
                print(f"    Source: {chunk.get('title', 'Unknown')}")
                print(f"    Score: {chunk.get('similarity_score', 0.0):.3f}")

        print(f"Query time: {response['query_time_ms']:.2f}ms")
        print(f"Confidence: {response.get('confidence', 'unknown')}")

        if i < len(test_queries):  # Don't sleep after the last query
            time.sleep(1)  # Small delay between queries


if __name__ == "__main__":
    main()