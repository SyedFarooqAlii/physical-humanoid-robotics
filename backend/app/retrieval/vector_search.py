from typing import List, Dict, Any, Optional
from app.vector_store.qdrant_client import QdrantVectorStore
from app.embeddings.minimal_embedding_generator import minimal_embedding_generator
from app.config import settings


class VectorSearchEngine:
    """
    Core vector search engine that handles semantic search operations
    """

    def __init__(self):
        self.qdrant_client = QdrantVectorStore()
        self.top_k_default = 5

    async def search_with_query(self, query: str, top_k: int = 5, filters: Optional[Dict] = None) -> List[Dict[str, Any]]:
        """
        Perform semantic search using a query string
        """
        # Generate embedding for the query using minimal generator
        query_embedding = minimal_embedding_generator.encode_query(query)

        if not query_embedding:
            return []

        # Perform vector search in Qdrant
        chapter_filter = filters.get('chapter') if filters else None
        search_results = self.qdrant_client.search_similar(
            query_embedding=query_embedding,
            top_k=top_k,
            chapter_filter=chapter_filter
        )

        return search_results

    async def search_with_embedding(self, query_embedding: List[float], top_k: int = 5, filters: Optional[Dict] = None) -> List[Dict[str, Any]]:
        """
        Perform semantic search using a pre-computed embedding
        """
        chapter_filter = filters.get('chapter') if filters else None
        search_results = self.qdrant_client.search_similar(
            query_embedding=query_embedding,
            top_k=top_k,
            chapter_filter=chapter_filter
        )

        return search_results

    def rank_results_by_relevance(self, results: List[Dict[str, Any]], query: str) -> List[Dict[str, Any]]:
        """
        Apply additional ranking based on relevance to the query
        This is a simple implementation; in production, you might want to use more sophisticated ranking
        """
        # For now, we'll just return the results as Qdrant already ranks by similarity score
        # In the future, we could implement additional ranking based on:
        # - keyword matching in title/content
        # - recency of content
        # - content length relative to query needs
        return sorted(results, key=lambda x: x.get('score', 0), reverse=True)

    def filter_results(self, results: List[Dict[str, Any]], filters: Optional[Dict] = None) -> List[Dict[str, Any]]:
        """
        Apply additional filtering to search results
        """
        if not filters:
            return results

        filtered_results = []
        for result in results:
            include = True

            # Apply content-based filters
            if 'min_score' in filters:
                if result.get('score', 0) < filters['min_score']:
                    include = False

            if 'required_keywords' in filters:
                content = result.get('content', '').lower()
                for keyword in filters['required_keywords']:
                    if keyword.lower() not in content:
                        include = False
                        break

            if include:
                filtered_results.append(result)

        return filtered_results

    async def get_document_content(self, doc_id: str) -> Optional[Dict[str, Any]]:
        """
        Retrieve content of a specific document by ID
        """
        return self.qdrant_client.get_document_by_id(doc_id)

    def get_collection_stats(self) -> Dict[str, Any]:
        """
        Get statistics about the vector collection
        """
        return self.qdrant_client.get_collection_info()


# Global instance
vector_search_engine = VectorSearchEngine()