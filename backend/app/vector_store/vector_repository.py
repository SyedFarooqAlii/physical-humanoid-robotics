from typing import List, Dict, Any, Optional
from app.vector_store.qdrant_client import QdrantVectorStore, qdrant_client
from app.ingestion.chunker import TextChunk


class VectorRepository:
    """
    Repository class for vector store operations
    """

    def __init__(self, vector_store: QdrantVectorStore):
        self.vector_store = vector_store

    def store_document_chunks(self, chunks_with_embeddings: List[Dict[str, Any]]):
        """
        Store document chunks with embeddings in the vector store
        """
        self.vector_store.store_embeddings(chunks_with_embeddings)

    def search_relevant_chunks(
        self,
        query_embedding: List[float],
        top_k: int = 5,
        chapter_filter: Optional[str] = None
    ) -> List[Dict[str, Any]]:
        """
        Search for relevant chunks based on query embedding
        """
        return self.vector_store.search_similar(
            query_embedding=query_embedding,
            top_k=top_k,
            chapter_filter=chapter_filter
        )

    def get_document_by_id(self, doc_id: str) -> Optional[Dict[str, Any]]:
        """
        Retrieve a document by its ID
        """
        return self.vector_store.get_document_by_id(doc_id)

    def get_collection_stats(self) -> Dict[str, Any]:
        """
        Get statistics about the vector collection
        """
        return self.vector_store.get_collection_info()


# Global instance
vector_repository = VectorRepository(qdrant_client)