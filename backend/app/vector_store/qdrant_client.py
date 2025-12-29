from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from app.config import settings
from app.ingestion.chunker import TextChunk


class QdrantVectorStore:
    """
    Qdrant vector database client for storing and retrieving embeddings
    """

    def __init__(self):
        try:
            self.client = QdrantClient(
                url=settings.QDRANT_URL,
                api_key=settings.QDRANT_API_KEY,
                prefer_grpc=False,  # Using HTTP for better compatibility
                timeout=60.0  # Increase timeout for large batch operations
            )
            self.collection_name = "book_content_chunks"
            self.vector_size = 1536  # Standard embedding size for text-embedding-ada-002
            self._initialize_collection()
        except Exception as e:
            print(f"[WARN] Could not connect to Qdrant: {e}")
            print("[WARN] Qdrant functionality will be unavailable until connection is restored")
            # Initialize with None values when connection fails
            self.client = None
            self.collection_name = "book_content_chunks"
            self.vector_size = 1536

    def _initialize_collection(self):
        """
        Initialize the collection if it doesn't exist
        """
        if self.client is None:
            return  # Skip initialization if no client

        try:
            # Check if collection exists
            self.client.get_collection(self.collection_name)
            print(f"[INFO] Collection '{self.collection_name}' already exists")
        except:
            # Create collection if it doesn't exist
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=self.vector_size,
                    distance=models.Distance.COSINE
                )
            )
            print(f"[INFO] Created collection '{self.collection_name}'")

            # Create payload index for faster filtering
            self.client.create_payload_index(
                collection_name=self.collection_name,
                field_name="chapter",
                field_schema=models.PayloadSchemaType.KEYWORD
            )

            self.client.create_payload_index(
                collection_name=self.collection_name,
                field_name="section",
                field_schema=models.PayloadSchemaType.KEYWORD
            )

    def store_embeddings(self, chunks_with_embeddings: List[Dict[str, Any]]):
        """
        Store chunks with their embeddings in Qdrant
        """
        if self.client is None:
            print("[WARN] Cannot store embeddings - Qdrant not connected")
            return

        points = []
        for item in chunks_with_embeddings:
            point = models.PointStruct(
                id=item['id'],
                vector=item['embedding'],
                payload={
                    'content': item['content'],
                    'title': item['title'],
                    'chapter': item['chapter'],
                    'section': item['section'],
                    'page_reference': item['page_reference'],
                    'token_count': item['token_count']
                }
            )
            points.append(point)

        # Upload points in smaller batches to avoid timeouts
        batch_size = 16  # Smaller batch size to avoid timeouts
        for i in range(0, len(points), batch_size):
            batch = points[i:i + batch_size]
            self.client.upsert(
                collection_name=self.collection_name,
                points=batch
            )
            # Add a small delay between batches to avoid overwhelming the server
            import time
            time.sleep(0.1)

    def search_similar(self, query_embedding: List[float], top_k: int = 5, chapter_filter: Optional[str] = None) -> List[Dict[str, Any]]:
        """
        Search for similar content based on embedding similarity
        """
        if self.client is None:
            print("[WARN] Cannot search - Qdrant not connected")
            return []

        # Build filters if needed
        filters = None
        if chapter_filter:
            filters = models.Filter(
                must=[
                    models.FieldCondition(
                        key="chapter",
                        match=models.MatchValue(value=chapter_filter)
                    )
                ]
            )

        # Perform search
        search_results = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            query_filter=filters,
            limit=top_k,
            with_payload=True
        )

        # Format results
        results = []
        for result in search_results:
            results.append({
                'id': result.id,
                'content': result.payload['content'],
                'title': result.payload['title'],
                'chapter': result.payload['chapter'],
                'section': result.payload['section'],
                'page_reference': result.payload['page_reference'],
                'score': result.score
            })

        return results

    def get_document_by_id(self, doc_id: str) -> Optional[Dict[str, Any]]:
        """
        Retrieve a specific document by its ID
        """
        if self.client is None:
            print("[WARN] Cannot retrieve document - Qdrant not connected")
            return None

        points = self.client.retrieve(
            collection_name=self.collection_name,
            ids=[doc_id],
            with_payload=True
        )

        if points:
            point = points[0]
            return {
                'id': point.id,
                'content': point.payload['content'],
                'title': point.payload['title'],
                'chapter': point.payload['chapter'],
                'section': point.payload['section'],
                'page_reference': point.payload['page_reference']
            }

        return None

    def delete_collection(self):
        """
        Delete the entire collection (use with caution!)
        """
        if self.client is None:
            print("[WARN] Cannot delete collection - Qdrant not connected")
            return

        self.client.delete_collection(self.collection_name)

    def get_collection_info(self) -> Dict[str, Any]:
        """
        Get information about the collection
        """
        if self.client is None:
            print("[WARN] Cannot get collection info - Qdrant not connected")
            return {
                'name': self.collection_name,
                'vector_size': self.vector_size,
                'distance': 'COSINE',
                'point_count': 0
            }

        info = self.client.get_collection(self.collection_name)
        return {
            'name': self.collection_name,
            'vector_size': info.config.params.vectors.size,
            'distance': info.config.params.vectors.distance,
            'point_count': info.points_count
        }


# Global instance
qdrant_client = QdrantVectorStore()