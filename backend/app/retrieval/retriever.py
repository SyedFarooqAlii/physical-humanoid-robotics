from typing import List, Dict, Any, Optional
from app.retrieval.vector_search import VectorSearchEngine


class Retriever:
    """
    High-level retriever that handles the complete retrieval process
    """

    def __init__(self):
        self.vector_search = VectorSearchEngine()

    async def retrieve_relevant_documents(
        self,
        query: str,
        top_k: int = 5,
        query_type: str = "global",  # "global" or "selection"
        selected_text: Optional[str] = None,
        filters: Optional[Dict] = None
    ) -> List[Dict[str, Any]]:
        """
        Retrieve relevant documents based on the query and query type
        """
        if query_type == "selection" and selected_text:
            # For selection-based queries, we use the selected text as context
            # but still search for relevant content in the book
            # This approach focuses on content related to the selected text
            search_query = f"{selected_text} {query}".strip()
        else:
            # For global queries, we search with the original query
            search_query = query

        # Perform the search
        results = await self.vector_search.search_with_query(
            query=search_query,
            top_k=top_k,
            filters=filters
        )

        # Apply ranking and filtering
        ranked_results = self.vector_search.rank_results_by_relevance(results, query)
        filtered_results = self.vector_search.filter_results(ranked_results, filters)

        return filtered_results

    async def retrieve_with_context_filtering(
        self,
        query: str,
        top_k: int = 5,
        query_type: str = "global",
        selected_text: Optional[str] = None
    ) -> List[Dict[str, Any]]:
        """
        Retrieve documents with special handling for different query types
        """
        if query_type == "selection" and selected_text:
            # For selection-based queries, we might want to prioritize results
            # that are semantically similar to both the selected text and the query
            combined_query = f"Context: {selected_text}\nQuestion: {query}"
            results = await self.vector_search.search_with_query(
                query=combined_query,
                top_k=top_k
            )
        else:
            # For global queries, search normally
            results = await self.vector_search.search_with_query(
                query=query,
                top_k=top_k
            )

        # Apply additional filtering to ensure results are relevant
        # For selection-based queries, we might want to ensure results are related to the selected text
        if query_type == "selection" and selected_text:
            # Filter results to ensure they're related to the selected text context
            filtered_results = []
            for result in results:
                # This is a simple check - in practice, you might want more sophisticated filtering
                content = result.get('content', '').lower()
                selected_lower = selected_text.lower()

                # Check if the result content has some relation to the selected text
                # This could be improved with semantic similarity checks
                if any(word in content for word in selected_lower.split()[:5]):  # Check first 5 words
                    filtered_results.append(result)
                elif len(filtered_results) < top_k:  # Add some results even if not perfectly matched
                    filtered_results.append(result)

            results = filtered_results

        return results

    async def retrieve_for_citation(self, query: str, top_k: int = 3) -> List[Dict[str, Any]]:
        """
        Retrieve documents specifically for citation purposes
        This method focuses on getting clean, citable content
        """
        results = await self.retrieve_relevant_documents(query, top_k)

        # Format results for citation
        citations = []
        for result in results:
            citation = {
                'document_id': result.get('id'),
                'title': result.get('title'),
                'chapter': result.get('chapter'),
                'section': result.get('section'),
                'page_reference': result.get('page_reference'),
                'content': result.get('content'),
                'relevance_score': result.get('score')
            }
            citations.append(citation)

        return citations

    def validate_retrieval_quality(self, results: List[Dict[str, Any]], query: str) -> Dict[str, Any]:
        """
        Validate the quality of the retrieval results
        """
        quality_metrics = {
            'total_results': len(results),
            'avg_relevance_score': sum(r.get('score', 0) for r in results) / len(results) if results else 0,
            'has_high_quality_results': any(r.get('score', 0) > 0.7 for r in results) if results else False,
            'query_coverage': self._assess_query_coverage(results, query)
        }

        return quality_metrics

    def _assess_query_coverage(self, results: List[Dict[str, Any]], query: str) -> float:
        """
        Assess how well the results cover the query topics
        This is a simplified implementation
        """
        if not results:
            return 0.0

        query_keywords = set(query.lower().split())
        covered_keywords = set()

        for result in results:
            content = result.get('content', '').lower()
            result_keywords = set(content.split())
            covered_keywords.update(query_keywords.intersection(result_keywords))

        coverage = len(covered_keywords) / len(query_keywords) if query_keywords else 0.0
        return min(coverage, 1.0)  # Ensure value is between 0 and 1


# Global instance
retriever = Retriever()