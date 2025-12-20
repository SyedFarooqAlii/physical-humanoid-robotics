from typing import List, Dict, Any, Optional
from app.retrieval.retriever import Retriever
from app.prompting.prompt_builder import PromptBuilder


class ContextFilter:
    """
    Filters and validates contexts to prevent information leakage between query types
    """

    def __init__(self):
        self.retriever = Retriever()
        self.prompt_builder = PromptBuilder()

    def filter_context_for_query_type(
        self,
        contexts: List[Dict[str, Any]],
        query_type: str,
        selected_text: Optional[str] = None
    ) -> List[Dict[str, Any]]:
        """
        Filter contexts based on query type to prevent information leakage
        """
        if query_type == "selection" and selected_text:
            # For selection-based queries, we need to ensure contexts are relevant
            # to the selected text and don't introduce unrelated global knowledge
            return self._filter_selection_contexts(contexts, selected_text)
        elif query_type == "global":
            # For global queries, we can use all retrieved contexts
            return contexts
        else:
            # Default to global behavior
            return contexts

    def _filter_selection_contexts(
        self,
        contexts: List[Dict[str, Any]],
        selected_text: str
    ) -> List[Dict[str, Any]]:
        """
        Filter contexts to ensure they're relevant to the selected text for selection-based queries
        """
        if not contexts or not selected_text:
            return contexts

        # Simple relevance check: ensure contexts have some connection to the selected text
        # In a more sophisticated implementation, you might use semantic similarity
        selected_keywords = set(selected_text.lower().split()[:10])  # Use first 10 words as keywords
        filtered_contexts = []

        for context in contexts:
            content = context.get('content', '').lower()
            content_words = set(content.split())

            # Check if there's significant overlap in keywords
            keyword_overlap = len(selected_keywords.intersection(content_words))
            keyword_ratio = keyword_overlap / len(selected_keywords) if selected_keywords else 0

            # Include context if it has some relevance to the selected text
            # or if we don't have enough contexts yet
            if keyword_ratio > 0.1 or len(filtered_contexts) < 2:  # At least 10% overlap or include first few
                filtered_contexts.append(context)

        return filtered_contexts

    def validate_context_isolation(
        self,
        contexts: List[Dict[str, Any]],
        query_type: str,
        selected_text: Optional[str] = None
    ) -> Dict[str, Any]:
        """
        Validate that contexts are properly isolated based on query type
        """
        validation_result = {
            'is_valid': True,
            'query_type': query_type,
            'context_count': len(contexts),
            'issues': []
        }

        if query_type == "selection" and selected_text:
            # Validate that contexts are related to selected text
            validation_result.update(self._validate_selection_contexts(contexts, selected_text))
        elif query_type == "global":
            # For global queries, validate that we have diverse contexts
            validation_result.update(self._validate_global_contexts(contexts))

        return validation_result

    def _validate_selection_contexts(
        self,
        contexts: List[Dict[str, Any]],
        selected_text: str
    ) -> Dict[str, Any]:
        """
        Validate selection-based contexts
        """
        result = {
            'is_valid': True,
            'issues': []
        }

        if not contexts:
            result['is_valid'] = False
            result['issues'].append("No contexts provided for selection-based query")
            return result

        # Check relevance to selected text
        relevant_count = 0
        selected_keywords = set(selected_text.lower().split()[:10])

        for context in contexts:
            content = context.get('content', '').lower()
            content_words = set(content.split())
            keyword_overlap = len(selected_keywords.intersection(content_words))

            if keyword_overlap > 0:
                relevant_count += 1

        relevance_ratio = relevant_count / len(contexts)
        if relevance_ratio < 0.3:  # Less than 30% of contexts are relevant
            result['is_valid'] = False
            result['issues'].append(
                f"Only {relevant_count}/{len(contexts)} contexts ({relevance_ratio:.1%}) "
                f"are relevant to the selected text"
            )

        return result

    def _validate_global_contexts(self, contexts: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Validate global contexts
        """
        result = {
            'is_valid': True,
            'issues': []
        }

        if not contexts:
            result['is_valid'] = False
            result['issues'].append("No contexts provided for global query")
            return result

        # Check for diversity in chapters/sections
        unique_chapters = set(ctx.get('chapter', '') for ctx in contexts)
        unique_sections = set(ctx.get('section', '') for ctx in contexts)

        if len(unique_chapters) < 2 and len(contexts) > 1:
            result['issues'].append("Contexts lack diversity - all from same chapter")

        return result

    def enforce_context_boundaries(
        self,
        contexts: List[Dict[str, Any]],
        query_type: str,
        selected_text: Optional[str] = None,
        max_contexts: int = 5
    ) -> List[Dict[str, Any]]:
        """
        Enforce strict boundaries on contexts to prevent information leakage
        """
        # First, filter based on query type
        filtered_contexts = self.filter_context_for_query_type(contexts, query_type, selected_text)

        # Then enforce maximum count
        if len(filtered_contexts) > max_contexts:
            filtered_contexts = filtered_contexts[:max_contexts]

        # Validate the final contexts
        validation = self.validate_context_isolation(filtered_contexts, query_type, selected_text)

        if not validation['is_valid']:
            print(f"Context validation warning: {validation['issues']}")

        return filtered_contexts

    def build_isolated_context_string(
        self,
        contexts: List[Dict[str, Any]],
        query_type: str,
        selected_text: Optional[str] = None
    ) -> str:
        """
        Build a context string that enforces isolation between query types
        """
        if query_type == "selection" and selected_text:
            # Build context string focused on selected text and related content
            context_str = f"SELECTED TEXT:\n{selected_text}\n\nRELATED CONTENT:\n"
            for i, ctx in enumerate(contexts):
                context_str += f"[{i+1}] {ctx.get('content', '')}\n"
                context_str += f"Source: {ctx.get('chapter', '')} - {ctx.get('section', '')}\n\n"
        else:
            # Build global context string
            context_str = "RETRIEVED CONTENT:\n"
            for i, ctx in enumerate(contexts):
                context_str += f"[{i+1}] {ctx.get('content', '')}\n"
                context_str += f"Source: {ctx.get('chapter', '')} - {ctx.get('section', '')}\n\n"

        return context_str


# Global instance
context_filter = ContextFilter()