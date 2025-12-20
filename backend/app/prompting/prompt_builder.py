from typing import List, Dict, Any, Optional
from app.retrieval.retriever import Retriever
from app.config import settings


class PromptBuilder:
    """
    Builds context-aware prompts for the LLM while preventing hallucinations
    """

    def __init__(self):
        self.retriever = Retriever()

    def build_global_query_prompt(
        self,
        query: str,
        retrieved_contexts: List[Dict[str, Any]],
        max_context_length: int = 2000
    ) -> str:
        """
        Build a prompt for global book queries using all retrieved contexts
        """
        # Start with system message to prevent hallucinations
        system_prompt = (
            "You are an AI assistant that answers questions based only on the provided context. "
            "Do not use any prior knowledge or information not present in the context. "
            "If the answer cannot be found in the provided context, respond with: "
            "'The provided context does not contain information to answer this question.'\n\n"
        )

        # Add retrieved contexts
        context_section = "### CONTEXT:\n\n"
        total_context_length = 0

        for i, ctx in enumerate(retrieved_contexts):
            if total_context_length >= max_context_length:
                break

            context_text = ctx.get('content', '')
            # Truncate if too long
            if len(context_text) + total_context_length > max_context_length:
                available_length = max_context_length - total_context_length
                context_text = context_text[:available_length]

            context_section += f"**Source {i+1} ({ctx.get('title', 'Unknown')} - {ctx.get('chapter', 'Unknown')}):**\n"
            context_section += f"{context_text}\n\n"
            total_context_length += len(context_text)

        # Add user query
        user_query_section = f"### QUESTION:\n{query}\n\n"

        # Add instruction for response format
        response_format = (
            "### INSTRUCTIONS:\n"
            "1. Answer the question based ONLY on the provided context\n"
            "2. If the context doesn't contain the answer, say so explicitly\n"
            "3. Include relevant citations to the sources in your response\n"
            "4. Keep your response concise and to the point\n\n"
        )

        # Combine all parts
        full_prompt = system_prompt + context_section + user_query_section + response_format
        return full_prompt

    def build_selection_based_prompt(
        self,
        query: str,
        selected_text: str,
        retrieved_contexts: List[Dict[str, Any]],
        max_context_length: int = 2000
    ) -> str:
        """
        Build a prompt for selection-based queries using only the selected text and relevant contexts
        """
        # Start with system message
        system_prompt = (
            "You are an AI assistant that answers questions based only on the provided selected text and related context. "
            "Do not use any prior knowledge or information not present in the provided content. "
            "Focus your answer on the relationship between the selected text and the question. "
            "If the answer cannot be found in the provided content, respond with: "
            "'The provided content does not contain information to answer this question.'\n\n"
        )

        # Add the selected text as primary context
        primary_context = f"### SELECTED TEXT:\n{selected_text}\n\n"

        # Add related contexts
        related_context = "### RELATED CONTEXT:\n\n"
        total_context_length = len(selected_text)

        for i, ctx in enumerate(retrieved_contexts):
            if total_context_length >= max_context_length:
                break

            context_text = ctx.get('content', '')
            # Truncate if too long
            if len(context_text) + total_context_length > max_context_length:
                available_length = max_context_length - total_context_length
                context_text = context_text[:available_length]

            related_context += f"**Related Content {i+1} ({ctx.get('title', 'Unknown')} - {ctx.get('chapter', 'Unknown')}):**\n"
            related_context += f"{context_text}\n\n"
            total_context_length += len(context_text)

        # Add user query
        user_query_section = f"### QUESTION ABOUT SELECTED TEXT:\n{query}\n\n"

        # Add instruction for response format
        response_format = (
            "### INSTRUCTIONS:\n"
            "1. Answer the question based ONLY on the selected text and related context\n"
            "2. Focus on how the question relates to the selected text\n"
            "3. If the content doesn't contain the answer, say so explicitly\n"
            "4. Include citations to the sources in your response\n"
            "5. Keep your response concise and relevant to the selected text\n\n"
        )

        # Combine all parts
        full_prompt = system_prompt + primary_context + related_context + user_query_section + response_format
        return full_prompt

    def build_context_filter_prompt(
        self,
        query: str,
        available_contexts: List[Dict[str, Any]],
        max_contexts_to_use: int = 3
    ) -> str:
        """
        Build a prompt to filter relevant contexts from a larger set
        """
        system_prompt = (
            "You are an AI assistant that helps filter relevant contexts for a given question. "
            "Analyze the question and the provided contexts, then select only the most relevant ones.\n\n"
        )

        # Add contexts
        contexts_section = "### AVAILABLE CONTEXTS:\n\n"
        for i, ctx in enumerate(available_contexts):
            contexts_section += f"**Context {i+1} ({ctx.get('title', 'Unknown')} - {ctx.get('chapter', 'Unknown')}):**\n"
            contexts_section += f"{ctx.get('content', '')[:500]}...\n\n"  # Truncate for brevity

        # Add query
        query_section = f"### QUESTION:\n{query}\n\n"

        # Add instructions
        instruction_section = (
            "### INSTRUCTIONS:\n"
            "1. Identify which contexts are most relevant to answering the question\n"
            "2. List the most relevant contexts by their number\n"
            "3. Provide a brief reason for why each selected context is relevant\n"
            f"4. Select at most {max_contexts_to_use} contexts\n\n"
        )

        # Add response format
        response_format = (
            "### RESPONSE FORMAT:\n"
            "Respond with only the following JSON format:\n"
            "{\n"
            "  \"relevant_contexts\": [\n"
            "    {\n"
            "      \"index\": 0,\n"
            "      \"reason\": \"Brief explanation of relevance\"\n"
            "    }\n"
            "  ]\n"
            "}\n"
        )

        full_prompt = system_prompt + contexts_section + query_section + instruction_section + response_format
        return full_prompt

    def validate_prompt_context_isolation(self, prompt: str, query_type: str, original_context: Optional[str] = None) -> bool:
        """
        Validate that the prompt properly isolates contexts based on query type
        """
        if query_type == "selection" and original_context:
            # For selection queries, ensure the prompt focuses on the selected text
            # This is a basic validation - in practice, you'd want more sophisticated checks
            return "SELECTED TEXT" in prompt or original_context[:50] in prompt
        else:
            # For global queries, ensure the prompt uses broader context
            return "CONTEXT:" in prompt

        return True


# Global instance
prompt_builder = PromptBuilder()