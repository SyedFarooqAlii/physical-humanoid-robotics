import google.generativeai as genai
from typing import List, Dict, Any, Optional
from app.config import settings
import asyncio
import logging
from dataclasses import dataclass

logger = logging.getLogger(__name__)


@dataclass
class GeminiResponse:
    """Response from Gemini API"""
    text: str
    citations: List[Dict[str, str]]
    usage: Dict[str, int]


class GeminiClient:
    """
    Google Gemini API client for generating responses to user queries
    """

    def __init__(self):
        self.api_key = settings.GEMINI_API_KEY
        genai.configure(api_key=self.api_key)

        # Initialize the model - using gemini-2.5-flash which is available
        self.model = genai.GenerativeModel(
            model_name='gemini-2.5-flash',  # Using the available model
            generation_config={
                "temperature": 0.3,  # Lower temperature for more consistent, factual responses
                "max_output_tokens": 2048,
                "top_p": 0.95,
                "top_k": 40,
            }
        )

    async def generate_response(
        self,
        query: str,
        context: str,
        query_type: str = "global",
        selected_text: Optional[str] = None
    ) -> GeminiResponse:
        """
        Generate a response using Gemini based on the query and context
        """
        try:
            # Construct the prompt based on query type
            if query_type == "selection":
                # For selection-based queries, focus on the selected text
                prompt = f"""
                You are an expert assistant for the Physical AI & Humanoid Robotics curriculum.
                Please provide a helpful, conversational response to the user's question based on the selected text below.

                Your response should be:
                - Friendly and approachable
                - Directly address the user's question
                - Use information only from the provided selected text
                - If the selected text doesn't contain the answer, acknowledge this politely
                - Keep responses concise but informative
                - Use a warm, educational tone appropriate for students
                - When possible, synthesize and summarize information rather than requiring exact matches

                Selected text: {selected_text}

                Question: {query}

                Provide a helpful response: """
            else:
                # For global queries, use the retrieved context
                prompt = f"""
                You are an expert assistant for the Physical AI & Humanoid Robotics curriculum.
                Please provide a helpful, conversational response to the user's question based on the following context from the book.

                Your response should be:
                - Friendly and approachable
                - Directly address the user's question
                - Use information only from the provided context
                - If the context doesn't contain the answer, acknowledge this politely and suggest where they might find the information
                - Keep responses concise but informative
                - Use a warm, educational tone appropriate for students
                - If possible, relate concepts to practical applications in robotics
                - For questions about the book itself, modules, or curriculum, synthesize information from multiple sections
                - When information is implied across multiple contexts, feel free to summarize and explain
                - Do not respond with "The provided context does not contain information" if relevant information exists in the context

                Context: {context}

                Question: {query}

                Provide a helpful response: """

            # Generate content using Gemini
            response = await self.model.generate_content_async(prompt)

            # Extract the text response
            response_text = response.text if response.text else "I couldn't find relevant information to answer your question."

            # Extract usage information if available
            usage = {}
            if hasattr(response, 'usage_metadata'):
                usage = {
                    'prompt_tokens': getattr(response.usage_metadata, 'prompt_token_count', 0),
                    'response_tokens': getattr(response.usage_metadata, 'candidates_token_count', 0),
                    'total_tokens': getattr(response.usage_metadata, 'total_token_count', 0)
                }

            # For now, return empty citations - in a full implementation,
            # citations would be extracted from the context chunks
            citations = []

            return GeminiResponse(
                text=response_text,
                citations=citations,
                usage=usage
            )

        except Exception as e:
            logger.error(f"Error generating response with Gemini: {str(e)}")
            error_msg = str(e)
            # Check if it's a quota exceeded error
            if "quota" in error_msg.lower() or "429" in error_msg:
                return GeminiResponse(
                    text="I've reached my daily usage limit for the AI service. Please try again tomorrow, or feel free to ask me about the Physical AI & Humanoid Robotics curriculum content that I have access to.",
                    citations=[],
                    usage={}
                )
            else:
                return GeminiResponse(
                    text="I encountered an error while processing your request. Please try again.",
                    citations=[],
                    usage={}
                )

    async def validate_query(
        self,
        query: str,
        context: str
    ) -> bool:
        """
        Validate if the query can be answered with the given context
        """
        try:
            # Simple validation: check if query and context are not empty
            return bool(query.strip()) and bool(context.strip())
        except Exception:
            return False


# Global instance
gemini_client = GeminiClient()