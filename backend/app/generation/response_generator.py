import asyncio
import json
from typing import List, Dict, Any, Optional
from app.services.gemini_client import gemini_client
from app.prompting.prompt_builder import PromptBuilder
from app.retrieval.retriever import Retriever
from app.config import settings


class ResponseGenerator:
    """
    Generates responses using Google Gemini with proper context and citation tracking
    """

    def __init__(self):
        self.gemini_client = gemini_client
        self.prompt_builder = PromptBuilder()
        self.retriever = Retriever()

    async def generate_response(
        self,
        query: str,
        retrieved_contexts: List[Dict[str, Any]],
        query_type: str = "global",
        selected_text: Optional[str] = None,
        session_id: Optional[str] = None
    ) -> Dict[str, Any]:
        """
        Generate a response to a query using Google Gemini
        """
        try:
            # Build context string from retrieved contexts
            context_parts = []
            for ctx in retrieved_contexts:
                context_text = ctx.get('content', '')
                if context_text:
                    context_parts.append(context_text)

            context_string = "\n\n".join(context_parts)

            # Generate response using Gemini
            gemini_response = await self.gemini_client.generate_response(
                query=query,
                context=context_string,
                query_type=query_type,
                selected_text=selected_text
            )

            if not gemini_response.text:
                return {
                    "response": "I encountered an issue generating a response. Please try again.",
                    "citations": [],
                    "query_type": query_type,
                    "session_id": session_id
                }

            # Extract citations from contexts used
            citations = self._extract_citations(retrieved_contexts)

            return {
                "response": gemini_response.text,
                "citations": citations,
                "query_type": query_type,
                "session_id": session_id
            }

        except Exception as e:
            print(f"Error generating response: {str(e)}")
            # If AI service fails, try to provide a helpful response based on the context
            if retrieved_contexts:
                # Extract key information from contexts to provide a basic response
                context_titles = [ctx.get('title', '') for ctx in retrieved_contexts if ctx.get('title')]
                unique_titles = list(set(context_titles))

                if unique_titles:
                    response = f"Based on the Physical AI & Humanoid Robotics curriculum, I found information related to: {', '.join(unique_titles[:3])}. "
                    response += "The specific answer to your question may be available in these sections of the book. "
                    response += "Feel free to ask more specific questions about these topics!"
                else:
                    response = "I found some relevant content in the Physical AI & Humanoid Robotics curriculum. "
                    response += "Could you ask a more specific question about the topic you're interested in?"
            else:
                response = "I couldn't find relevant information in the Physical AI & Humanoid Robotics curriculum to answer your question. "
                response += "Please try asking about specific topics from the curriculum like ROS 2, Digital Twins, AI-Brain, or VLA."

            return {
                "response": response,
                "citations": self._extract_citations(retrieved_contexts),
                "query_type": query_type,
                "session_id": session_id,
                "error": str(e)
            }

    async def generate_response_with_validation(
        self,
        query: str,
        retrieved_contexts: List[Dict[str, Any]],
        query_type: str = "global",
        selected_text: Optional[str] = None,
        session_id: Optional[str] = None
    ) -> Dict[str, Any]:
        """
        Generate response with additional validation to ensure it's based only on provided context
        """
        try:
            # First, generate the initial response
            result = await self.generate_response(
                query=query,
                retrieved_contexts=retrieved_contexts,
                query_type=query_type,
                selected_text=selected_text,
                session_id=session_id
            )

            # Validate that the response is grounded in the provided context
            # But be more lenient for high-level book/module questions
            if retrieved_contexts and "The provided context does not contain" not in result.get("response", ""):
                is_valid = self._validate_response_uses_context(
                    response=result["response"],
                    contexts=retrieved_contexts
                )

                # For high-level book/module questions, be more lenient with validation
                query_lower = query.lower()
                is_book_overview_query = any(phrase in query_lower for phrase in [
                    'what is', 'tell me about', 'describe', 'overview', 'introduction',
                    'physical ai', 'humanoid robotics', 'book', 'curriculum', 'module',
                    'quick start', 'setup', 'getting started', 'chapter', 'section'
                ])

                # Only retry with stronger guidance if it's not a book overview query and validation fails
                if not is_valid and not is_book_overview_query and query_type != "selection":
                    # If response doesn't seem to use context, try again with stronger instructions
                    result = await self._generate_with_stronger_context_guidance(
                        query=query,
                        retrieved_contexts=retrieved_contexts,
                        query_type=query_type,
                        selected_text=selected_text,
                        session_id=session_id
                    )

            return result
        except Exception as e:
            print(f"Error in generate_response_with_validation: {str(e)}")
            # If AI service fails, try to provide a helpful response based on the context
            if retrieved_contexts:
                # Extract key information from contexts to provide a basic response
                context_titles = [ctx.get('title', '') for ctx in retrieved_contexts if ctx.get('title')]
                unique_titles = list(set(context_titles))

                if unique_titles:
                    response = f"Based on the Physical AI & Humanoid Robotics curriculum, I found information related to: {', '.join(unique_titles[:3])}. "
                    response += "The specific answer to your question may be available in these sections of the book. "
                    response += "Feel free to ask more specific questions about these topics!"
                else:
                    response = "I found some relevant content in the Physical AI & Humanoid Robotics curriculum. "
                    response += "Could you ask a more specific question about the topic you're interested in?"
            else:
                response = "I couldn't find relevant information in the Physical AI & Humanoid Robotics curriculum to answer your question. "
                response += "Please try asking about specific topics from the curriculum like ROS 2, Digital Twins, AI-Brain, or VLA."

            return {
                "response": response,
                "citations": self._extract_citations(retrieved_contexts),
                "query_type": query_type,
                "session_id": session_id,
                "error": str(e)
            }

    async def _generate_with_stronger_context_guidance(
        self,
        query: str,
        retrieved_contexts: List[Dict[str, Any]],
        query_type: str,
        selected_text: Optional[str] = None,
        session_id: Optional[str] = None
    ) -> Dict[str, Any]:
        """
        Generate response with stronger instructions to use only provided context
        """
        # Build context string from retrieved contexts
        context_parts = []
        for ctx in retrieved_contexts:
            context_text = ctx.get('content', '')
            if context_text:
                context_parts.append(context_text)

        context_string = "\n\n".join(context_parts)

        # Add stronger instructions to the context
        stronger_context = (
            context_string +
            "\n\nIMPORTANT: The response MUST be based ONLY on the provided context above. "
            "Do not use any external knowledge or make up information. "
            "If the context does not contain the answer, explicitly state this fact."
        )

        # Generate response using Gemini
        gemini_response = await self.gemini_client.generate_response(
            query=query,
            context=stronger_context,
            query_type=query_type,
            selected_text=selected_text
        )

        if not gemini_response.text:
            return {
                "response": "I encountered an issue generating a response. Please try again.",
                "citations": [],
                "query_type": query_type,
                "session_id": session_id
            }

        citations = self._extract_citations(retrieved_contexts)

        return {
            "response": gemini_response.text,
            "citations": citations,
            "query_type": query_type,
            "session_id": session_id
        }

    def _extract_citations(self, contexts: List[Dict[str, Any]]) -> List[Dict[str, str]]:
        """
        Extract citation information from retrieved contexts
        """
        citations = []
        for ctx in contexts:
            citation = {
                "document_id": ctx.get('id', ''),
                "title": ctx.get('title', ''),
                "chapter": ctx.get('chapter', ''),
                "section": ctx.get('section', ''),
                "page_reference": ctx.get('page_reference', '')
            }
            citations.append(citation)
        return citations

    def _validate_response_uses_context(
        self,
        response: str,
        contexts: List[Dict[str, Any]]
    ) -> bool:
        """
        Validate if response uses the provided context, allowing for summarization and synthesis
        """
        if not contexts:
            return False

        response_lower = response.lower()

        # Check for semantic relevance rather than exact keyword matching
        # Look for broader topic matches and concepts
        context_text = " ".join([ctx.get('content', '') for ctx in contexts if ctx.get('content')])
        context_lower = context_text.lower()

        # Check for relevant topics and concepts that indicate proper grounding
        book_related_terms = [
            'physical ai', 'humanoid', 'robotics', 'module', 'ros', 'digital twin',
            'ai-brain', 'vla', 'curriculum', 'course', 'book', 'chapter', 'section',
            'setup', 'quickstart', 'introduction', 'learning', 'education'
        ]

        # Check if response mentions book-related concepts that are in context
        response_has_relevant_terms = any(term in response_lower for term in book_related_terms)
        context_has_relevant_terms = any(term in context_lower for term in book_related_terms)

        if response_has_relevant_terms and context_has_relevant_terms:
            return True

        # Also check for title and chapter references
        context_titles = " ".join([ctx.get('title', '') + ' ' + ctx.get('chapter', '') + ' ' + ctx.get('section', '')
                                  for ctx in contexts if ctx.get('title') or ctx.get('chapter') or ctx.get('section')])
        context_titles_lower = context_titles.lower()

        # If response mentions specific titles/chapters that exist in context, it's likely valid
        if any(title_term in response_lower for title_term in context_titles_lower.split() if len(title_term) > 3):
            return True

        # Fallback: check if there's general overlap in concepts
        response_words = set(response_lower.split())
        context_words = set(context_lower.split())

        # Find intersection of meaningful words (longer than 3 chars, not common words)
        common_words = response_words.intersection(context_words)
        meaningful_common_words = [w for w in common_words if len(w) > 3 and w not in ['the', 'and', 'for', 'are', 'but', 'not', 'you', 'have', 'with', 'this', 'that', 'from']]

        # If there are meaningful overlapping words, consider it valid
        return len(meaningful_common_words) >= 2

    async def generate_citation_aware_response(
        self,
        query: str,
        retrieved_contexts: List[Dict[str, Any]],
        query_type: str = "global",
        selected_text: Optional[str] = None,
        session_id: Optional[str] = None
    ) -> Dict[str, Any]:
        """
        Generate a response that explicitly mentions which sources were used
        """
        # Build context string from retrieved contexts
        context_parts = []
        for ctx in retrieved_contexts:
            context_text = ctx.get('content', '')
            if context_text:
                context_parts.append(context_text)

        context_string = "\n\n".join(context_parts)

        # Add specific instruction about citations to the context
        citation_context = (
            context_string +
            "\n\nWhen answering, please indicate which sources you used by referencing them as "
            "[Source 1], [Source 2], etc., corresponding to the order they appear in the context section."
        )

        # Generate response using Gemini
        gemini_response = await self.gemini_client.generate_response(
            query=query,
            context=citation_context,
            query_type=query_type,
            selected_text=selected_text
        )

        if not gemini_response.text:
            return {
                "response": "I encountered an issue generating a response. Please try again.",
                "citations": [],
                "query_type": query_type,
                "session_id": session_id
            }

        citations = self._extract_citations(retrieved_contexts)

        return {
            "response": gemini_response.text,
            "citations": citations,
            "query_type": query_type,
            "session_id": session_id
        }


# Global instance
response_generator = ResponseGenerator()