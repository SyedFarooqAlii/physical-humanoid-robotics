import re
import uuid
from typing import List, Dict, Tuple
from dataclasses import dataclass
from .document_parser import DocumentParser


@dataclass
class TextChunk:
    """Represents a chunk of text with metadata"""
    id: str
    content: str
    title: str
    chapter: str
    section: str
    page_reference: str
    token_count: int
    original_start_pos: int
    original_end_pos: int


class TextChunker:
    """
    Implements heading-aware text chunking to maintain semantic boundaries
    """

    def __init__(self, max_tokens: int = 800, min_tokens: int = 100, overlap_ratio: float = 0.1):
        self.max_tokens = max_tokens
        self.min_tokens = min_tokens
        self.overlap_ratio = overlap_ratio
        self.parser = DocumentParser()

    def chunk_document(self, document: Dict) -> List[TextChunk]:
        """
        Chunk a document while preserving semantic boundaries
        """
        content = document['content']
        title = document['title']
        chapter = document['chapter']
        section = document['section']
        file_path = document['file_path']

        # First, identify structural boundaries (headings)
        structure = self._identify_structure(content)

        if not structure:
            # If no headings found, chunk by max token size
            return self._chunk_by_size(content, title, chapter, section, file_path)

        # Split content by structural boundaries
        structured_chunks = self._split_by_structure(content, structure)

        # Further chunk large structural sections if needed
        final_chunks = []
        for i, (start_pos, end_pos, heading_context) in enumerate(structured_chunks):
            chunk_content = content[start_pos:end_pos]

            if self._count_tokens(chunk_content) > self.max_tokens:
                # Split large sections while preserving heading context
                sub_chunks = self._chunk_large_section(
                    chunk_content,
                    title,
                    chapter,
                    section,
                    file_path,
                    heading_context,
                    start_pos
                )
                final_chunks.extend(sub_chunks)
            else:
                # Create a single chunk for this section
                chunk_id = str(uuid.uuid4())
                token_count = self._count_tokens(chunk_content)

                chunk = TextChunk(
                    id=chunk_id,
                    content=chunk_content,
                    title=title,
                    chapter=chapter,
                    section=section,
                    page_reference=file_path,
                    token_count=token_count,
                    original_start_pos=start_pos,
                    original_end_pos=end_pos
                )
                final_chunks.append(chunk)

        return final_chunks

    def _identify_structure(self, content: str) -> List[Tuple[int, str]]:
        """
        Identify structural boundaries in the content (headings)
        Returns list of (position, heading_text) tuples
        """
        lines = content.split('\n')
        structure = []
        pos = 0

        for line in lines:
            # Check for markdown headings
            heading_match = re.match(r'^(\#{1,6})\s+(.+)', line)
            if heading_match:
                level = len(heading_match.group(1))
                heading_text = heading_match.group(2).strip()
                structure.append((pos, f"{'#' * level} {heading_text}"))

            pos += len(line) + 1  # +1 for newline

        return structure

    def _split_by_structure(self, content: str, structure: List[Tuple[int, str]]) -> List[Tuple[int, int, str]]:
        """
        Split content by structural boundaries
        Returns list of (start_pos, end_pos, heading_context) tuples
        """
        if not structure:
            return [(0, len(content), "")]

        splits = []
        start_pos = 0

        for pos, heading in structure:
            if pos > start_pos:
                # Add the chunk before this heading
                splits.append((start_pos, pos, heading))
            start_pos = pos

        # Add the final chunk if there's remaining content
        if start_pos < len(content):
            splits.append((start_pos, len(content), structure[-1][1]))

        return splits

    def _chunk_by_size(self, content: str, title: str, chapter: str, section: str, file_path: str) -> List[TextChunk]:
        """
        Fallback method to chunk content by size when no structure is available
        """
        chunks = []
        tokens_per_chunk = self._estimate_tokens_per_chunk()
        chunk_size = tokens_per_chunk * 4  # Rough estimate: 4 chars per token

        for i in range(0, len(content), chunk_size):
            chunk_content = content[i:i + chunk_size]
            chunk_id = str(uuid.uuid4())

            chunk = TextChunk(
                id=chunk_id,
                content=chunk_content,
                title=title,
                chapter=chapter,
                section=section,
                page_reference=file_path,
                token_count=self._count_tokens(chunk_content),
                original_start_pos=i,
                original_end_pos=min(i + chunk_size, len(content))
            )
            chunks.append(chunk)

        return chunks

    def _chunk_large_section(
        self,
        content: str,
        title: str,
        chapter: str,
        section: str,
        file_path: str,
        heading_context: str,
        offset: int
    ) -> List[TextChunk]:
        """
        Further chunk a large section while preserving heading context
        """
        chunks = []

        # If we have heading context, prepend it to each chunk
        context_prefix = f"{heading_context}\n\n" if heading_context else ""

        # Split content into sentences to find good break points
        sentences = self._split_into_sentences(content)

        current_chunk = ""
        current_tokens = 0
        chunk_idx = 0

        for sentence in sentences:
            sentence_tokens = self._count_tokens(sentence)

            # If adding this sentence would exceed the token limit
            if current_tokens + sentence_tokens > self.max_tokens and current_chunk:
                # Save the current chunk
                chunk_id = str(uuid.uuid4())
                chunk = TextChunk(
                    id=chunk_id,
                    content=context_prefix + current_chunk,
                    title=title,
                    chapter=chapter,
                    section=section,
                    page_reference=file_path,
                    token_count=current_tokens,
                    original_start_pos=offset + content.find(current_chunk),
                    original_end_pos=offset + content.find(current_chunk) + len(current_chunk)
                )
                chunks.append(chunk)

                # Start a new chunk with potential overlap
                overlap_size = int(len(current_chunk) * self.overlap_ratio)
                overlap_content = current_chunk[-overlap_size:] if overlap_size > 0 else ""

                current_chunk = overlap_content + sentence
                current_tokens = self._count_tokens(context_prefix + current_chunk)
                chunk_idx += 1
            else:
                # Add sentence to current chunk
                current_chunk += sentence
                current_tokens += sentence_tokens

        # Add the final chunk if it has content
        if current_chunk.strip():
            chunk_id = str(uuid.uuid4())
            chunk = TextChunk(
                id=chunk_id,
                content=context_prefix + current_chunk,
                title=title,
                chapter=chapter,
                section=section,
                page_reference=file_path,
                token_count=self._count_tokens(context_prefix + current_chunk),
                original_start_pos=offset + content.find(current_chunk),
                original_end_pos=offset + content.find(current_chunk) + len(current_chunk)
            )
            chunks.append(chunk)

        return chunks

    def _split_into_sentences(self, text: str) -> List[str]:
        """
        Split text into sentences using common sentence boundaries
        """
        # Use regex to split on sentence boundaries while preserving the delimiters
        sentence_pattern = r'(?<=[.!?])\s+'
        sentences = re.split(sentence_pattern, text)

        # Re-add the punctuation to each sentence
        result = []
        for i, sentence in enumerate(sentences):
            if i < len(sentences) - 1:
                # Check if the original text had punctuation at the end of this sentence
                # by looking at the character that followed this sentence in the original text
                next_char_idx = len(''.join(sentences[:i+1])) + i  # +i for spaces
                if next_char_idx < len(text):
                    next_char = text[next_char_idx] if next_char_idx < len(text) else ''
                    if next_char in '.!?':
                        sentence += next_char
            result.append(sentence + ' ')

        # Clean up and ensure each sentence is properly formatted
        result = [s.strip() for s in result if s.strip()]
        return result

    def _count_tokens(self, text: str) -> int:
        """
        Count approximate number of tokens in text
        This is a simple estimation; for more accurate counting, use tiktoken
        """
        import tiktoken
        # Use cl100k_base encoding which is used by many OpenAI models
        encoding = tiktoken.get_encoding("cl100k_base")
        return len(encoding.encode(text))

    def _estimate_tokens_per_chunk(self) -> int:
        """
        Estimate number of tokens that would fit in a chunk based on max_tokens
        """
        # This is a rough estimation - in practice, you might want to use
        # a more sophisticated approach based on your specific content
        return min(self.max_tokens, 800)  # Conservative estimate


def chunk_documents(documents: List[Dict]) -> List[TextChunk]:
    """
    Convenience function to chunk a list of documents
    """
    chunker = TextChunker()
    all_chunks = []

    for document in documents:
        chunks = chunker.chunk_document(document)
        all_chunks.extend(chunks)

    return all_chunks