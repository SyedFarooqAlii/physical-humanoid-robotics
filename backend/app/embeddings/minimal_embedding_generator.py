import numpy as np
from typing import List
import hashlib
import re
from collections import Counter


class MinimalEmbeddingGenerator:
    """
    Minimal embedding generator using simple hashing and basic statistics
    This avoids heavy dependencies and memory issues
    """

    def __init__(self):
        self.vocab = set()
        self.fitted = False

    def _pad_embedding(self, embedding: List[float], target_size: int = 1536) -> List[float]:
        """
        Pad embedding to target size with zeros
        """
        current_size = len(embedding)
        if current_size >= target_size:
            return embedding[:target_size]
        else:
            padded = [0.0] * target_size
            padded[:current_size] = embedding
            return padded

    def _text_to_vector(self, text: str) -> List[float]:
        """
        Convert text to a vector using simple hashing approach
        """
        # Clean text
        text = re.sub(r'[^\w\s]', ' ', text.lower())
        words = text.split()

        if not words:
            return [0.0] * 1536

        # Create a simple vector based on word hashes
        vector = [0.0] * 1536

        for word in words:
            # Use hash to determine position in vector
            hash_val = int(hashlib.md5(word.encode()).hexdigest(), 16)
            pos = hash_val % 1536
            # Add to vector with some normalization
            vector[pos] += 1.0 / len(words)  # Normalize by document length

        # Normalize the vector
        norm = sum(x**2 for x in vector) ** 0.5
        if norm > 0:
            vector = [x / norm for x in vector]

        return vector

    def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a list of texts using simple hashing
        """
        embeddings = []
        for text in texts:
            embedding = self._text_to_vector(text)
            padded_embedding = self._pad_embedding(embedding)
            embeddings.append(padded_embedding)

        return embeddings

    def encode_query(self, query: str) -> List[float]:
        """
        Encode a single query for similarity search
        """
        embedding = self._text_to_vector(query)
        return self._pad_embedding(embedding)


# Global instance
minimal_embedding_generator = MinimalEmbeddingGenerator()