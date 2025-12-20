import asyncio
import httpx
from typing import List, Dict, Any, Optional
from app.config import settings


class OpenRouterClient:
    """
    Client for interacting with OpenRouter API for both embeddings and chat completions
    """

    def __init__(self):
        self.base_url = settings.OPENROUTER_BASE_URL
        self.api_key = settings.OPENROUTER_API_KEY
        self.max_retries = 3
        self.retry_delay = 1

    async def generate_embeddings(self, texts: List[str], model: str = "text-embedding-ada-002") -> List[List[float]]:
        """
        Generate embeddings for a list of texts using OpenRouter API
        """
        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json"
        }

        embeddings = []
        for text in texts:
            # Truncate text if it's too long
            max_length = 8000  # Conservative limit
            if len(text) > max_length:
                text = text[:max_length]

            data = {
                "model": model,
                "input": text
            }

            async with httpx.AsyncClient(timeout=30.0) as client:
                for attempt in range(self.max_retries):
                    try:
                        response = await client.post(
                            f"{self.base_url}/embeddings",
                            headers=headers,
                            json=data
                        )

                        if response.status_code == 200:
                            result = response.json()
                            embedding = result['data'][0]['embedding']
                            embeddings.append(embedding)
                            break
                        elif response.status_code == 429:
                            # Rate limited - wait and retry
                            wait_time = self.retry_delay * (2 ** attempt)  # Exponential backoff
                            print(f"Rate limited, waiting {wait_time}s before retry {attempt + 1}")
                            await asyncio.sleep(wait_time)
                            continue
                        else:
                            print(f"Error {response.status_code}: {response.text}")
                            if attempt == self.max_retries - 1:
                                # Last attempt, return zeros as fallback
                                embeddings.append([0.0] * 1536)
                            break

                    except httpx.RequestError as e:
                        print(f"Request error on attempt {attempt + 1}: {str(e)}")
                        if attempt == self.max_retries - 1:
                            embeddings.append([0.0] * 1536)
                        await asyncio.sleep(self.retry_delay * (2 ** attempt))
                    except Exception as e:
                        print(f"Unexpected error on attempt {attempt + 1}: {str(e)}")
                        if attempt == self.max_retries - 1:
                            embeddings.append([0.0] * 1536)
                        await asyncio.sleep(self.retry_delay * (2 ** attempt))

        return embeddings

    async def generate_completion(
        self,
        messages: List[Dict[str, str]],
        model: str = "anthropic/claude-3-sonnet",
        temperature: float = 0.7,
        max_tokens: int = 1000
    ) -> Optional[str]:
        """
        Generate completion using OpenRouter API with Claude model
        """
        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json"
        }

        data = {
            "model": model,
            "messages": messages,
            "temperature": temperature,
            "max_tokens": max_tokens
        }

        async with httpx.AsyncClient(timeout=60.0) as client:
            for attempt in range(self.max_retries):
                try:
                    response = await client.post(
                        f"{self.base_url}/chat/completions",
                        headers=headers,
                        json=data
                    )

                    if response.status_code == 200:
                        result = response.json()
                        return result['choices'][0]['message']['content']
                    elif response.status_code == 429:
                        # Rate limited - wait and retry
                        wait_time = self.retry_delay * (2 ** attempt)  # Exponential backoff
                        print(f"Rate limited, waiting {wait_time}s before retry {attempt + 1}")
                        await asyncio.sleep(wait_time)
                        continue
                    else:
                        print(f"Error {response.status_code}: {response.text}")
                        if attempt == self.max_retries - 1:
                            return None
                        break

                except httpx.RequestError as e:
                    print(f"Request error on attempt {attempt + 1}: {str(e)}")
                    if attempt == self.max_retries - 1:
                        return None
                    await asyncio.sleep(self.retry_delay * (2 ** attempt))
                except Exception as e:
                    print(f"Unexpected error on attempt {attempt + 1}: {str(e)}")
                    if attempt == self.max_retries - 1:
                        return None
                    await asyncio.sleep(self.retry_delay * (2 ** attempt))

        return None

    async def get_model_info(self, model: str) -> Optional[Dict[str, Any]]:
        """
        Get information about a specific model
        """
        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json"
        }

        try:
            async with httpx.AsyncClient(timeout=30.0) as client:
                response = await client.get(
                    f"{self.base_url}/models/{model}",
                    headers=headers
                )

                if response.status_code == 200:
                    return response.json()
                else:
                    print(f"Error getting model info {response.status_code}: {response.text}")
                    return None
        except Exception as e:
            print(f"Error getting model info: {str(e)}")
            return None


# Global client instance
openrouter_client = OpenRouterClient()