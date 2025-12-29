from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import List, Dict, Any
import asyncio
from app.services.openrouter_client import openrouter_client

app = FastAPI(title="Test OpenRouter API", description="Minimal API to test OpenRouter integration")

class Message(BaseModel):
    role: str
    content: str

class ChatRequest(BaseModel):
    messages: List[Message]
    model: str = "mistralai/devstral-2512:free"
    temperature: float = 0.7
    max_tokens: int = 200

@app.get('/')
async def root():
    return {"message": "OpenRouter Test API is running!"}

@app.get('/health')
async def health():
    return {"status": "healthy", "model": "mistralai/devstral-2512:free"}

@app.post('/chat/completions')
async def chat_completions(request: ChatRequest):
    try:
        # Convert messages to the format expected by OpenRouter client
        message_list = [{"role": msg.role, "content": msg.content} for msg in request.messages]

        response = await openrouter_client.generate_completion(
            messages=message_list,
            model=request.model,
            temperature=request.temperature,
            max_tokens=request.max_tokens
        )

        if response:
            return {
                "choices": [{
                    "message": {
                        "role": "assistant",
                        "content": response
                    }
                }],
                "model": request.model
            }
        else:
            raise HTTPException(status_code=500, detail="No response from OpenRouter")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error calling OpenRouter: {str(e)}")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8003)
