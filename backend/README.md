# RAG Chatbot Backend

This backend provides a Retrieval-Augmented Generation (RAG) chatbot for the Physical AI & Humanoid Robotics book.

## Features

- **Global Book Q&A**: Ask questions about the entire book content
- **Selection-Based Q&A**: Ask questions about user-selected text only
- **Citation-Aware Responses**: All answers include proper citations to book sections
- **Context-Only Responses**: Strictly answers based only on provided context (no hallucinations)

## Tech Stack

- **Framework**: FastAPI
- **Vector Database**: Qdrant Cloud
- **Database**: Neon Serverless PostgreSQL
- **AI Provider**: OpenRouter (Claude for reasoning, Qwen for embeddings)
- **Deployment**: Vercel

## API Endpoints

- `GET /` - Root endpoint
- `GET /api/health` - Health check
- `POST /api/ingest` - Ingest book content
- `POST /api/chat` - Chat endpoint with RAG capabilities

## Setup

1. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

2. Configure environment variables in `.env`:
   - `OPENROUTER_API_KEY`: Your OpenRouter API key
   - `QDRANT_URL`: Qdrant Cloud URL
   - `QDRANT_API_KEY`: Qdrant API key
   - `NEON_DATABASE_URL`: Neon PostgreSQL connection string

3. Run the server:
   ```bash
   python run_server.py
   ```

## Environment Variables

The application requires the following environment variables:

```env
OPENROUTER_API_KEY=your_openrouter_api_key
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_CLUSTER_ID=your_qdrant_cluster_id
NEON_DATABASE_URL=your_neon_database_url
BACKEND_API_KEY=your_backend_api_key
DEBUG=False
```

## Architecture

The backend follows a clean architecture with the following layers:

- **API Layer**: FastAPI endpoints
- **Service Layer**: Business logic and orchestration
- **Repository Layer**: Database operations
- **Infrastructure Layer**: External services (Qdrant, OpenRouter)

## Running Tests

```bash
python test_basic.py
```

## API Usage

### Chat Endpoint

```bash
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -H "X-API-Key: your_backend_api_key" \
  -d '{
    "session_id": "test-session",
    "message": "What are the main challenges in humanoid robotics?",
    "query_type": "global"
  }'
```

### Selection-Based Query

```bash
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -H "X-API-Key: your_backend_api_key" \
  -d '{
    "session_id": "test-session",
    "message": "Explain this concept?",
    "selected_text": "The control algorithms used in this humanoid robot include PID controllers...",
    "query_type": "selection"
  }'
```