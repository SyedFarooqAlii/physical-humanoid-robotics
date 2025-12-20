# Quickstart Guide: RAG Chatbot for Physical AI Book

## Overview
This guide provides step-by-step instructions to set up and run the RAG chatbot for the Physical AI & Humanoid Robotics book.

## Prerequisites
- Python 3.9+
- Node.js 16+ (for Docusaurus frontend)
- Access to OpenRouter API (Claude and Qwen models)
- Qdrant Cloud account (free tier)
- Neon Serverless PostgreSQL account

## Backend Setup

### 1. Clone and Navigate to Backend
```bash
cd backend
```

### 2. Create Virtual Environment
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies
```bash
pip install -r requirements.txt
```

### 4. Configure Environment Variables
```bash
cp .env.example .env
```

Edit `.env` and add your API keys and configuration:
```
OPENROUTER_API_KEY=your_openrouter_api_key
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
NEON_DATABASE_URL=your_neon_database_url
```

### 5. Run the Application
```bash
uvicorn app.main:app --reload
```

The backend will be available at `http://localhost:8000`

## Frontend Setup

### 1. Navigate to Docusaurus Directory
```bash
cd docusaurus
```

### 2. Install Dependencies
```bash
npm install
```

### 3. Start Development Server
```bash
npm run start
```

## Ingest Book Content

### 1. Prepare Book Content
Ensure your Docusaurus markdown files are in the correct directory structure.

### 2. Run Ingestion API
```bash
curl -X POST http://localhost:8000/api/ingest \
  -H "Content-Type: application/json" \
  -d '{
    "book_path": "/path/to/your/docusaurus/docs",
    "chunk_size": 800
  }'
```

## Test the Chat Functionality

### 1. Global Book Q&A
```bash
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{
    "session_id": "test-session",
    "message": "What are the main challenges in humanoid robotics?",
    "query_type": "global"
  }'
```

### 2. Selection-Based Q&A
```bash
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{
    "session_id": "test-session",
    "message": "Explain the control algorithms mentioned here?",
    "selected_text": "The control algorithms used in this humanoid robot include PID controllers for joint position control and model predictive control for dynamic balance maintenance.",
    "query_type": "selection"
  }'
```

## Frontend Integration

The chat widget will automatically appear on your Docusaurus pages. Users can:
1. Type questions in the chat widget for global book Q&A
2. Select text and use the "Ask about selected text" option for context-specific answers
3. View citations to book sections in the responses

## Deployment

### Backend (Vercel)
1. Connect your repository to Vercel
2. Set environment variables in Vercel dashboard
3. Deploy the application

### Frontend (GitHub + Vercel)
1. Push your Docusaurus changes to GitHub
2. Connect GitHub repository to Vercel
3. Configure build settings to run `npm run build`
4. Deploy the site

## Troubleshooting

### Common Issues

1. **API Rate Limits**: If you encounter rate limit errors, implement proper caching or reduce query frequency.

2. **Vector Database Connection**: Verify Qdrant URL and API key are correctly configured.

3. **Embedding Generation Errors**: Check OpenRouter API key and model availability.

4. **Slow Response Times**: Consider implementing caching for frequently asked questions.

### Environment Variables Reference
- `OPENROUTER_API_KEY`: API key for OpenRouter (required)
- `QDRANT_URL`: URL for Qdrant Cloud instance (required)
- `QDRANT_API_KEY`: API key for Qdrant Cloud (required)
- `NEON_DATABASE_URL`: Connection string for Neon PostgreSQL (required)
- `DEBUG`: Set to "True" for development debugging (optional)
- `RATE_LIMIT_REQUESTS`: Number of requests per minute (default: 100)
- `RATE_LIMIT_WINDOW`: Time window in seconds (default: 60)

## Next Steps

1. Run the ingestion process to index your book content
2. Test both global and selection-based Q&A
3. Verify citation accuracy
4. Monitor performance and usage
5. Deploy to production environment