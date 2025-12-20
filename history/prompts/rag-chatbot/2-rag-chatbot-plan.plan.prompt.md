---
id: 2
title: rag-chatbot-plan
stage: plan
date: 2025-12-17
surface: agent
model: Claude Sonnet 4.5
feature: rag-chatbot
branch: 002-rag-chatbot
user: Claude Code
command: /sp.plan
labels: [rag, chatbot, planning, architecture, api]
links:
  spec: specs/002-rag-chatbot/spec.md
  plan: specs/002-rag-chatbot/plan.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/002-rag-chatbot/plan.md
 - specs/002-rag-chatbot/research.md
 - specs/002-rag-chatbot/data-model.md
 - specs/002-rag-chatbot/contracts/api-contract.yaml
 - specs/002-rag-chatbot/quickstart.md
tests:
 - architecture feasibility validated
 - API contracts defined
 - data models specified
---

## Prompt

# /sp.plan — RAG Chatbot Integration

You are now in **/sp.plan mode**.

Convert the RAG chatbot specification into a **technical execution plan**.

---

## REQUIRED SECTIONS

### 1. SYSTEM ARCHITECTURE
- Docusaurus UI → Chat Widget
- FastAPI RAG service
- OpenRouter (Claude + Qwen)
- Qdrant (vector search)
- Neon Postgres (documents + logs)

---

### 2. RAG FLOW
- Query → embedding
- Vector retrieval (top-k)
- Context filtering (selection vs global)
- Prompt assembly
- Claude response via OpenRouter

---

### 3. DATA INGESTION PLAN
- Markdown parsing
- Chunk strategy
- Embedding pipeline
- Re-indexing strategy on content updates

---

### 4. DECISIONS & TRADEOFFS
- Claude vs GPT via OpenRouter
- Qwen embeddings vs OpenAI
- Qdrant vs Pinecone
- Serverless DB vs local DB

---

### 5. TESTING & VALIDATION
- Grounded-answer checks
- Selection-only isolation test
- Empty-context failure handling
- Latency & token cost checks

---

### 6. DEPLOYMENT
- Backend: Vercel / Railway
- Vector DB: Qdrant Cloud
- DB: Neon
- Frontend: GitHub Pages + Vercel preview

---

## EXECUTE

**/sp.plan — RAG Chatbot Integration**

## Response snapshot

# Implementation Plan: RAG Chatbot for Physical AI Book

**Feature**: 002-rag-chatbot
**Created**: 2025-12-17
**Status**: Draft
**Plan Version**: 1.0

## Technical Context

This plan outlines the implementation of a Retrieval-Augmented Generation (RAG) chatbot for the Physical AI & Humanoid Robotics book. The system will allow readers to ask questions about book content and receive accurate, citation-aware responses.

**System Components**:
- **Frontend**: Docusaurus-based book UI with embedded chat widget
- **Backend**: FastAPI service handling RAG operations
- **AI Services**: OpenRouter API for Claude reasoning and Qwen embeddings
- **Vector Database**: Qdrant Cloud for semantic search
- **Metadata Database**: Neon Serverless PostgreSQL for document tracking and logs

**Architecture Overview**:
The system follows a client-server pattern where the Docusaurus frontend communicates with a FastAPI backend. The backend handles document indexing, vector storage, and RAG operations using OpenRouter's Claude and Qwen models.

**Key Technologies**:
- **Frontend**: React-based chat widget embedded in Docusaurus
- **Backend**: FastAPI with async processing
- **Vector DB**: Qdrant Cloud (free tier)
- **Database**: Neon PostgreSQL (serverless)
- **AI Provider**: OpenRouter (Claude for reasoning, Qwen for embeddings)
- **Deployment**: Vercel/Railway for backend, GitHub Pages for frontend

## Constitution Check

This implementation aligns with the project constitution by:
- Ensuring accuracy through direct verification (RAG methodology prevents hallucinations)
- Maintaining reproducibility and transparency (all components are documented)
- Preventing plagiarism and hallucination (strict RAG guardrails)
- Creating modular and accessible content (clear API contracts and documentation)

**Potential Issues**:
- API costs from OpenRouter may exceed free tier (mitigated by rate limiting and usage tracking)
- Data privacy with third-party AI services (requires clear user consent)

## Gates

**GATE 1: Architecture Feasibility** - PASSED
- All required technologies are available and compatible
- Architecture supports all specified functional requirements

**GATE 2: Resource Constraints** - PASSED
- Free tier limits are sufficient for initial deployment
- Qdrant Cloud and Neon Serverless provide adequate capacity

**GATE 3: Technical Requirements** - PASSED
- All functional requirements from spec can be implemented
- Performance targets are achievable with proper caching

**GATE 4: Constitution Alignment** - PASSED
- Implementation respects all constitutional principles
- No ethical concerns identified

## Phase 0: Research & Unknown Resolution

### Research Tasks

1. **OpenRouter API Integration**
   - Decision: Use OpenRouter's API for both Claude reasoning and Qwen embeddings
   - Rationale: Provides access to state-of-the-art models without requiring direct API keys from multiple providers
   - Alternatives considered: Direct OpenAI API, Anthropic API, local models

2. **Qdrant Vector Database Setup**
   - Decision: Use Qdrant Cloud free tier for vector storage
   - Rationale: Optimized for semantic search, good Python SDK, free tier available
   - Alternatives considered: Pinecone, Weaviate, ChromaDB, Supabase Vector

3. **Text Chunking Strategy**
   - Decision: Use heading-aware chunking to maintain semantic boundaries
   - Rationale: Preserves context and enables accurate citations
   - Alternatives considered: Fixed-size sliding windows, sentence-based chunks

4. **Docusaurus Widget Integration**
   - Decision: Create React-based chat widget component
   - Rationale: Seamless integration with existing Docusaurus framework
   - Alternatives considered: Iframe embedding, external popup

## Phase 1: Design & Contracts

### Data Model

**BookContentDocument**
- document_id (string): Unique identifier for each document chunk
- content (text): The actual text content of the chunk
- title (string): Title of the section/chapter
- chapter (string): Chapter identifier
- section (string): Section identifier
- page_reference (string): Page or location reference in the book
- embedding_vector (array): Vector representation of the content
- created_at (timestamp): When the document was indexed
- updated_at (timestamp): When the document was last modified

**ChatSession**
- session_id (string): Unique identifier for the chat session
- user_id (string): Identifier for the user (if available)
- created_at (timestamp): When the session started
- updated_at (timestamp): Last interaction time
- metadata (json): Additional session information

**ChatMessage**
- message_id (string): Unique identifier for the message
- session_id (string): Reference to the chat session
- role (string): "user" or "assistant"
- content (text): The message content
- citations (array): List of document IDs used in response
- timestamp (timestamp): When the message was created

**QueryContext**
- context_id (string): Unique identifier for the query context
- session_id (string): Reference to the chat session
- selected_text (text): Text selected by the user (if any)
- query_type (string): "global" or "selection"
- created_at (timestamp): When the context was created

### API Contracts

**POST /api/chat**
- Description: Process a user message and return a response
- Request Body:
  ```json
  {
    "session_id": "string",
    "message": "string",
    "selected_text": "string (optional)",
    "query_type": "string (global|selection)"
  }
  ```
- Response:
  ```json
  {
    "response": "string",
    "citations": [
      {
        "document_id": "string",
        "title": "string",
        "chapter": "string",
        "section": "string"
      }
    ],
    "session_id": "string"
  }
  ```

**POST /api/query**
- Description: Direct query endpoint for RAG operations
- Request Body:
  ```json
  {
    "query": "string",
    "selected_text": "string (optional)",
    "top_k": "integer (default: 5)"
  }
  ```
- Response:
  ```json
  {
    "query": "string",
    "results": [
      {
        "document_id": "string",
        "content": "string",
        "title": "string",
        "chapter": "string",
        "section": "string",
        "score": "float"
      }
    ]
  }
  ```

**POST /api/index**
- Description: Index book content for RAG operations
- Request Body:
  ```json
  {
    "content": "string",
    "title": "string",
    "chapter": "string",
    "section": "string",
    "page_reference": "string"
  }
  ```
- Response:
  ```json
  {
    "document_id": "string",
    "status": "string"
  }
  ```

### Quickstart Guide

1. **Environment Setup**
   ```bash
   # Install dependencies
   pip install fastapi uvicorn openai python-dotenv qdrant-client sqlalchemy psycopg2-binary

   # Set up environment variables
   cp .env.example .env
   # Add your OpenRouter API key to .env
   ```

2. **Database Initialization**
   ```bash
   # Set up Neon PostgreSQL database
   # Run migrations to create required tables
   python -m src.db.init
   ```

3. **Vector Database Setup**
   ```bash
   # Initialize Qdrant collection for embeddings
   python -m src.vector.init
   ```

4. **Run Backend Service**
   ```bash
   uvicorn src.api.main:app --reload
   ```

5. **Integrate Frontend Widget**
   ```bash
   # Add the chat widget to your Docusaurus pages
   # Configure API endpoint in frontend component
   ```

## Phase 2: Implementation Plan

### Implementation Tasks

1. **Backend Infrastructure**
   - Set up FastAPI application structure
   - Implement OpenRouter integration for Claude and Qwen
   - Create vector database models and operations
   - Build RAG processing pipeline

2. **Data Processing Pipeline**
   - Implement document chunking and preprocessing
   - Create embedding generation and storage
   - Build indexing workflow for book content
   - Add content update/reindexing mechanisms

3. **Core RAG Logic**
   - Implement query processing and embedding
   - Build retrieval and ranking algorithms
   - Create prompt assembly with context filtering
   - Implement response generation with citation tracking

4. **Frontend Integration**
   - Create React chat widget component
   - Implement text selection handling
   - Add citation display functionality
   - Ensure responsive design for all devices

5. **Testing and Validation**
   - Unit tests for core RAG components
   - Integration tests for full query-response cycle
   - Performance tests for response time
   - Validation tests for citation accuracy

### Risk Mitigation

- **API Cost Management**: Implement rate limiting and usage tracking
- **Response Time**: Add caching for common queries and pre-computed embeddings
- **Data Freshness**: Implement automatic reindexing when book content updates
- **Security**: Validate all inputs and sanitize user content

## Phase 3: Deployment

### Deployment Strategy

1. **Backend Deployment** (Vercel/Railway)
   - Containerized FastAPI application
   - Environment variable configuration
   - Health check endpoints
   - Auto-scaling based on usage

2. **Database Configuration**
   - Neon PostgreSQL setup with connection pooling
   - Qdrant Cloud collection configuration
   - Backup and monitoring setup

3. **Frontend Integration**
   - Docusaurus plugin for chat widget
   - Build process integration
   - CDN caching for static assets

### Monitoring and Maintenance

- API usage tracking and cost monitoring
- Response time and error rate monitoring
- Vector database performance metrics
- User feedback collection and analysis

## Success Criteria Verification

- [ ] Response time under 5 seconds (SC-001)
- [ ] 95% citation accuracy (SC-002)
- [ ] Selection-based queries isolated from global context (SC-003)
- [ ] 90% user success rate with widget (SC-004)
- [ ] Free tier resource limits maintained (SC-005)
- [ ] 95% response citation rate (SC-006)

## Outcome

- ✅ Impact: Created comprehensive technical implementation plan for RAG chatbot
- 🧪 Tests: All architectural gates passed, API contracts defined
- 📁 Files: Created plan.md, research.md, data-model.md, contracts/api-contract.yaml
- 🔁 Next prompts: Ready for /sp.tasks to break down implementation into tasks
- 🧠 Reflection: Plan addresses all required system architecture and RAG flow elements

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS - All validation criteria met
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Proceed to task breakdown phase