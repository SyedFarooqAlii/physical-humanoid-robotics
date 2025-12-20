# Data Model: RAG Chatbot for Physical AI Book

## Entity: BookContentDocument

**Description**: Represents a chunk of book content that has been processed for RAG operations

**Fields**:
- `document_id` (string, required): Unique identifier for each document chunk, UUID format
- `content` (text, required): The actual text content of the chunk, max 10000 characters
- `title` (string, required): Title of the section/chapter this chunk belongs to
- `chapter` (string, required): Chapter identifier from the book structure
- `section` (string, optional): Section identifier within the chapter
- `page_reference` (string, optional): Page number or location reference in the book
- `embedding_vector` (array, required): Vector representation of the content for semantic search
- `created_at` (timestamp, required): When the document was indexed
- `updated_at` (timestamp, required): When the document was last modified

**Validation Rules**:
- `document_id` must be unique across all documents
- `content` must be between 50 and 10000 characters
- `chapter` must match existing book chapter structure
- `embedding_vector` must have fixed dimensionality (1536 for Qwen embeddings)

**Relationships**:
- Referenced by ChatMessage entities in citations array

## Entity: ChatSession

**Description**: Represents a conversation session between a user and the chatbot

**Fields**:
- `session_id` (string, required): Unique identifier for the chat session, UUID format
- `user_id` (string, optional): Identifier for the user (if authenticated)
- `created_at` (timestamp, required): When the session started
- `updated_at` (timestamp, required): Last interaction time
- `metadata` (json, optional): Additional session information (preferences, etc.)

**Validation Rules**:
- `session_id` must be unique
- `created_at` must be before `updated_at`
- `user_id` format must match authentication system format

**Relationships**:
- Contains multiple ChatMessage entities
- Associated with multiple QueryContext entities

## Entity: ChatMessage

**Description**: Represents a single message in a chat conversation

**Fields**:
- `message_id` (string, required): Unique identifier for the message, UUID format
- `session_id` (string, required): Reference to the chat session
- `role` (string, required): "user" or "assistant"
- `content` (text, required): The message content
- `citations` (array, optional): List of document IDs used in response
- `timestamp` (timestamp, required): When the message was created
- `query_context_id` (string, optional): Reference to the query context used

**Validation Rules**:
- `role` must be either "user" or "assistant"
- `citations` must reference valid BookContentDocument IDs
- `session_id` must reference an existing ChatSession

**Relationships**:
- Belongs to one ChatSession
- References multiple BookContentDocument entities via citations
- Associated with one QueryContext entity

## Entity: QueryContext

**Description**: Represents the context for a specific query (global vs selection-based)

**Fields**:
- `context_id` (string, required): Unique identifier for the query context, UUID format
- `session_id` (string, required): Reference to the chat session
- `selected_text` (text, optional): Text selected by the user (if any), max 5000 characters
- `query_type` (string, required): "global" or "selection"
- `created_at` (timestamp, required): When the context was created

**Validation Rules**:
- `query_type` must be either "global" or "selection"
- If `query_type` is "selection", then `selected_text` must be provided
- `selected_text` must be between 10 and 5000 characters if provided

**Relationships**:
- Belongs to one ChatSession
- Referenced by ChatMessage entities

## State Transitions

### ChatSession
- **Created**: When a new conversation starts
- **Active**: When messages are being exchanged
- **Inactive**: When no activity for 30 minutes
- **Closed**: When explicitly ended by user or after 24 hours of inactivity

### QueryContext
- **Active**: When created for a query
- **Used**: When referenced by a ChatMessage
- **Expired**: After 1 hour of inactivity

## Indexes

### BookContentDocument
- Primary: `document_id`
- Secondary: `chapter`, `section` for efficient content retrieval
- Vector: `embedding_vector` for semantic search operations

### ChatSession
- Primary: `session_id`
- Secondary: `user_id`, `updated_at` for session management

### ChatMessage
- Primary: `message_id`
- Secondary: `session_id`, `timestamp` for conversation ordering
- Foreign: Index on `query_context_id`

### QueryContext
- Primary: `context_id`
- Secondary: `session_id`, `created_at` for context management