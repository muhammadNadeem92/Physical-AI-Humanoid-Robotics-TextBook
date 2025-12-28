# Data Model: Embedded RAG Chatbot – Part 2: RAG Orchestration & Chat APIs

**Feature**: 011-rag-orchestration-api
**Date**: 2025-12-25

## ChatSession

Represents a user's conversation session with metadata like session_id, start_time, and user_id.

**Fields**:
- `id` (string): Unique identifier for the session (UUID)
- `session_id` (string): Public session identifier for API usage
- `user_id` (string, optional): User identifier if authenticated
- `start_time` (timestamp): When the session was created
- `last_activity` (timestamp): When the last message was processed
- `active` (boolean): Whether the session is currently active
- `created_at` (timestamp): When this record was created
- `updated_at` (timestamp): When this record was last updated

**Validation Rules**:
- Session ID must be unique
- Start time must be before or equal to last activity
- Active sessions must have recent activity (less than 24 hours)

## UserQuery

Represents a user's question with text content, scope filters, and session context.

**Fields**:
- `id` (string): Unique identifier for the query (UUID)
- `session_id` (string): Reference to the ChatSession
- `question_text` (text): The actual question text from the user
- `scope_type` (string): Type of scope ('entire_book', 'module', 'chapter', 'selected_text')
- `scope_filters` (json, optional): Additional filters for module/chapter scope
- `selected_text` (text, optional): Text selected by user for selected-text mode
- `timestamp` (timestamp): When the query was submitted
- `created_at` (timestamp): When this record was created

**Validation Rules**:
- Scope type must be one of the allowed values
- For module/chapter scopes, scope_filters must be provided
- For selected-text mode, selected_text must be provided
- Question text must not be empty

## RetrievedChunk

Represents content chunks retrieved from Qdrant with text, source metadata (module, chapter, section, page URL), and relevance score.

**Fields**:
- `id` (string): Unique identifier for the chunk (UUID)
- `query_id` (string): Reference to the UserQuery that triggered retrieval
- `content` (text): The actual content text from the textbook
- `module` (string): Module name from the textbook structure
- `chapter` (string): Chapter name from the textbook structure
- `section` (string): Section name from the textbook structure
- `page_url` (string): URL path for the page containing this content
- `relevance_score` (float): Similarity score from vector search (0.0 to 1.0)
- `chunk_id` (string): Original ID from Qdrant vector database
- `token_count` (integer): Number of tokens in the content
- `created_at` (timestamp): When this record was created

**Validation Rules**:
- Relevance score must be between 0.0 and 1.0
- Module, chapter, section, and page_url must not be empty
- Content must not be empty

## ChatResponse

Represents the system's response with answer text, cited source snippets, and page references.

**Fields**:
- `id` (string): Unique identifier for the response (UUID)
- `query_id` (string): Reference to the UserQuery that generated this response
- `answer_text` (text): The generated answer text from the LLM
- `source_citations` (json): Array of source citations with page URLs and snippets
- `response_time_ms` (integer): Time taken to generate the response in milliseconds
- `token_usage` (json): Token usage information (input, output, total)
- `grounding_confidence` (float): Confidence score for how well the response is grounded in retrieved content
- `timestamp` (timestamp): When the response was generated
- `created_at` (timestamp): When this record was created

**Validation Rules**:
- Answer text must not be empty
- Source citations must contain valid page URLs
- Response time must be positive
- Grounding confidence must be between 0.0 and 1.0

## Feedback

Represents user feedback on responses (thumbs up/down) with associated session and query.

**Fields**:
- `id` (string): Unique identifier for the feedback (UUID)
- `session_id` (string): Reference to the ChatSession
- `query_id` (string): Reference to the UserQuery
- `response_id` (string): Reference to the ChatResponse
- `feedback_type` (string): Type of feedback ('thumbs_up', 'thumbs_down', 'report_inaccurate')
- `feedback_text` (text, optional): Additional feedback text from user
- `timestamp` (timestamp): When the feedback was submitted
- `created_at` (timestamp): When this record was created

**Validation Rules**:
- Feedback type must be one of the allowed values
- Query ID and response ID must reference existing records

## TokenUsageLog

Represents logging of token usage and other operational metrics for the session.

**Fields**:
- `id` (string): Unique identifier for the log entry (UUID)
- `session_id` (string): Reference to the ChatSession
- `query_id` (string, optional): Reference to specific query if applicable
- `operation_type` (string): Type of operation ('query', 'retrieval', 'generation', 'other')
- `input_tokens` (integer): Number of input tokens processed
- `output_tokens` (integer): Number of output tokens generated
- `total_tokens` (integer): Total tokens (input + output)
- `timestamp` (timestamp): When the operation occurred
- `created_at` (timestamp): When this record was created

**Validation Rules**:
- Token counts must be non-negative
- Operation type must be one of the allowed values
- Total tokens must equal input + output tokens

## Relationships

- ChatSession 1 ←→ * UserQuery (one session to many queries)
- UserQuery 1 ←→ 1 ChatResponse (one query to one response)
- UserQuery 1 ←→ * RetrievedChunk (one query to many retrieved chunks)
- UserQuery 1 ←→ * Feedback (one query to many feedback entries)
- ChatSession 1 ←→ * TokenUsageLog (one session to many token logs)
- ChatResponse 1 ←→ * Feedback (one response to many feedback entries)