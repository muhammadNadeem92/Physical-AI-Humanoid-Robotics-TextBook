# Backend API Contracts: ChatKit UI & Docusaurus Integration

## Overview
This document defines the API contracts between the frontend ChatKit UI and the backend RAG orchestration API from Part 2.

## Base Configuration
- **Base URL**: Configurable via environment variables (defaults to `/api/v1`)
- **Content-Type**: `application/json`
- **Authentication**: None required (public API)
- **Error Format**: Standard JSON error format

## Chat Query Endpoint

### POST `/chat/query`
Submit a question to the RAG system for global book queries or scoped queries.

#### Request
```json
{
  "question": "What are the key principles of humanoid robotics?",
  "session_id": "session-123",
  "module": "optional-module-name",
  "chapter": "optional-chapter-name",
  "stream": false
}
```

#### Request Parameters
- `question` (string, required): The user's question
- `session_id` (string, required): Unique session identifier
- `module` (string, optional): Module to scope the query to
- `chapter` (string, optional): Chapter to scope the query to
- `stream` (boolean, optional): Whether to stream the response (default: false)

#### Response (Success - 200)
```json
{
  "session_id": "session-123",
  "query_id": "query-456",
  "response_text": "The key principles of humanoid robotics include...",
  "citations": [
    "https://example.com/module1/chapter1#section1",
    "https://example.com/module2/chapter3#section2"
  ],
  "timestamp": "2023-12-01T10:00:00Z",
  "is_streaming": false
}
```

#### Response Fields
- `session_id` (string): Session identifier
- `query_id` (string): Unique query identifier
- `response_text` (string): The AI-generated response
- `citations` (string[]): Array of source URLs
- `timestamp` (string): ISO 8601 timestamp
- `is_streaming` (boolean): Whether response is streamed

#### Error Response (400, 500)
```json
{
  "error_code": "VALIDATION_ERROR",
  "message": "Invalid input parameters",
  "details": {
    "errors": ["Invalid question text"]
  },
  "timestamp": "2023-12-01T10:00:00Z"
}
```

## Selected Text Query Endpoint

### POST `/chat/selected-text`
Submit a question about specific selected text to the RAG system.

#### Request
```json
{
  "selected_text": "Humanoid robots are robots with physical characteristics resembling humans.",
  "question": "What does this mean?",
  "session_id": "session-123",
  "stream": false
}
```

#### Request Parameters
- `selected_text` (string, required): The text that was selected by the user
- `question` (string, required): The user's question about the selected text
- `session_id` (string, required): Unique session identifier
- `stream` (boolean, optional): Whether to stream the response (default: false)

#### Response (Success - 200)
```json
{
  "session_id": "session-123",
  "query_id": "query-456",
  "response_text": "This means that humanoid robots are designed to mimic human form and behavior...",
  "citations": [],
  "timestamp": "2023-12-01T10:00:00Z",
  "is_streaming": false
}
```

#### Notes
- Citations may be empty for selected text queries as the context is the selected text itself

## Chat History Endpoint

### GET `/chat/history/{session_id}`
Retrieve conversation history for a specific session.

#### Path Parameters
- `session_id` (string): Session identifier

#### Response (Success - 200)
```json
{
  "session_id": "session-123",
  "history": [
    {
      "query_id": "query-456",
      "response_id": "response-789",
      "question": "What are the key principles?",
      "answer": "The key principles include...",
      "timestamp": "2023-12-01T09:00:00Z",
      "query_type": "global",
      "citations": ["https://example.com/source"]
    }
  ]
}
```

#### History Item Fields
- `query_id` (string): Unique query identifier
- `response_id` (string): Unique response identifier
- `question` (string): Original user question
- `answer` (string): AI-generated response
- `timestamp` (string): ISO 8601 timestamp
- `query_type` (string): Type of query ("global", "scoped", "selected_text")
- `citations` (string[]): Array of source URLs

## Feedback Endpoint

### POST `/chat/feedback`
Submit feedback on a response.

#### Request
```json
{
  "session_id": "session-123",
  "query_id": "query-456",
  "response_id": "response-789",
  "feedback_type": "thumbs_up",
  "feedback_text": "optional feedback text"
}
```

#### Request Parameters
- `session_id` (string, required): Session identifier
- `query_id` (string, required): Query identifier
- `response_id` (string, required): Response identifier
- `feedback_type` (string, required): Type of feedback ("positive", "negative", "thumbs_up", "thumbs_down", etc.)
- `feedback_text` (string, optional): Additional feedback text

#### Response (Success - 200)
```json
{
  "message": "Feedback submitted successfully",
  "success": true
}
```

## Session Stats Endpoint

### GET `/chat/stats/{session_id}`
Get statistics for a session.

#### Path Parameters
- `session_id` (string): Session identifier

#### Response (Success - 200)
```json
{
  "session_id": "session-123",
  "total_queries": 5,
  "total_feedback": 2,
  "positive_feedback": 1,
  "negative_feedback": 1,
  "last_activity": "2023-12-01T10:00:00Z"
}
```

## Health Check Endpoint

### GET `/health`
Check the health status of the API.

#### Response (Success - 200)
```json
{
  "status": "healthy",
  "timestamp": 1698765432.123456,
  "version": "1.0.0"
}
```

## Error Response Format

All error responses follow this format:

```json
{
  "error_code": "ERROR_TYPE",
  "message": "Human-readable error message",
  "timestamp": "2023-12-01T10:00:00Z"
}
```

### Common Error Codes
- `VALIDATION_ERROR`: Input validation failed
- `CONTENT_RETRIEVAL_ERROR`: Failed to retrieve content from vector store
- `RESPONSE_GENERATION_ERROR`: Failed to generate response
- `SESSION_ERROR`: Session management error
- `INTERNAL_ERROR`: General internal server error
- `HTTP_4XX`: Client error (4xx status codes)
- `HTTP_5XX`: Server error (5xx status codes)

## Streaming Response Format

When `stream: true` is specified, the API returns Server-Sent Events (SSE) with the following format:

### Text Chunk
```json
{
  "type": "text",
  "content": "Partial response text",
  "timestamp": "2023-12-01T10:00:00Z"
}
```

### Citations Chunk
```json
{
  "type": "citations",
  "content": [
    "https://example.com/source1",
    "https://example.com/source2"
  ],
  "timestamp": "2023-12-01T10:00:00Z"
}
```

### Start Chunk
```json
{
  "type": "start",
  "content": "",
  "metadata": {
    "query_id": "query-456",
    "session_id": "session-123"
  },
  "timestamp": "2023-12-01T10:00:00Z"
}
```

### End Chunk
```json
{
  "type": "end",
  "content": "",
  "metadata": {
    "query_id": "query-456",
    "session_id": "session-123"
  },
  "timestamp": "2023-12-01T10:00:00Z"
}
```

### Error Chunk
```json
{
  "type": "error",
  "error_code": "INTERNAL_ERROR",
  "message": "An error occurred while streaming the response",
  "timestamp": "2023-12-01T10:00:00Z"
}
```