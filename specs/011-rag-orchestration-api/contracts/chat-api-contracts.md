# Chat API Contracts

**Feature**: 011-rag-orchestration-api
**Date**: 2025-12-25

## Chat Query Endpoint

### POST /chat/query

**Input**:
- `question` (string): The question text from the user
- `session_id` (string): Session identifier for maintaining conversational context
- `module` (string, optional): Module filter for scoped queries
- `chapter` (string, optional): Chapter filter for scoped queries

**Output**:
- `response_id` (string): Unique identifier for the response
- `answer` (string): The generated answer text
- `citations` (array): Array of source citations
  - `text` (string): The cited text snippet
  - `page_url` (string): URL of the source page
  - `module` (string): Module name of the source
  - `chapter` (string): Chapter name of the source
  - `section` (string): Section name of the source
- `session_id` (string): Session identifier
- `timestamp` (timestamp): When the response was generated

**Error Conditions**:
- Invalid session ID
- Question text is empty
- No relevant content found in vector database
- Gemini API is unavailable
- Rate limiting exceeded

## Selected Text Query Endpoint

### POST /chat/selected-text

**Input**:
- `selected_text` (string): The text selected by the user
- `question` (string): The question about the selected text
- `session_id` (string): Session identifier for maintaining conversational context

**Output**:
- `response_id` (string): Unique identifier for the response
- `answer` (string): The generated answer text (strictly based on selected text)
- `session_id` (string): Session identifier
- `timestamp` (timestamp): When the response was generated

**Error Conditions**:
- Invalid session ID
- Selected text or question is empty
- Answer cannot be provided from selected text alone
- Gemini API is unavailable
- Rate limiting exceeded

## Chat History Endpoint

### GET /chat/history/{session_id}

**Input**:
- `session_id` (string): Session identifier to retrieve history for

**Output**:
- `session_id` (string): Session identifier
- `history` (array): Array of conversation history items
  - `id` (string): Unique identifier for the history item
  - `type` (string): Type of item ('user_query' or 'chat_response')
  - `content` (string): The content of the message
  - `timestamp` (timestamp): When the message was created
  - `citations` (array, optional): Citations if it's a response

**Error Conditions**:
- Invalid session ID
- Session not found
- Database connection issues

## Chat Feedback Endpoint

### POST /chat/feedback

**Input**:
- `session_id` (string): Session identifier
- `query_id` (string): Query identifier that the feedback refers to
- `response_id` (string): Response identifier that the feedback refers to
- `feedback_type` (string): Type of feedback ('thumbs_up', 'thumbs_down', 'report_inaccurate')
- `feedback_text` (string, optional): Additional feedback text

**Output**:
- `success` (boolean): Whether feedback was recorded successfully
- `feedback_id` (string): Unique identifier for the feedback record

**Error Conditions**:
- Invalid session, query, or response ID
- Invalid feedback type
- Feedback text too long
- Database connection issues

## Streaming Response Format

### Server-Sent Events (SSE) for Real-time Responses

**Event Format**:
- `data`: JSON object with response chunk
  - `type` (string): Type of data ('start', 'token', 'citations', 'end')
  - `content` (string): The content of the chunk
  - `citations` (array, optional): Citations when available
  - `timestamp` (timestamp): When this chunk was generated

**Error Conditions**:
- Connection interrupted
- Timeout during generation
- Invalid event format