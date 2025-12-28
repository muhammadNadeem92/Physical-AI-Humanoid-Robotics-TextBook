# Feature Specification: Embedded RAG Chatbot – Part 2: RAG Orchestration & Chat APIs

**Feature Branch**: `011-rag-orchestration-api`
**Created**: 2025-12-25
**Status**: Draft
**Input**: User description: "Embedded RAG Chatbot – Part 2: RAG Orchestration & Chat APIs

Objective:
Design and implement the backend RAG orchestration and chat API layer for an
embedded chatbot used inside a Docusaurus-based textbook on Physical AI & Humanoid Robotics.

This part connects pre-ingested embeddings (from Part 1) to a conversational AI system.

Scope:
This part includes ONLY backend runtime logic.
No frontend UI, no markdown ingestion, and no embedding generation.

Target Runtime:
Python 3.11+
uv-based project
FastAPI for HTTP APIs

Core Technologies:
- FastAPI
- OpenAI Agent SDK (Agent orchestration)
- ChatKit-compatible streaming responses
- Gemini (Google Generative AI) as the LLM provider
- Qdrant Cloud (vector retrieval)
- Neon Serverless Postgres (sessions, logs, feedback)

Responsibilities:
1. Implement RAG query pipeline:
   - Accept user queries
   - Determine retrieval scope:
     a) entire book
     b) specific module/chapter
     c) user-selected text only
2. Retrieve relevant chunks from Qdrant using Cohere embeddings
3. Inject retrieved context into Gemini prompts via Agent SDK
4. Enforce grounding rules:
   - No response without retrieved context
   - Selected-text mode strictly blocks external retrieval
5. Support conversational memory per session
6. Return answers with:
   - cited source snippets
   - page URLs
7. Stream responses in a ChatKit-compatible format

Agent Design:
- Use Agent SDK as the orchestration layer
- Gemini is used as the LLM behind the Agent
- Agent responsibilities:
  - Query understanding
  - Tool-based retrieval invocation
  - Context injection
  - Response validation against retrieved chunks

API Endpoints:
- POST /chat/query
  Input: question, optional filters (module, chapter), session_id
- POST /chat/selected-text
  Input: selected_text, question, session_id
- GET /chat/history/{session_id}
- POST /chat/feedback

Database Responsibilities (Neon Postgres):
- Store chat sessions
- Store user queries
- Store retrieved chunk IDs
- Store feedback (thumbs up/down)
- Log token usage and"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Global Book Query (Priority: P1)

As a student using the Physical AI & Humanoid Robotics textbook, I want to ask questions about the entire book content, so that I can get comprehensive answers that draw from the full textbook corpus and understand complex concepts through relevant citations.

**Why this priority**: This is the foundational capability that enables the core RAG functionality. Without this, the chatbot cannot serve as a comprehensive study aid for the textbook.

**Independent Test**: Can be fully tested by sending questions to the /chat/query endpoint without scope filters and verifying that responses include cited source snippets with page URLs from across the entire book, delivering the core capability to access textbook knowledge through conversation.

**Acceptance Scenarios**:

1. **Given** a user has a question about general textbook content, **When** they send a query to the /chat/query endpoint, **Then** the system retrieves relevant content chunks from across the entire book and returns a response with cited source snippets and page URLs
2. **Given** the user's query requires information from multiple chapters, **When** the system processes the query, **Then** it combines information from relevant chunks and provides a coherent response with proper citations

---

### User Story 2 - Scoped Content Query (Priority: P2)

As a student studying a specific module or chapter, I want to ask questions that are limited to that specific content section, so that I can focus my learning on targeted material without getting responses that reference unrelated parts of the book.

**Why this priority**: Enables focused learning by allowing students to constrain queries to specific study areas, which is essential for effective learning and review.

**Independent Test**: Can be fully tested by sending queries with module/chapter filters to the /chat/query endpoint and verifying that responses only include citations from the specified sections, delivering the capability to focus on targeted content areas.

**Acceptance Scenarios**:

1. **Given** a user wants to ask about a specific module, **When** they send a query with module filters to the /chat/query endpoint, **Then** the system retrieves only relevant chunks from that module and returns responses with citations limited to that scope
2. **Given** a user is reviewing a specific chapter, **When** they send a query with chapter filters, **Then** the system ensures responses only reference content from that chapter

---

### User Story 3 - Selected Text Query (Priority: P3)

As a student reading a specific section of the textbook, I want to ask questions about only the text I've selected, so that I can get clarifications that are strictly based on that specific content without external context.

**Why this priority**: Supports close reading and detailed understanding of specific passages, which is important for deep learning and comprehension.

**Independent Test**: Can be fully tested by sending selected text with questions to the /chat/selected-text endpoint and verifying that responses are strictly grounded in the provided text with no external retrieval, delivering the capability for focused analysis of specific content.

**Acceptance Scenarios**:

1. **Given** a user has selected specific text in the textbook, **When** they send the selected text and question to the /chat/selected-text endpoint, **Then** the system responds only based on that provided text without retrieving additional content
2. **Given** a user's selected text is insufficient to answer their question, **When** they send the query, **Then** the system acknowledges the limitation and explains that the answer cannot be provided from the selected text alone

---

### User Story 4 - Conversational Memory & History (Priority: P4)

As a student having an ongoing conversation with the chatbot, I want to maintain context across multiple questions in a session, so that I can have natural, flowing discussions about the textbook content.

**Why this priority**: Enables natural conversation flow and builds upon previous interactions, which is important for effective learning conversations.

**Independent Test**: Can be fully tested by maintaining a session_id across multiple queries and verifying that the system maintains context and can reference previous exchanges, delivering the capability for contextual conversations.

**Acceptance Scenarios**:

1. **Given** a user is in an ongoing session, **When** they ask follow-up questions, **Then** the system maintains context from previous exchanges in the session
2. **Given** a user wants to review their conversation history, **When** they call the /chat/history/{session_id} endpoint, **Then** the system returns the complete history of their interaction

---

### Edge Cases

- What happens when the Qdrant Cloud service is temporarily unavailable during retrieval?
- How does the system handle very long user queries that exceed LLM token limits?
- What if the Gemini API is rate-limited or unavailable during processing?
- How does the system handle queries in languages other than the textbook content?
- What happens when no relevant content is found in the vector database for a query?
- How does the system handle concurrent users accessing the API simultaneously?
- What if the Neon Postgres database is unavailable when storing session data?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST accept user queries via the /chat/query endpoint with question text, optional filters (module, chapter), and session_id
- **FR-002**: System MUST accept selected text queries via the /chat/selected-text endpoint with selected_text, question, and session_id
- **FR-003**: System MUST retrieve relevant content chunks from Qdrant Cloud using Cohere embeddings based on the query and scope
- **FR-004**: System MUST enforce grounding rules by requiring retrieved context for responses (no response without context)
- **FR-005**: System MUST block external retrieval when in selected-text mode (strict grounding in provided text only)
- **FR-006**: System MUST stream responses in a ChatKit-compatible format for real-time user experience
- **FR-007**: System MUST return answers with cited source snippets and page URLs from retrieved content
- **FR-008**: System MUST maintain conversational memory per session to support context-aware responses
- **FR-009**: System MUST store chat sessions, user queries, and retrieved chunk IDs in Neon Postgres
- **FR-010**: System MUST provide access to conversation history via the /chat/history/{session_id} endpoint
- **FR-011**: System MUST accept feedback via the /chat/feedback endpoint to improve response quality
- **FR-012**: System MUST log token usage and other metrics for operational monitoring
- **FR-013**: System MUST support three retrieval scopes: entire book, specific module/chapter, and user-selected text only
- **FR-014**: System MUST use OpenAI Agent SDK as the orchestration layer with Gemini as the LLM provider
- **FR-015**: System MUST validate that responses are properly grounded in retrieved content before returning to user

### Key Entities

- **ChatSession**: Represents a user's conversation session with metadata like session_id, start_time, and user_id
- **UserQuery**: Represents a user's question with text content, scope filters, and session context
- **RetrievedChunk**: Represents content chunks retrieved from Qdrant with text, source metadata (module, chapter, section, page URL), and relevance score
- **ChatResponse**: Represents the system's response with answer text, cited source snippets, and page references
- **Feedback**: Represents user feedback on responses (thumbs up/down) with associated session and query
- **TokenUsageLog**: Represents logging of token usage and other operational metrics for the session

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: The system responds to 95% of queries within 10 seconds for typical textbook questions
- **SC-002**: 90% of responses include proper source citations with page URLs from the textbook content
- **SC-003**: The system maintains conversational context across 10+ exchanges in a single session with 85% accuracy
- **SC-004**: 98% of queries result in grounded responses that properly reference retrieved content (no hallucinations)
- **SC-005**: The system handles 100 concurrent users without performance degradation
- **SC-006**: Selected-text mode responses are 100% grounded in provided text without external retrieval