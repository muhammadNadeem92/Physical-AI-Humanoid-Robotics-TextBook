# Feature Specification: ChatKit UI & Docusaurus Integration

## Overview

### Feature Description
Design and implement the frontend user interface for an embedded RAG chatbot inside a Docusaurus-based textbook using OpenAI Agent SDK ChatKit, connected to a FastAPI backend.

This part focuses ONLY on UI, UX, and frontend integration. No embedding, ingestion, or LLM logic is included.

### Scope
- Chat UI components
- Docusaurus integration
- Text selection → contextual questioning
- Chat state & session handling

### Target Runtime
- Docusaurus v3
- React 18
- TypeScript
- OpenAI Agent SDK ChatKit (UI layer only)

### Inputs
- Docusaurus markdown pages in /frontend/docs
- Backend chat APIs (already defined in Part 2)
- Book structure (modules, chapters, sections)

## User Scenarios & Testing

### Primary User Scenarios

1. **Floating Chat Interaction**
   - User sees a floating chat widget on any documentation page
   - User clicks the widget to open the chat interface
   - User asks questions about the textbook content
   - User receives contextual responses with source citations

2. **Contextual Text Selection**
   - User highlights text within a documentation page
   - User sees an "Ask about this selection" action
   - User clicks the action to open chat focused on selected text
   - User asks questions specifically about the selected content
   - User receives responses restricted to the selected text context

3. **Session Continuation**
   - User continues a conversation across multiple pages
   - User's session state is maintained as they navigate
   - User can ask follow-up questions within the same session

4. **Source Transparency**
   - User sees cited sources with each response
   - User can click on citations to navigate to original content
   - User understands when responses are restricted to selected text

### Acceptance Scenarios

1. **As a reader, I want to access a chatbot from any page so that I can get help with textbook content without leaving the documentation.**

2. **As a reader, I want to ask questions about selected text so that I can get specific clarifications on content I'm reading.**

3. **As a reader, I want to see source citations so that I can verify the information and navigate to original content.**

4. **As a reader, I want the chat interface to be non-intrusive so that it doesn't disrupt my reading experience.**

5. **As a reader with accessibility needs, I want the chat interface to be keyboard and screen reader friendly.**

### Edge Cases
- User selects very large text blocks
- User selects code blocks or special formatting
- User navigates away and returns to continue session
- Network connectivity issues during chat
- Multiple concurrent selections on the same page
- Very long chat conversations

## Functional Requirements

### FR1: Chat UI Integration
- The system SHALL provide a floating chat widget available on all documentation pages
- The system SHALL provide an inline contextual chat option within documentation pages
- The system SHALL support responsive design for desktop and mobile devices
- The system SHALL integrate with OpenAI Agent SDK ChatKit for the UI layer
- The system SHALL maintain visual consistency with Docusaurus theme

### FR2: Text Selection Mode
- The system SHALL detect text selection within documentation pages
- The system SHALL display an "Ask about this selection" action when text is selected
- The system SHALL send selected text to backend via `/chat/selected-text` endpoint
- The system SHALL enforce selected-text-only mode when initiated from text selection
- The system SHALL handle selections of varying lengths (minimum 1 word, maximum 1000 words)

### FR3: Chat Behavior
- The system SHALL maintain chat session state using session_id
- The system SHALL support follow-up questions within the same session
- The system SHALL stream responses using ChatKit streaming support
- The system SHALL display loading, typing, and error states appropriately
- The system SHALL preserve conversation context across page navigations within the same session

### FR4: Source Transparency
- The system SHALL display cited source snippets returned by the backend
- The system SHALL provide clickable links to original content (chapter, section, page URL)
- The system SHALL clearly indicate when answers are restricted to selected text
- The system SHALL format citations in a user-friendly manner

### FR5: Configuration & Environment
- The system SHALL read backend API base URL from environment variables
- The system SHALL support development and production environments
- The system SHALL implement CORS-compatible communication with FastAPI backend
- The system SHALL handle API endpoint configuration dynamically

### FR6: UI/UX Constraints
- The system SHALL implement a non-intrusive floating widget design
- The system SHALL be accessible (keyboard + screen reader friendly)
- The system SHALL minimize visual impact on reading experience
- The system SHALL maintain consistency with Docusaurus theme

## Non-Functional Requirements

### Performance
- The chat interface SHALL load within 2 seconds on standard connections
- Text selection detection SHALL respond within 100ms
- Chat responses SHALL begin streaming within 1-2 seconds of submission

### Usability
- The interface SHALL follow accessibility standards (WCAG 2.1 AA)
- The interface SHALL support keyboard navigation
- The interface SHALL provide clear visual feedback for all interactions

### Compatibility
- The system SHALL work with modern browsers (Chrome, Firefox, Safari, Edge)
- The system SHALL support Docusaurus v3
- The system SHALL maintain compatibility with React 18 and TypeScript

## Key Entities

### Session
- session_id: Unique identifier for chat session
- created_at: Timestamp when session started
- last_interaction: Timestamp of last message

### Message
- message_id: Unique identifier for message
- content: Text content of message
- sender: "user" or "assistant"
- timestamp: When message was sent/received
- citations: Array of source references

### Citation
- source_url: URL to original content
- chapter: Chapter reference
- section: Section reference
- snippet: Excerpt from source

## Assumptions

1. Backend APIs from Part 2 are available and functional
2. Docusaurus v3 is already configured with React 18 and TypeScript
3. OpenAI Agent SDK ChatKit is available for UI implementation
4. Users have JavaScript enabled in their browsers
5. The documentation structure follows standard Docusaurus conventions
6. Network connectivity is available for API communication

## Dependencies

1. Docusaurus v3 framework
2. React 18 runtime
3. TypeScript compilation
4. OpenAI Agent SDK ChatKit
5. FastAPI backend from Part 2
6. Browser JavaScript support

## Success Criteria

### Quantitative Metrics
- 95% of users can access the chat interface within 3 seconds of page load
- 90% of text selection interactions successfully trigger the contextual chat
- 98% of chat sessions maintain state across page navigations
- 95% of source citations are properly displayed and clickable
- Page load time impact is less than 100ms after integration

### Qualitative Measures
- Users report high satisfaction with chat accessibility features
- Users find the floating widget non-intrusive during reading
- Users successfully complete contextual questioning tasks
- Users can easily identify source citations in responses
- Users can seamlessly continue conversations across different documentation pages

### Business Outcomes
- Documentation engagement time increases by 15%
- User support requests decrease by 20%
- User comprehension of complex topics improves
- User satisfaction with documentation experience increases

## Constraints

### Technical Constraints
- Must integrate with existing Docusaurus v3 setup
- Must use TypeScript for type safety
- Must maintain compatibility with existing documentation structure
- Must follow Docusaurus theming guidelines

### UX Constraints
- Floating widget must not obstruct primary content
- Text selection should not interfere with normal reading
- Chat interface must be accessible to all users
- Performance impact on page load must be minimal

### Scope Constraints
- No embedding or ingestion logic to be implemented
- No LLM processing at the frontend level
- Focus only on UI and UX integration
- Backend APIs are provided and assumed functional