# Implementation Tasks: ChatKit UI & Docusaurus Integration

## Feature Overview
Frontend user interface for an embedded RAG chatbot inside a Docusaurus-based textbook using OpenAI Agent SDK ChatKit, connected to a FastAPI backend.

## Implementation Strategy
Implement the feature in phases following user story priority. Start with core functionality (floating chat widget) then add advanced features (text selection, session management, etc.). Each user story should be independently testable.

## Phase 1: Setup Tasks

### Goal
Initialize project structure and install dependencies needed for the ChatKit UI integration.

- [X] T001 Create src/components directory for React components
- [X] T002 Create src/plugins directory for Docusaurus plugin
- [X] T003 Create src/contexts directory for React context providers
- [X] T004 Create src/types directory for TypeScript type definitions
- [X] T005 Create src/utils directory for utility functions
- [ ] T006 Install OpenAI Agent SDK ChatKit dependency
- [ ] T007 Install additional dependencies: @docusaurus/core, react, react-dom
- [X] T008 Set up environment variable configuration for API base URL
- [X] T009 Create .env.example file with API configuration

## Phase 2: Foundational Tasks

### Goal
Implement core infrastructure components that all user stories depend on.

- [X] T010 [P] Define TypeScript interfaces for Session entity in src/types/session.ts
- [X] T011 [P] Define TypeScript interfaces for Message entity in src/types/message.ts
- [X] T012 [P] Define TypeScript interfaces for Citation entity in src/types/citation.ts
- [X] T013 [P] Define TypeScript interfaces for Configuration entity in src/types/config.ts
- [X] T014 [P] Define TypeScript interfaces for TextSelection entity in src/types/selection.ts
- [X] T015 [P] Define TypeScript interfaces for UserAction entity in src/types/action.ts
- [X] T016 [P] Create global state context with TypeScript interfaces in src/contexts/ChatContext.tsx
- [X] T017 Create API service layer to handle backend communication in src/services/api.ts
- [X] T018 Implement session management utilities in src/utils/session.ts
- [X] T019 Implement text selection utilities in src/utils/selection.ts
- [X] T020 Implement citation formatting utilities in src/utils/citations.ts
- [X] T021 Create configuration provider with environment variable handling in src/contexts/ConfigContext.tsx

## Phase 3: [US1] Floating Chat Interaction

### Goal
User sees a floating chat widget on any documentation page, clicks the widget to open the chat interface, asks questions about the textbook content, and receives contextual responses with source citations.

### Independent Test Criteria
- Floating widget appears on all documentation pages
- Widget can be opened and closed
- User can submit questions to backend
- Responses include source citations
- Widget is responsive on desktop and mobile

- [X] T022 [US1] Create ChatWidget React component with floating toggle in src/components/ChatWidget.tsx
- [X] T023 [US1] Implement ChatWidget styling with CSS modules in src/components/ChatWidget.module.css
- [X] T024 [US1] Add responsive design for desktop and mobile in src/components/ChatWidget.module.css
- [X] T025 [US1] Integrate ChatKit UI component into ChatWidget in src/components/ChatWidget.tsx
- [X] T026 [US1] Implement API communication for chat queries in src/services/chatService.ts
- [X] T027 [US1] Handle loading, typing, and error states in src/components/ChatWidget.tsx
- [X] T028 [US1] Display source citations with clickable links in src/components/ChatWidget.tsx
- [X] T029 [US1] Implement non-intrusive widget design in src/components/ChatWidget.module.css
- [X] T030 [US1] Add accessibility features (keyboard navigation, screen reader support) in src/components/ChatWidget.tsx
- [X] T031 [US1] Implement session state maintenance across page navigations in src/components/ChatWidget.tsx

## Phase 4: [US2] Contextual Text Selection

### Goal
User highlights text within a documentation page, sees an "Ask about this selection" action, clicks the action to open chat focused on selected text, asks questions specifically about the selected content, and receives responses restricted to the selected text context.

### Independent Test Criteria
- Text selection is detected within documentation pages
- "Ask about this selection" action appears when text is selected
- Selected text is sent to backend via `/chat/selected-text` endpoint
- Responses are restricted to selected text context
- Handles selections of varying lengths (1 word to 1000 words)

- [X] T032 [US2] Create text selection detection system in src/components/TextSelectionHandler.tsx
- [X] T033 [US2] Implement "Ask about this selection" UI overlay in src/components/SelectionOverlay.tsx
- [X] T034 [US2] Add text selection detection using Selection API in src/utils/selection.ts
- [X] T035 [US2] Implement selected-text-only mode enforcement in src/components/ChatWidget.tsx
- [X] T036 [US2] Create API service method for selected-text endpoint in src/services/chatService.ts
- [X] T037 [US2] Handle selected text validation (length constraints) in src/utils/selection.ts
- [X] T038 [US2] Display selected text context in chat interface in src/components/ChatWidget.tsx
- [X] T039 [US2] Ensure text selection doesn't interfere with normal reading in src/components/TextSelectionHandler.tsx
- [ ] T040 [US2] Implement handling for code blocks and special formatting in src/utils/selection.ts

## Phase 5: [US3] Session Continuation

### Goal
User continues a conversation across multiple pages, session state is maintained as they navigate, and user can ask follow-up questions within the same session.

### Independent Test Criteria
- Session state is preserved across page navigations
- Conversation context is maintained
- Follow-up questions work within the same session
- Session timeout is handled appropriately
- Session can be resumed after returning to page

- [X] T041 [US3] Implement session persistence using localStorage in src/utils/session.ts
- [X] T042 [US3] Create session timeout management in src/utils/session.ts
- [ ] T043 [US3] Implement session resumption on page load in src/components/ChatWidget.tsx
- [ ] T044 [US3] Add session state synchronization across page navigations in src/contexts/ChatContext.tsx
- [ ] T045 [US3] Handle network connectivity issues during session in src/services/api.ts
- [ ] T046 [US3] Implement session cleanup after timeout in src/utils/session.ts
- [X] T047 [US3] Add session metadata tracking (last interaction time) in src/utils/session.ts

## Phase 6: [US4] Source Transparency

### Goal
User sees cited sources with each response, can click on citations to navigate to original content, and understands when responses are restricted to selected text.

### Independent Test Criteria
- Source citations are displayed with each response
- Citation links are clickable and navigate to original content
- Clear indication when responses are restricted to selected text
- Citations are formatted in user-friendly manner
- Citation metadata (chapter, section) is displayed appropriately

- [X] T048 [US4] Create CitationDisplay component for rendering citations in src/components/CitationDisplay.tsx
- [X] T049 [US4] Implement citation link formatting with chapter/section info in src/components/CitationDisplay.tsx
- [ ] T050 [US4] Add visual indicators for selected-text-only responses in src/components/ChatWidget.tsx
- [ ] T051 [US4] Implement citation validation and error handling in src/utils/citations.ts
- [ ] T052 [US4] Create citation metadata extraction from API responses in src/utils/citations.ts
- [X] T053 [US4] Add citation accessibility features (screen reader support) in src/components/CitationDisplay.tsx

## Phase 7: Docusaurus Integration

### Goal
Integrate the chat components into the Docusaurus documentation site using a custom plugin.

### Independent Test Criteria
- Chat widget appears on all documentation pages
- Widget integrates seamlessly with Docusaurus theme
- No conflicts with existing Docusaurus functionality
- Widget works across all documentation page types

- [X] T054 Create Docusaurus plugin for chat widget injection in src/plugins/docusaurus-plugin-chatkit/src/index.js
- [X] T055 Implement client-side chat injector in src/plugins/docusaurus-plugin-chatkit/src/client/chat-injector.js
- [X] T056 Add plugin configuration options in src/plugins/docusaurus-plugin-chatkit/src/index.js
- [X] T057 Update docusaurus.config.js to include the chat plugin
- [X] T058 Ensure plugin compatibility with Docusaurus v3
- [X] T059 Test plugin integration with different Docusaurus page layouts

## Phase 8: Polish & Cross-Cutting Concerns

### Goal
Address cross-cutting concerns and polish the implementation for production.

### Independent Test Criteria
- All components meet accessibility standards (WCAG 2.1 AA)
- Performance impact is minimal (<100ms page load impact)
- Error handling is comprehensive
- Cross-browser compatibility is verified
- All user stories work together seamlessly

- [ ] T060 Conduct accessibility audit and implement fixes
- [ ] T061 Optimize performance and measure page load impact
- [ ] T062 Implement comprehensive error handling and user feedback
- [ ] T063 Add cross-browser compatibility testing and fixes
- [ ] T064 Create comprehensive test suite for all components
- [ ] T065 Update documentation with setup and usage instructions
- [ ] T066 Implement analytics and usage tracking
- [ ] T067 Add security considerations and validation
- [ ] T068 Conduct final integration testing across all user stories

## Dependencies

### User Story Completion Order
1. US1 (Floating Chat Interaction) - Foundation for all other features
2. US2 (Contextual Text Selection) - Depends on US1 for chat interface
3. US3 (Session Continuation) - Depends on US1 for session management
4. US4 (Source Transparency) - Depends on US1 for response display

### Critical Path Dependencies
- T010-T016 (Foundational types and contexts) must be completed before US1
- US1 must be completed before US2, US3, and US4
- T054-T059 (Docusaurus integration) can be done in parallel with US1-4 but needed for final deployment

## Parallel Execution Examples

### Per User Story
- **US1**: T022-T025 can be done in parallel with T026-T027
- **US2**: T032-T033 can be done in parallel with T034-T035
- **US3**: T041-T042 can be done in parallel with T043-T044
- **US4**: T048-T049 can be done in parallel with T050-T051

## MVP Scope
The MVP includes US1 (Floating Chat Interaction) which provides core functionality:
- T001-T021 (Setup and foundational tasks)
- T022-T031 (Floating chat widget with basic functionality)
- T054-T059 (Docusaurus integration)
- T060, T062, T065 (Essential polish tasks)

This MVP delivers a working chat interface that users can interact with on documentation pages.