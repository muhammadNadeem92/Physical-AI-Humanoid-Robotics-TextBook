---
id: 1
title: chatkit-ui-docusaurus-integration-spec
stage: spec
date_iso: 2025-12-26
model: Claude Sonnet 4.5
surface: agent
feature: chatkit-docusaurus-integration
branch: 012-chatkit-docusaurus-integration
user: Claude Code User
command: /sp.specify
labels: ["spec", "ui", "docusaurus", "chatkit", "integration"]
links:
  spec: "specs/012-chatkit-docusaurus-integration/spec.md"
  ticket: null
  adr: null
  pr: null
files_yaml:
  - "specs/012-chatkit-docusaurus-integration/spec.md"
  - "specs/012-chatkit-docusaurus-integration/checklists/requirements.md"
tests_yaml: []
prompt_text: |
  Embedded RAG Chatbot – Part 3: ChatKit UI & Docusaurus Integration

  Objective:
  Design and implement the frontend user interface for an embedded RAG chatbot
  inside a Docusaurus-based textbook using OpenAI Agent SDK ChatKit,
  connected to a FastAPI backend.

  This part focuses ONLY on UI, UX, and frontend integration.
  No embedding, ingestion, or LLM logic is included.

  Scope:
  - Chat UI components
  - Docusaurus integration
  - Text selection → contextual questioning
  - Chat state & session handling

  Target Runtime:
  - Docusaurus v3
  - React 18
  - TypeScript
  - OpenAI Agent SDK ChatKit (UI layer only)

  Inputs:
  - Docusaurus markdown pages in /frontend/docs
  - Backend chat APIs (already defined in Part 2)
  - Book structure (modules, chapters, sections)

  Responsibilities:

  1. Chat UI Integration
     - Embed ChatKit-based chatbot into Docusaurus
     - Floating chat widget available on all pages
     - Inline contextual chat option within docs pages
     - Responsive UI for desktop and mobile

  2. Text Selection Mode
     - Allow users to highlight text inside documentation pages
     - Show "Ask about this selection" action
     - Send selected text to backend via `/chat/selected-text`
     - Enforce selected-text-only mode at UI level

  3. Chat Behavior
     - Maintain chat session state (session_id)
     - Support follow-up questions within same session
     - Stream responses using ChatKit streaming support
     - Display loading, typing, and error states

  4. Source Transparency
     - Display cited source snippets returned by backend
     - Link citations to:
       - Chapter
       - Section
       - Page URL
     - Clearly indicate when answers are restricted to selected text

  5. Configuration & Environment
     - Read backend API base URL from environment variables
     - Support dev and production environments
     - CORS-compatible communication with FastAPI backend

  6. UI/UX Constraints
     - Non-intrusive floating widget
     - Accessible (keyboard + screen reader friendly)
     - Minimal visual impact on reading experience
     - Consistent with Docusaurus them
response_text: |
  Created comprehensive specification for ChatKit UI & Docusaurus Integration.
  The specification includes:

  - Overview with feature description, scope, target runtime, and inputs
  - User scenarios and acceptance criteria for floating chat, text selection, session continuation, and source transparency
  - Functional requirements covering UI integration, text selection mode, chat behavior, source transparency, configuration, and UX constraints
  - Non-functional requirements for performance, usability, and compatibility
  - Key entities (Session, Message, Citation)
  - Assumptions about backend availability, Docusaurus setup, etc.
  - Dependencies on Docusaurus, React, TypeScript, etc.
  - Success criteria with quantitative metrics (95% load times, 90% success rates) and qualitative measures (user satisfaction)
  - Technical, UX, and scope constraints

  Also created a quality checklist to validate the specification completeness.
outcome: |
  Successfully created a comprehensive specification for the ChatKit UI & Docusaurus Integration feature. The specification covers all required aspects including user scenarios, functional requirements, success criteria, and constraints. Ready for planning phase.
evaluation: |
  The specification meets all requirements with clear functional requirements, measurable success criteria, and comprehensive user scenarios. The quality checklist confirms completeness.
---