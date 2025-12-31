# ChatKit UI & Docusaurus Integration

This directory contains the implementation of the ChatKit UI & Docusaurus Integration feature, providing an embedded RAG chatbot inside the Physical AI & Humanoid Robotics textbook.

## Overview

The implementation provides:
- A floating chat widget available on all documentation pages
- Text selection functionality with contextual questioning
- Session state management across page navigations
- Source transparency with clickable citations
- Full accessibility support

## Architecture

### Components
- `ChatWidget`: The main chat interface with floating toggle
- `TextSelectionHandler`: Handles text selection detection and overlay
- `SelectionOverlay`: UI overlay for "Ask about this selection" functionality
- `CitationDisplay`: Component for rendering source citations
- `MockChatKit`: Mock implementation of ChatKit UI (to be replaced with actual ChatKit)

### Services
- `chatService`: Handles API communication with the backend
- `api`: Low-level API service for backend communication

### Utilities
- `session`: Session management with localStorage persistence
- `selection`: Text selection utilities and validation
- `citations`: Citation formatting and validation

### Contexts
- `ChatContext`: Global state management for chat functionality
- `ConfigContext`: Configuration management with environment variable support

### Types
- TypeScript interfaces for all entities (Session, Message, Citation, etc.)

## Docusaurus Integration

The chat functionality is integrated into Docusaurus via a custom plugin located at `src/plugins/docusaurus-plugin-chatkit/`. The plugin:
- Injects the chat widget into all documentation pages
- Handles text selection detection
- Manages session persistence across page navigations
- Provides configuration options for customization

## Environment Configuration

The following environment variables can be configured:

- `BACKEND_API_BASE_URL`: Base URL for the backend API
- `CHATKIT_WIDGET_POSITION`: Position of the chat widget (default: 'bottom-right')
- `CHATKIT_SHOW_ON_MOBILE`: Whether to show widget on mobile (default: true)
- `CHATKIT_SESSION_TIMEOUT`: Session timeout in minutes (default: 30)
- `CHATKIT_MAX_SELECTED_TEXT_LENGTH`: Max length of selected text (default: 1000)

## API Integration

The chat interface communicates with the backend RAG orchestration API through the following endpoints:
- `POST /chat/query` - For global book queries
- `POST /chat/selected-text` - For selected text queries
- `GET /chat/history/{session_id}` - For chat history
- `POST /chat/feedback` - For feedback submission
- `GET /chat/stats/{session_id}` - For session statistics

## Accessibility Features

The implementation includes:
- Keyboard navigation support
- Screen reader compatibility
- Proper ARIA labels and roles
- Focus management
- Semantic HTML structure

## Responsive Design

The chat widget is fully responsive and works on:
- Desktop (default positioning)
- Mobile devices (configurable display)
- All screen sizes with appropriate layout adjustments

## Development

To run the implementation locally with Docusaurus:

1. Navigate to the `frontend/` directory
2. Install dependencies: `npm install`
3. Start the development server: `npm run start`
4. The chat widget will be available on all documentation pages

## Testing

The implementation follows the user stories defined in the specification:
- US1: Floating Chat Interaction
- US2: Contextual Text Selection
- US3: Session Continuation
- US4: Source Transparency

Each component has been implemented to meet the acceptance criteria defined in the specification.