# Quickstart Guide: ChatKit UI & Docusaurus Integration

## Prerequisites

### System Requirements
- Node.js v18 or higher
- npm or yarn package manager
- Git for version control

### Project Dependencies
- Docusaurus v3 installed in your project
- OpenAI Agent SDK ChatKit
- React 18
- TypeScript (if using TypeScript)

## Installation

### 1. Install Required Dependencies

```bash
npm install openai @docusaurus/core react react-dom
# or
yarn add openai @docusaurus/core react react-dom
```

### 2. Environment Configuration

Create or update your `.env` file with the backend API configuration:

```env
# Backend API base URL
BACKEND_API_BASE_URL=https://your-backend-api.com/api/v1

# For development
BACKEND_API_BASE_URL=http://localhost:8000
```

## Setup Process

### 1. Create the Chat Component

Create a new React component for the chat interface:

```typescript
// src/components/ChatWidget.tsx
import React, { useState, useEffect } from 'react';
import { useChat } from 'openai/react'; // Example ChatKit integration

interface ChatWidgetProps {
  sessionId?: string;
  selectedText?: string;
  onSessionChange?: (sessionId: string) => void;
}

const ChatWidget: React.FC<ChatWidgetProps> = ({
  sessionId: propSessionId,
  selectedText,
  onSessionChange
}) => {
  const [sessionId, setSessionId] = useState<string>(propSessionId || generateSessionId());
  const [isWidgetOpen, setIsWidgetOpen] = useState(false);

  // Initialize chat with backend API
  const { messages, input, handleInputChange, handleSubmit, isLoading } = useChat({
    api: `${process.env.BACKEND_API_BASE_URL}/chat/query`,
    body: { session_id: sessionId },
    onResponse: (response) => {
      // Handle response from backend
    },
    onError: (error) => {
      // Handle error
    }
  });

  // Update session ID when prop changes
  useEffect(() => {
    if (propSessionId) {
      setSessionId(propSessionId);
    }
  }, [propSessionId]);

  // Handle selected text mode
  useEffect(() => {
    if (selectedText) {
      // Switch to selected text mode
      // This would trigger a different API endpoint
    }
  }, [selectedText]);

  return (
    <div className={`chat-widget ${isWidgetOpen ? 'open' : 'closed'}`}>
      {!isWidgetOpen ? (
        <button
          className="chat-toggle"
          onClick={() => setIsWidgetOpen(true)}
          aria-label="Open chat"
        >
          💬 Chat
        </button>
      ) : (
        <div className="chat-container">
          <div className="chat-header">
            <h3>Textbook Assistant</h3>
            <button
              className="close-button"
              onClick={() => setIsWidgetOpen(false)}
              aria-label="Close chat"
            >
              ×
            </button>
          </div>

          <div className="chat-messages">
            {messages.map((message) => (
              <div
                key={message.id}
                className={`message ${message.role}`}
              >
                {message.content}
              </div>
            ))}
          </div>

          <form onSubmit={handleSubmit} className="chat-input-form">
            <input
              value={input}
              onChange={handleInputChange}
              placeholder="Ask about the textbook..."
              disabled={isLoading}
            />
            <button type="submit" disabled={isLoading}>
              Send
            </button>
          </form>
        </div>
      )}
    </div>
  );
};

function generateSessionId(): string {
  return `session_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
}

export default ChatWidget;
```

### 2. Create the Docusaurus Plugin

Create a Docusaurus plugin to inject the chat widget:

```typescript
// src/plugins/docusaurus-plugin-chatkit/src/index.js
const path = require('path');

module.exports = function (context, options) {
  return {
    name: 'docusaurus-plugin-chatkit',

    getClientModules() {
      return [path.resolve(__dirname, './client/chat-injector')];
    },

    configureWebpack(config, isServer, utils) {
      return {
        resolve: {
          alias: {
            '@chatkit': path.resolve(__dirname, '../components'),
          },
        },
      };
    },
  };
};
```

### 3. Create the Client Injector

```javascript
// src/plugins/docusaurus-plugin-chatkit/src/client/chat-injector.js
import React from 'react';
import { useLocation } from '@docusaurus/router';
import ChatWidget from '@chatkit/ChatWidget';

let globalSessionId = null;

function ChatInjector() {
  const location = useLocation();

  // Create or maintain session ID across page navigations
  if (!globalSessionId) {
    globalSessionId = `session_${Date.now()}`;
  }

  // Detect text selection
  const [selectedText, setSelectedText] = React.useState(null);

  React.useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      if (selection && selection.toString().trim().length > 0) {
        const text = selection.toString().trim();
        if (text.length <= 1000) { // Max length check
          setSelectedText(text);
        }
      } else {
        setSelectedText(null);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('keyup', handleSelection);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('keyup', handleSelection);
    };
  }, []);

  return (
    <ChatWidget
      sessionId={globalSessionId}
      selectedText={selectedText}
      key={location.pathname}
    />
  );
}

export default ChatInjector;
```

### 4. Register the Plugin

Update your Docusaurus config to include the plugin:

```javascript
// docusaurus.config.js
module.exports = {
  // ... other config
  plugins: [
    // ... other plugins
    './src/plugins/docusaurus-plugin-chatkit',
  ],
  // ... rest of config
};
```

## Configuration

### Environment Variables

Set the backend API URL in your environment:

```env
# Production
BACKEND_API_BASE_URL=https://api.yourtextbook.com

# Development
BACKEND_API_BASE_URL=http://localhost:8000
```

### Component Props

The ChatWidget component accepts the following props:

| Prop | Type | Description |
|------|------|-------------|
| `sessionId` | `string` | Pre-existing session ID to continue a conversation |
| `selectedText` | `string` | Text that was selected to initiate a focused conversation |
| `onSessionChange` | `(sessionId: string) => void` | Callback when session ID changes |

## Development Workflow

### 1. Start Docusaurus Development Server

```bash
npm run start
# or
yarn start
```

### 2. Test Features

1. Verify the floating chat widget appears on all pages
2. Test text selection detection
3. Verify session continuity across page navigations
4. Test API communication with the backend
5. Verify citation links work properly

### 3. Run Tests

```bash
npm run test
# or
yarn test
```

## Testing

### Unit Tests

Create unit tests for your components:

```typescript
// src/components/__tests__/ChatWidget.test.tsx
import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import ChatWidget from '../ChatWidget';

describe('ChatWidget', () => {
  test('renders chat toggle button initially', () => {
    render(<ChatWidget />);
    expect(screen.getByLabelText('Open chat')).toBeInTheDocument();
  });

  test('opens chat interface when toggle is clicked', () => {
    render(<ChatWidget />);
    fireEvent.click(screen.getByLabelText('Open chat'));
    expect(screen.getByLabelText('Close chat')).toBeInTheDocument();
  });
});
```

### Integration Tests

Test the integration with the backend API:

```typescript
// tests/api-integration.test.ts
import { test, expect } from '@playwright/test';

test.describe('Chat API Integration', () => {
  test('successfully submits a query', async ({ page }) => {
    await page.goto('/docs/intro');

    // Simulate text selection
    await page.evaluate(() => {
      const selection = window.getSelection();
      const range = document.createRange();
      const textNode = document.querySelector('p');
      if (textNode) {
        range.selectNodeContents(textNode);
        selection.removeAllRanges();
        selection.addRange(range);
      }
    });

    // Verify chat widget responds to selection
    await expect(page.locator('.chat-widget')).toBeVisible();
  });
});
```

## Deployment

### Build for Production

```bash
npm run build
# or
yarn build
```

### Environment-Specific Configuration

Ensure your production environment has the correct backend API URL:

```env
BACKEND_API_BASE_URL=https://api.production-textbook.com
```

## Troubleshooting

### Common Issues

1. **Chat widget not appearing**
   - Verify plugin is registered in `docusaurus.config.js`
   - Check browser console for JavaScript errors

2. **API calls failing**
   - Verify `BACKEND_API_BASE_URL` is set correctly
   - Check browser network tab for CORS issues

3. **Session not persisting**
   - Verify session ID is being passed correctly between components
   - Check localStorage if using it for session persistence

### Debugging Tips

1. Enable verbose logging in development
2. Use browser developer tools to inspect network requests
3. Check the console for React warnings or errors
4. Verify all environment variables are properly set