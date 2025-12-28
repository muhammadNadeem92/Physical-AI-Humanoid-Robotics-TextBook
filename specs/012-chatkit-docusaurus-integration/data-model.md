# Data Model: ChatKit UI & Docusaurus Integration

## Session Entity

### Fields
- `id: string` - Unique identifier for the chat session
- `createdAt: Date` - Timestamp when session was created
- `lastInteraction: Date` - Timestamp of last message in session
- `isActive: boolean` - Whether the session is currently active
- `selectedText: string | null` - Text that was selected when session started (if any)

### Relationships
- One-to-many with Message entities
- Associated with current page context

### Validation Rules
- `id` must be a valid UUID or unique string
- `createdAt` must be in the past
- `lastInteraction` must be >= `createdAt`
- `isActive` defaults to true when created

## Message Entity

### Fields
- `id: string` - Unique identifier for the message
- `sessionId: string` - Reference to parent session
- `content: string` - Text content of the message
- `sender: 'user' | 'assistant'` - Who sent the message
- `timestamp: Date` - When the message was sent/received
- `status: 'sent' | 'sending' | 'error'` - Current status of the message
- `citations: Citation[]` - Array of source citations (for assistant messages)

### Relationships
- Belongs to one Session
- Contains multiple Citation entities (for assistant responses)

### Validation Rules
- `id` must be unique within the session
- `sessionId` must reference an existing session
- `content` must not be empty
- `sender` must be either 'user' or 'assistant'
- `timestamp` must be in chronological order within session

## Citation Entity

### Fields
- `id: string` - Unique identifier for the citation
- `messageId: string` - Reference to parent message
- `sourceUrl: string` - URL to the original content
- `chapter: string` - Chapter reference
- `section: string` - Section reference
- `snippet: string` - Excerpt from the source
- `pageUrl: string` - Full page URL where the content is located

### Relationships
- Belongs to one Message
- Points to external documentation content

### Validation Rules
- `sourceUrl` must be a valid URL
- `chapter` and `section` must not be empty
- `snippet` must have content
- `pageUrl` must be a valid URL

## Configuration Entity

### Fields
- `apiBaseUrl: string` - Base URL for backend API calls
- `sessionTimeout: number` - Session timeout in minutes
- `widgetPosition: 'bottom-right' | 'bottom-left' | 'top-right' | 'top-left'` - Position of floating widget
- `showWidgetOnMobile: boolean` - Whether to show widget on mobile devices
- `maxSelectedTextLength: number` - Maximum length of selected text to process

### Validation Rules
- `apiBaseUrl` must be a valid URL
- `sessionTimeout` must be positive number
- `maxSelectedTextLength` must be positive number

## TextSelection Entity

### Fields
- `id: string` - Unique identifier for the selection
- `sessionId: string` - Reference to associated session
- `content: string` - The selected text content
- `pageUrl: string` - URL of the page where text was selected
- `selectionStart: number` - Start position of selection in the text
- `selectionEnd: number` - End position of selection in the text
- `timestamp: Date` - When the selection was made

### Relationships
- Associated with one Session
- Associated with one page context

### Validation Rules
- `content` must not be empty
- `pageUrl` must be a valid URL
- `selectionStart` must be >= 0
- `selectionEnd` must be >= `selectionStart`
- `selectionEnd` must be <= length of content

## UserAction Entity

### Fields
- `id: string` - Unique identifier for the action
- `sessionId: string` - Reference to associated session
- `actionType: 'text-selection' | 'chat-opened' | 'message-sent' | 'citation-clicked' | 'session-ended'` - Type of action
- `timestamp: Date` - When the action occurred
- `metadata: Record<string, any>` - Additional context-specific data

### Relationships
- Associated with one Session

### Validation Rules
- `actionType` must be one of the allowed values
- `timestamp` must be in chronological order within session

## State Management

### Global State Structure
```
{
  currentSession: Session | null,
  messages: Message[],
  textSelection: TextSelection | null,
  configuration: Configuration,
  uiState: {
    isWidgetOpen: boolean,
    isLoading: boolean,
    error: string | null,
    selectedText: string | null
  }
}
```

### Validation Rules
- `currentSession` must be null or a valid Session
- `messages` array must be sorted by timestamp
- `textSelection` must be null or a valid TextSelection
- `configuration` must be a valid Configuration object
- `uiState` properties must be of correct types