/**
 * TypeScript interface for Message entity
 */
export interface Message {
  id: string;
  sessionId: string;
  content: string;
  sender: 'user' | 'assistant';
  timestamp: Date;
  status: 'sent' | 'sending' | 'error';
  citations?: Citation[];
}

/**
 * TypeScript interface for Message creation
 */
export interface CreateMessageParams {
  sessionId: string;
  content: string;
  sender: 'user' | 'assistant';
}