/**
 * TypeScript interface for UserAction entity
 */
export type ActionType =
  | 'text-selection'
  | 'chat-opened'
  | 'message-sent'
  | 'citation-clicked'
  | 'session-ended';

export interface UserAction {
  id: string;
  sessionId: string;
  actionType: ActionType;
  timestamp: Date;
  metadata: Record<string, any>;
}

/**
 * TypeScript interface for UserAction creation
 */
export interface CreateUserActionParams {
  sessionId: string;
  actionType: ActionType;
  metadata?: Record<string, any>;
}