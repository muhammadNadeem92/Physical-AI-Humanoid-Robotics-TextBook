/**
 * TypeScript interface for Session entity
 */
export interface Session {
  id: string;
  createdAt: Date;
  lastInteraction: Date;
  isActive: boolean;
  selectedText?: string | null;
}

/**
 * TypeScript interface for Session creation
 */
export interface CreateSessionParams {
  selectedText?: string;
}