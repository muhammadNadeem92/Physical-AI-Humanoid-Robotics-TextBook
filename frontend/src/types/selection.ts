/**
 * TypeScript interface for TextSelection entity
 */
export interface TextSelection {
  id: string;
  sessionId: string;
  content: string;
  pageUrl: string;
  selectionStart: number;
  selectionEnd: number;
  timestamp: Date;
}

/**
 * TypeScript interface for TextSelection creation
 */
export interface CreateTextSelectionParams {
  sessionId: string;
  content: string;
  pageUrl: string;
  selectionStart: number;
  selectionEnd: number;
}