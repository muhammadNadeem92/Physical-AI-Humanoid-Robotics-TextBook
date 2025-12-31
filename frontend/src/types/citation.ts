/**
 * TypeScript interface for Citation entity
 */
export interface Citation {
  id: string;
  messageId: string;
  sourceUrl: string;
  chapter: string;
  section: string;
  snippet: string;
  pageUrl: string;
}

/**
 * TypeScript interface for Citation creation
 */
export interface CreateCitationParams {
  messageId: string;
  sourceUrl: string;
  chapter: string;
  section: string;
  snippet: string;
  pageUrl: string;
}