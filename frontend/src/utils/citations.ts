/**
 * Citation formatting utilities
 */
import { Citation } from '../types/citation';

/**
 * Format a citation for display
 */
export const formatCitation = (citation: Citation): string => {
  return `${citation.chapter} - ${citation.section}: ${citation.snippet.substring(0, 100)}${citation.snippet.length > 100 ? '...' : ''}`;
};

/**
 * Format citations as a list for display
 */
export const formatCitationsList = (citations: Citation[]): string[] => {
  return citations.map(citation => formatCitation(citation));
};

/**
 * Extract and format citation metadata from API response
 */
export const extractCitationsFromResponse = (responseText: string, citations: string[]): Citation[] => {
  return citations.map((url, index) => {
    // Parse the URL to extract chapter and section information
    const urlParts = new URL(url);
    const pathParts = urlParts.pathname.split('/');

    // Extract chapter and section from URL path
    let chapter = 'Unknown';
    let section = 'Unknown';

    // Look for patterns like /module/chapter/ or /chapter/section/
    if (pathParts.length >= 2) {
      for (let i = 0; i < pathParts.length; i++) {
        if (pathParts[i] === 'module' && i + 1 < pathParts.length) {
          chapter = pathParts[i + 1];
        } else if (pathParts[i] === 'chapter' && i + 1 < pathParts.length) {
          chapter = pathParts[i + 1];
        } else if (pathParts[i] === 'section' && i + 1 < pathParts.length) {
          section = pathParts[i + 1];
        }
      }
    }

    // Extract a snippet from the response text if possible
    let snippet = `Source: ${url}`;

    return {
      id: `citation_${Date.now()}_${index}`,
      messageId: `temp_message_id`, // This will be set when the citation is associated with a message
      sourceUrl: url,
      chapter,
      section,
      snippet,
      pageUrl: url,
    };
  });
};

/**
 * Validate citation format
 */
export const validateCitation = (citation: Citation): boolean => {
  return (
    !!citation.sourceUrl &&
    !!citation.chapter &&
    !!citation.section &&
    !!citation.snippet &&
    !!citation.pageUrl &&
    isValidUrl(citation.sourceUrl) &&
    isValidUrl(citation.pageUrl)
  );
};

/**
 * Validate if a string is a valid URL
 */
const isValidUrl = (urlString: string): boolean => {
  try {
    new URL(urlString);
    return true;
  } catch (err) {
    return false;
  }
};

/**
 * Format citation as HTML link
 */
export const formatCitationLink = (citation: Citation): string => {
  return `<a href="${citation.sourceUrl}" target="_blank" rel="noopener noreferrer">${citation.chapter} - ${citation.section}</a>`;
};

/**
 * Group citations by chapter
 */
export const groupCitationsByChapter = (citations: Citation[]): Record<string, Citation[]> => {
  return citations.reduce((acc, citation) => {
    if (!acc[citation.chapter]) {
      acc[citation.chapter] = [];
    }
    acc[citation.chapter].push(citation);
    return acc;
  }, {} as Record<string, Citation[]>);
};

/**
 * Get unique citations (remove duplicates)
 */
export const getUniqueCitations = (citations: Citation[]): Citation[] => {
  const seen = new Set<string>();
  return citations.filter(citation => {
    const key = `${citation.sourceUrl}-${citation.snippet}`;
    if (seen.has(key)) {
      return false;
    }
    seen.add(key);
    return true;
  });
};

/**
 * Format citations for accessibility (screen reader friendly)
 */
export const formatCitationsForAccessibility = (citations: Citation[]): string => {
  if (citations.length === 0) {
    return 'No citations provided.';
  }

  const formattedCitations = citations.map((citation, index) =>
    `Citation ${index + 1}: ${citation.chapter}, Section ${citation.section}. Source: ${citation.snippet.substring(0, 50)}${citation.snippet.length > 50 ? '...' : ''}`
  );

  return `The response is based on the following sources: ${formattedCitations.join('; ')}`;
};