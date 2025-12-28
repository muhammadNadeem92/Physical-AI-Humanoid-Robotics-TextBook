import React from 'react';
import { Citation } from '../types/citation';
import { formatCitationsForAccessibility } from '../utils/citations';

interface CitationDisplayProps {
  citations: Citation[];
  showChapterSection?: boolean;
  showSnippet?: boolean;
}

const CitationDisplay: React.FC<CitationDisplayProps> = ({
  citations,
  showChapterSection = true,
  showSnippet = false
}) => {
  if (!citations || citations.length === 0) {
    return null;
  }

  return (
    <div
      className="citations-display"
      aria-label={formatCitationsForAccessibility(citations)}
    >
      <h4 className="citations-title">Sources:</h4>
      <ul className="citations-list">
        {citations.map((citation) => (
          <li key={citation.id} className="citation-item">
            <a
              href={citation.sourceUrl}
              target="_blank"
              rel="noopener noreferrer"
              className="citation-link"
              aria-label={`Source: ${citation.chapter} - ${citation.section}, ${citation.snippet.substring(0, 50)}${citation.snippet.length > 50 ? '...' : ''}`}
            >
              {showChapterSection && (
                <span className="citation-location">
                  {citation.chapter} - {citation.section}
                </span>
              )}
              {showChapterSection && showSnippet && <span className="citation-divider">: </span>}
              {showSnippet && (
                <span className="citation-snippet">
                  {citation.snippet.substring(0, 100)}{citation.snippet.length > 100 ? '...' : ''}
                </span>
              )}
            </a>
          </li>
        ))}
      </ul>
    </div>
  );
};

export default CitationDisplay;