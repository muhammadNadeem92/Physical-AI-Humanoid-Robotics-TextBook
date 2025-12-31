import React, { useEffect, useRef, useState } from 'react';
import { useChatContext } from '../contexts/ChatContext';
import { useConfigContext } from '../contexts/ConfigContext';
import { createTextSelection, validateSelectedTextWithMessage } from '../utils/selection';

interface TextSelectionHandlerProps {
  children: React.ReactNode;
}

const TextSelectionHandler: React.FC<TextSelectionHandlerProps> = ({ children }) => {
  const { currentSession, setSelectedText } = useChatContext();
  const { config } = useConfigContext();
  const [showSelectionOverlay, setShowSelectionOverlay] = useState(false);
  const [overlayPosition, setOverlayPosition] = useState({ top: 0, left: 0 });
  const [selectedText, setSelectedTextState] = useState('');
  const overlayRef = useRef<HTMLDivElement>(null);

  // Handle text selection
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      if (selection && selection.toString().trim().length > 0) {
        const text = selection.toString().trim();

        // Validate the selected text
        const validation = validateSelectedTextWithMessage(text, config.maxSelectedTextLength);

        if (validation.isValid) {
          setSelectedTextState(text);

          // Get the position for the overlay
          const range = selection.getRangeAt(0);
          const rect = range.getBoundingClientRect();

          // Position the overlay above the selected text
          setOverlayPosition({
            top: rect.top + window.scrollY - 40, // 40px above the selection
            left: rect.left + window.scrollX + rect.width / 2 // Centered horizontally
          });

          setShowSelectionOverlay(true);
        } else {
          setShowSelectionOverlay(false);
          setSelectedTextState('');
        }
      } else {
        setShowSelectionOverlay(false);
        setSelectedTextState('');
      }
    };

    // Add event listeners
    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('keyup', handleSelection);

    // Cleanup event listeners
    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('keyup', handleSelection);
    };
  }, [config.maxSelectedTextLength]);

  // Handle clicking the "Ask about this selection" button
  const handleAskAboutSelection = () => {
    if (selectedText && currentSession) {
      // Set the selected text in the context
      setSelectedText(selectedText);

      // Hide the overlay
      setShowSelectionOverlay(false);
    }
  };

  // Handle clicking outside the overlay to close it
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (overlayRef.current && !overlayRef.current.contains(event.target as Node)) {
        setShowSelectionOverlay(false);
      }
    };

    if (showSelectionOverlay) {
      document.addEventListener('mousedown', handleClickOutside);
    }

    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [showSelectionOverlay]);

  return (
    <>
      {children}

      {showSelectionOverlay && (
        <div
          ref={overlayRef}
          className="text-selection-overlay"
          style={{
            position: 'absolute',
            top: `${overlayPosition.top}px`,
            left: `${overlayPosition.left}px`,
            transform: 'translateX(-50%)',
            zIndex: 10000,
            backgroundColor: '#4f46e5',
            color: 'white',
            padding: '8px 12px',
            borderRadius: '4px',
            fontSize: '14px',
            cursor: 'pointer',
            boxShadow: '0 4px 6px rgba(0, 0, 0, 0.1)',
            whiteSpace: 'nowrap',
          }}
          onClick={handleAskAboutSelection}
          role="button"
          tabIndex={0}
          aria-label="Ask about this selection"
          onKeyDown={(e) => {
            if (e.key === 'Enter' || e.key === ' ') {
              handleAskAboutSelection();
            }
          }}
        >
          Ask about this
        </div>
      )}
    </>
  );
};

export default TextSelectionHandler;