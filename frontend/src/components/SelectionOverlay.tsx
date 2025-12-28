import React, { useState, useEffect, useRef } from 'react';

interface SelectionOverlayProps {
  isVisible: boolean;
  position: { top: number; left: number };
  onAskClick: () => void;
  selectedText: string;
}

const SelectionOverlay: React.FC<SelectionOverlayProps> = ({
  isVisible,
  position,
  onAskClick,
  selectedText
}) => {
  const [showPreview, setShowPreview] = useState(false);
  const overlayRef = useRef<HTMLDivElement>(null);

  // Handle click outside to close preview
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (overlayRef.current && !overlayRef.current.contains(event.target as Node)) {
        setShowPreview(false);
      }
    };

    if (showPreview) {
      document.addEventListener('mousedown', handleClickOutside);
    }

    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [showPreview]);

  if (!isVisible) {
    return null;
  }

  return (
    <div
      ref={overlayRef}
      className="selection-overlay"
      style={{
        position: 'absolute',
        top: `${position.top}px`,
        left: `${position.left}px`,
        transform: 'translateX(-50%)',
        zIndex: 10000,
        display: 'flex',
        gap: '4px',
      }}
    >
      <button
        className="selection-overlay-button"
        style={{
          backgroundColor: '#4f46e5',
          color: 'white',
          border: 'none',
          borderRadius: '4px',
          padding: '8px 12px',
          fontSize: '14px',
          cursor: 'pointer',
          boxShadow: '0 4px 6px rgba(0, 0, 0, 0.1)',
        }}
        onClick={onAskClick}
        onMouseEnter={() => setShowPreview(true)}
        aria-label="Ask about this selection"
      >
        Ask about this
      </button>

      {showPreview && selectedText && (
        <div
          className="selection-preview"
          style={{
            position: 'absolute',
            top: '-120px',
            left: '50%',
            transform: 'translateX(-50%)',
            width: '300px',
            backgroundColor: 'white',
            border: '1px solid #e5e7eb',
            borderRadius: '8px',
            padding: '12px',
            boxShadow: '0 10px 15px rgba(0, 0, 0, 0.1)',
            zIndex: 10001,
            fontSize: '14px',
            lineHeight: '1.4',
          }}
        >
          <div
            style={{
              position: 'absolute',
              bottom: '-10px',
              left: '50%',
              transform: 'translateX(-50%)',
              width: 0,
              height: 0,
              borderLeft: '10px solid transparent',
              borderRight: '10px solid transparent',
              borderTop: '10px solid white',
            }}
          />
          <strong>Selected text:</strong>
          <div
            style={{
              marginTop: '8px',
              fontStyle: 'italic',
              color: '#6b7280',
              maxHeight: '80px',
              overflow: 'hidden',
            }}
          >
            {selectedText.length > 100 ? `${selectedText.substring(0, 100)}...` : selectedText}
          </div>
        </div>
      )}
    </div>
  );
};

export default SelectionOverlay;