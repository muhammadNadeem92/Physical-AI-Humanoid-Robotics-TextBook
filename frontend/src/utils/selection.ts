/**
 * Text selection utilities
 */

/**
 * Get the currently selected text in the document
 */
export const getSelectedText = (): string => {
  return window.getSelection()?.toString() || '';
};

/**
 * Get the current selection range
 */
export const getSelectionRange = (): Range | null => {
  const selection = window.getSelection();
  if (!selection || selection.rangeCount === 0) {
    return null;
  }
  return selection.getRangeAt(0);
};

/**
 * Check if there is a valid text selection
 */
export const hasValidSelection = (): boolean => {
  const selectedText = getSelectedText();
  return selectedText.trim().length > 0;
};

/**
 * Get selection context (surrounding text)
 */
export const getSelectionContext = (range: Range, contextLength: number = 50): { before: string; selected: string; after: string } => {
  const selectedText = range.toString();

  // Create ranges for text before and after selection
  const beforeRange = range.cloneRange();
  beforeRange.setStart(range.startContainer, 0);
  const beforeText = beforeRange.toString().slice(-contextLength);

  const afterRange = range.cloneRange();
  afterRange.setEnd(range.endContainer, range.endContainer.textContent?.length || 0);
  const afterText = afterRange.toString().substring(range.toString().length).substring(0, contextLength);

  return {
    before: beforeText,
    selected: selectedText,
    after: afterText
  };
};

/**
 * Highlight selected text with a temporary style
 */
export const highlightSelection = (className: string = 'selection-highlight'): HTMLElement | null => {
  const range = getSelectionRange();
  if (!range) return null;

  // Create a temporary element to wrap the selection
  const highlightElement = document.createElement('span');
  highlightElement.className = className;
  highlightElement.style.backgroundColor = 'yellow';
  highlightElement.style.opacity = '0.5';

  // Wrap the selection with the highlight element
  range.surroundContents(highlightElement);

  return highlightElement;
};

/**
 * Remove highlights from the document
 */
export const removeHighlights = (className: string = 'selection-highlight'): void => {
  const highlights = document.querySelectorAll(`.${className}`);
  highlights.forEach(highlight => {
    const parent = highlight.parentNode;
    if (parent) {
      while (highlight.firstChild) {
        parent.insertBefore(highlight.firstChild, highlight);
      }
      parent.removeChild(highlight);
    }
  });
};

/**
 * Get the bounding rectangle of the current selection
 */
export const getSelectionRect = (): DOMRect | null => {
  const range = getSelectionRange();
  if (!range) return null;

  return range.getBoundingClientRect();
};

/**
 * Check if selection is within a specific element
 */
export const isSelectionInElement = (element: HTMLElement): boolean => {
  const range = getSelectionRange();
  if (!range) return false;

  return element.contains(range.startContainer) && element.contains(range.endContainer);
};

/**
 * Get all selected text nodes within a range
 */
export const getSelectedTextNodes = (range: Range): Node[] => {
  const nodes: Node[] = [];
  const walker = document.createTreeWalker(
    range.commonAncestorContainer,
    NodeFilter.SHOW_TEXT,
    {
      acceptNode: (node) => {
        if (range.intersectsNode(node)) {
          return NodeFilter.FILTER_ACCEPT;
        }
        return NodeFilter.FILTER_REJECT;
      }
    }
  );

  let node;
  while (node = walker.nextNode()) {
    if (range.intersectsNode(node)) {
      nodes.push(node);
    }
  }

  return nodes;
};