// Export all main components
export { default as ChatWidget } from './ChatWidget';
export { default as TextSelectionHandler } from './TextSelectionHandler';
export { default as SelectionOverlay } from './SelectionOverlay';
export { default as CitationDisplay } from './CitationDisplay';
export { default as MockChatKit } from './MockChatKit';

// Export context providers
export { ChatProvider, useChatContext } from '../contexts/ChatContext';
export { ConfigProvider, useConfigContext } from '../contexts/ConfigContext';