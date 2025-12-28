import React from 'react';
import { useLocation } from '@docusaurus/router';
import { ChatProvider } from '../../../../src/contexts/ChatContext';
import { ConfigProvider } from '../../../../src/contexts/ConfigContext';
import ChatWidget from '../../../../src/components/ChatWidget';
import TextSelectionHandler from '../../../../src/components/TextSelectionHandler';

// Get configuration from Docusaurus
const config = typeof window !== 'undefined' ? window.chatkitConfig || {} : {};

// Default configuration
const defaultConfig = {
  apiBaseUrl: '/api/v1', // Default API base URL
  sessionTimeout: 30,
  widgetPosition: 'bottom-right',
  showWidgetOnMobile: true,
  maxSelectedTextLength: 1000,
  ...config,
};

function ChatInjector() {
  const location = useLocation();
  const [globalSessionId, setGlobalSessionId] = React.useState(null);

  // Create or maintain session ID across page navigations
  React.useEffect(() => {
    if (!globalSessionId) {
      // Generate a session ID that persists across the entire site visit
      const storedSessionId = localStorage.getItem('chatkit-session-id');
      if (storedSessionId) {
        setGlobalSessionId(storedSessionId);
      } else {
        const newSessionId = `session_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
        localStorage.setItem('chatkit-session-id', newSessionId);
        setGlobalSessionId(newSessionId);
      }
    }
  }, [globalSessionId]);

  // Reset selected text when location changes
  React.useEffect(() => {
    // This will trigger a re-render of components that depend on location
  }, [location.pathname]);

  return (
    <ConfigProvider initialConfig={defaultConfig}>
      <ChatProvider>
        <TextSelectionHandler>
          <ChatWidget sessionId={globalSessionId} />
        </TextSelectionHandler>
      </ChatProvider>
    </ConfigProvider>
  );
}

export default ChatInjector;