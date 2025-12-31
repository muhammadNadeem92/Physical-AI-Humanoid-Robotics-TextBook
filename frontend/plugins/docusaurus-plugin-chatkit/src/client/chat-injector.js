import React from 'react';
import { useLocation } from '@docusaurus/router';
import { ChatProvider } from '../../../../src/contexts/ChatContext';
import { ConfigProvider } from '../../../../src/contexts/ConfigContext';
import ChatWidget from '../../../../src/components/ChatWidget';
import TextSelectionHandler from '../../../../src/components/TextSelectionHandler';

// Get configuration from Docusaurus global data
let config = {};
if (typeof window !== 'undefined') {
  // Try to get from global data set by the plugin
  config = window?.__DOCUSAURUS_GLOBAL_DATA__?.['docusaurus-plugin-chatkit/src/index']?.chatkit || {};

  // Fallback to window.chatkitConfig if not found in global data
  if (!config.apiBaseUrl) {
    config = { ...config, ...window.chatkitConfig };
  }

  // Set default if still not available
  if (!config.apiBaseUrl) {
    config.apiBaseUrl = '/api/v1';
  }
}

// Default configuration with plugin config
const defaultConfig = {
  apiBaseUrl: config.apiBaseUrl || '/api/v1',
  sessionTimeout: config.sessionTimeout || 30,
  widgetPosition: config.position || 'bottom-right',
  showWidgetOnMobile: config.showOnMobile !== undefined ? config.showOnMobile : true,
  maxSelectedTextLength: config.maxSelectedTextLength || 1000,
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