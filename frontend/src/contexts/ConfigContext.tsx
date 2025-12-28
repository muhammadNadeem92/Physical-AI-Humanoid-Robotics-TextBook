import React, { createContext, useContext, useState, ReactNode, useEffect } from 'react';
import { Configuration } from '../types/config';

// Define the shape of our config context
interface ConfigContextType {
  config: Configuration;
  updateConfig: (newConfig: Partial<Configuration>) => void;
  resetConfig: () => void;
}

// Helper function to safely get environment variables in browser
const getEnvVar = (key: string, defaultValue: string): string => {
  if (typeof window !== 'undefined' && (window as any).env) {
    return (window as any).env[key] || defaultValue;
  }
  return defaultValue;
};

const getEnvVarAsNumber = (key: string, defaultValue: number): number => {
  const value = getEnvVar(key, defaultValue.toString());
  const parsed = parseInt(value, 10);
  return isNaN(parsed) ? defaultValue : parsed;
};

const getEnvVarAsBoolean = (key: string, defaultValue: boolean): boolean => {
  const value = getEnvVar(key, defaultValue.toString());
  return value === 'true';
};

// Default configuration values
const defaultConfig: Configuration = {
  apiBaseUrl: getEnvVar('BACKEND_API_BASE_URL', '/api/v1'),
  sessionTimeout: getEnvVarAsNumber('CHATKIT_SESSION_TIMEOUT', 30),
  widgetPosition: (getEnvVar('CHATKIT_WIDGET_POSITION', 'bottom-right') as 'bottom-right' | 'bottom-left' | 'top-right' | 'top-left') || 'bottom-right',
  showWidgetOnMobile: getEnvVarAsBoolean('CHATKIT_SHOW_ON_MOBILE', true),
  maxSelectedTextLength: getEnvVarAsNumber('CHATKIT_MAX_SELECTED_TEXT_LENGTH', 1000),
};

// Create context
const ConfigContext = createContext<ConfigContextType | undefined>(undefined);

// Provider component
interface ConfigProviderProps {
  children: ReactNode;
  initialConfig?: Partial<Configuration>;
}

export const ConfigProvider: React.FC<ConfigProviderProps> = ({ children, initialConfig = {} }) => {
  const [config, setConfig] = useState<Configuration>(() => {
    // Merge default config with initial config and environment variables
    return {
      ...defaultConfig,
      ...initialConfig,
    };
  });

  // Update config when environment variables change (in case they load asynchronously)
  useEffect(() => {
    setConfig(prevConfig => ({
      ...prevConfig,
      apiBaseUrl: getEnvVar('BACKEND_API_BASE_URL', prevConfig.apiBaseUrl),
      sessionTimeout: getEnvVarAsNumber('CHATKIT_SESSION_TIMEOUT', prevConfig.sessionTimeout),
      widgetPosition: (getEnvVar('CHATKIT_WIDGET_POSITION', prevConfig.widgetPosition) as 'bottom-right' | 'bottom-left' | 'top-right' | 'top-left') || prevConfig.widgetPosition,
      showWidgetOnMobile: getEnvVarAsBoolean('CHATKIT_SHOW_ON_MOBILE', prevConfig.showWidgetOnMobile),
      maxSelectedTextLength: getEnvVarAsNumber('CHATKIT_MAX_SELECTED_TEXT_LENGTH', prevConfig.maxSelectedTextLength),
    }));
  }, []);

  // Update config function
  const updateConfig = (newConfig: Partial<Configuration>) => {
    setConfig(prevConfig => ({
      ...prevConfig,
      ...newConfig,
    }));
  };

  // Reset config to defaults
  const resetConfig = () => {
    setConfig(defaultConfig);
  };

  const value: ConfigContextType = {
    config,
    updateConfig,
    resetConfig,
  };

  return <ConfigContext.Provider value={value}>{children}</ConfigContext.Provider>;
};

// Custom hook to use the ConfigContext
export const useConfigContext = (): ConfigContextType => {
  const context = useContext(ConfigContext);
  if (context === undefined) {
    throw new Error('useConfigContext must be used within a ConfigProvider');
  }
  return context;
};

// HOC to provide config to class components
export const withConfig = <P extends object>(Component: React.ComponentType<P & ConfigContextType>): React.FC<P> => {
  return (props: P) => {
    const configContext = useConfigContext();
    return <Component {...props} {...configContext} />;
  };
};