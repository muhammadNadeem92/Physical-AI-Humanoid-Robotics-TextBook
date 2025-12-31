/**
 * TypeScript interface for Configuration entity
 */
export interface Configuration {
  apiBaseUrl: string;
  sessionTimeout: number;
  widgetPosition: 'bottom-right' | 'bottom-left' | 'top-right' | 'top-left';
  showWidgetOnMobile: boolean;
  maxSelectedTextLength: number;
}

/**
 * TypeScript interface for Configuration updates
 */
export interface UpdateConfigParams {
  apiBaseUrl?: string;
  sessionTimeout?: number;
  widgetPosition?: 'bottom-right' | 'bottom-left' | 'top-right' | 'top-left';
  showWidgetOnMobile?: boolean;
  maxSelectedTextLength?: number;
}