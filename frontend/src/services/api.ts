/**
 * API service for chat-related operations
 */

// Types
export interface ChatQueryRequest {
  question: string;
  session_id: string;
  module?: string;
  chapter?: string;
  stream: boolean;
}

export interface SelectedTextQueryRequest {
  selected_text: string;
  question: string;
  session_id: string;
  stream: boolean;
}

export interface FeedbackRequest {
  session_id: string;
  query_id: string;
  response_id: string;
  feedback_type: string;
  feedback_text?: string;
}

export interface Configuration {
  apiBaseUrl: string;
  timeout?: number;
}

// Main API service class
class ApiService {
  private config: Configuration;

  constructor(config: Configuration) {
    this.config = config;
  }

  /**
   * Update configuration
   */
  updateConfig(config: Configuration): void {
    this.config = config;
  }

  /**
   * Send a chat query to the backend
   */
  async chatQuery(request: ChatQueryRequest): Promise<any> {
    const response = await fetch(`${this.config.apiBaseUrl}/chat/query`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(request),
    });

    if (!response.ok) {
      throw new Error(`Chat query failed: ${response.status} ${response.statusText}`);
    }

    return await response.json();
  }

  /**
   * Send a selected text query to the backend
   */
  async selectedTextQuery(request: SelectedTextQueryRequest): Promise<any> {
    const response = await fetch(`${this.config.apiBaseUrl}/chat/selected-text`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(request),
    });

    if (!response.ok) {
      throw new Error(`Selected text query failed: ${response.status} ${response.statusText}`);
    }

    return await response.json();
  }

  /**
   * Get chat history for a session
   */
  async getChatHistory(sessionId: string): Promise<any> {
    const response = await fetch(`${this.config.apiBaseUrl}/chat/history/${sessionId}`, {
      method: 'GET',
      headers: {
        'Content-Type': 'application/json',
      },
    });

    if (!response.ok) {
      throw new Error(`Get chat history failed: ${response.status} ${response.statusText}`);
    }

    return await response.json();
  }

  /**
   * Submit feedback for a response
   */
  async submitFeedback(request: FeedbackRequest): Promise<any> {
    const response = await fetch(`${this.config.apiBaseUrl}/chat/feedback`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(request),
    });

    if (!response.ok) {
      throw new Error(`Submit feedback failed: ${response.status} ${response.statusText}`);
    }

    return await response.json();
  }

  /**
   * Get session statistics
   */
  async getSessionStats(sessionId: string): Promise<any> {
    const response = await fetch(`${this.config.apiBaseUrl}/chat/stats/${sessionId}`, {
      method: 'GET',
      headers: {
        'Content-Type': 'application/json',
      },
    });

    if (!response.ok) {
      throw new Error(`Get session stats failed: ${response.status} ${response.statusText}`);
    }

    return await response.json();
  }

  /**
   * Create a streaming request
   */
  async createStreamingRequest(endpoint: string, request: any): Promise<ReadableStreamDefaultReader> {
    const response = await fetch(`${this.config.apiBaseUrl}${endpoint}`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(request),
    });

    if (!response.ok) {
      throw new Error(`Streaming request failed: ${response.status} ${response.statusText}`);
    }

    if (!response.body) {
      throw new Error('No response body for streaming request');
    }

    return response.body.getReader();
  }
}

let apiService: ApiService;

export const getApiService = (config: Configuration): ApiService => {
  if (!apiService) {
    apiService = new ApiService(config);
  } else {
    apiService.updateConfig(config);
  }
  return apiService;
};

export default ApiService;