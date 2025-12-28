/**
 * Chat service for handling chat-related API communication
 */
import { getApiService, ChatQueryRequest, SelectedTextQueryRequest } from './api';
import { Configuration } from '../types/config';
import { Message } from '../types/message';
import { Citation } from '../types/citation';
import { extractCitationsFromResponse } from '../utils/citations';

class ChatService {
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
   * Send a regular chat query to the backend
   */
  async sendChatQuery(
    question: string,
    sessionId: string,
    module?: string,
    chapter?: string
  ): Promise<Message> {
    const apiService = getApiService(this.config);

    const request: ChatQueryRequest = {
      question,
      session_id: sessionId,
      ...(module && { module }),
      ...(chapter && { chapter }),
      stream: false
    };

    try {
      const response = await apiService.chatQuery(request);

      // Create the assistant message with citations
      const citations: Citation[] = response.citations.map((url, index) => ({
        id: `cit_${Date.now()}_${index}`,
        messageId: `msg_${Date.now()}_assistant`,
        sourceUrl: url,
        chapter: 'Unknown', // Will be parsed from URL if possible
        section: 'Unknown',
        snippet: `Source: ${url}`,
        pageUrl: url
      }));

      const message: Message = {
        id: `msg_${Date.now()}_assistant`,
        sessionId: response.session_id,
        content: response.response_text,
        sender: 'assistant',
        timestamp: new Date(response.timestamp),
        status: 'sent',
        citations
      };

      return message;
    } catch (error) {
      throw new Error(`Failed to send chat query: ${error instanceof Error ? error.message : 'Unknown error'}`);
    }
  }

  /**
   * Send a selected text query to the backend
   */
  async sendSelectedTextQuery(
    selectedText: string,
    question: string,
    sessionId: string
  ): Promise<Message> {
    const apiService = getApiService(this.config);

    const request: SelectedTextQueryRequest = {
      selected_text: selectedText,
      question,
      session_id: sessionId,
      stream: false
    };

    try {
      const response = await apiService.selectedTextQuery(request);

      // Create the assistant message with citations
      const citations: Citation[] = response.citations.map((url, index) => ({
        id: `cit_${Date.now()}_${index}`,
        messageId: `msg_${Date.now()}_assistant`,
        sourceUrl: url,
        chapter: 'Unknown', // Will be parsed from URL if possible
        section: 'Unknown',
        snippet: `Source: ${url}`,
        pageUrl: url
      }));

      const message: Message = {
        id: `msg_${Date.now()}_assistant`,
        sessionId: response.session_id,
        content: response.response_text,
        sender: 'assistant',
        timestamp: new Date(response.timestamp),
        status: 'sent',
        citations
      };

      return message;
    } catch (error) {
      throw new Error(`Failed to send selected text query: ${error instanceof Error ? error.message : 'Unknown error'}`);
    }
  }

  /**
   * Get chat history for a session
   */
  async getChatHistory(sessionId: string): Promise<any[]> {
    const apiService = getApiService(this.config);

    try {
      const response = await apiService.getChatHistory(sessionId);
      return response.history;
    } catch (error) {
      throw new Error(`Failed to get chat history: ${error instanceof Error ? error.message : 'Unknown error'}`);
    }
  }

  /**
   * Submit feedback for a response
   */
  async submitFeedback(
    sessionId: string,
    queryId: string,
    responseId: string,
    feedbackType: string,
    feedbackText?: string
  ): Promise<boolean> {
    const apiService = getApiService(this.config);

    try {
      const response = await apiService.submitFeedback({
        session_id: sessionId,
        query_id: queryId,
        response_id: responseId,
        feedback_type: feedbackType,
        feedback_text: feedbackText
      });

      return response.success;
    } catch (error) {
      throw new Error(`Failed to submit feedback: ${error instanceof Error ? error.message : 'Unknown error'}`);
    }
  }

  /**
   * Get session statistics
   */
  async getSessionStats(sessionId: string): Promise<any> {
    const apiService = getApiService(this.config);

    try {
      return await apiService.getSessionStats(sessionId);
    } catch (error) {
      throw new Error(`Failed to get session stats: ${error instanceof Error ? error.message : 'Unknown error'}`);
    }
  }

  /**
   * Create a streaming chat query
   */
  async createStreamingChatQuery(
    question: string,
    sessionId: string,
    onMessage: (chunk: string) => void,
    onCitations?: (citations: string[]) => void,
    onError?: (error: string) => void
  ): Promise<void> {
    const apiService = getApiService(this.config);

    const request: ChatQueryRequest = {
      question,
      session_id: sessionId,
      stream: true
    };

    try {
      const reader = await apiService.createStreamingRequest('/chat/query', request);

      // Process the stream
      const decoder = new TextDecoder();
      let buffer = '';

      while (true) {
        const { done, value } = await reader.read();
        if (done) break;

        buffer += decoder.decode(value, { stream: true });

        // Process complete lines (SSE format)
        const lines = buffer.split('\n');
        buffer = lines.pop() || ''; // Keep incomplete line in buffer

        for (const line of lines) {
          if (line.startsWith('data: ')) {
            try {
              const data = JSON.parse(line.slice(6)); // Remove 'data: ' prefix

              if (data.type === 'text' && data.content) {
                onMessage(data.content);
              } else if (data.type === 'citations' && onCitations) {
                onCitations(data.content);
              } else if (data.type === 'error' && onError) {
                onError(data.message);
              }
            } catch (e) {
              // Ignore malformed JSON
            }
          }
        }
      }
    } catch (error) {
      if (onError) {
        onError(error instanceof Error ? error.message : 'Unknown error');
      } else {
        throw error;
      }
    }
  }

  /**
   * Create a streaming selected text query
   */
  async createStreamingSelectedTextQuery(
    selectedText: string,
    question: string,
    sessionId: string,
    onMessage: (chunk: string) => void,
    onCitations?: (citations: string[]) => void,
    onError?: (error: string) => void
  ): Promise<void> {
    const apiService = getApiService(this.config);

    const request: SelectedTextQueryRequest = {
      selected_text: selectedText,
      question,
      session_id: sessionId,
      stream: true
    };

    try {
      const reader = await apiService.createStreamingRequest('/chat/selected-text', request);

      // Process the stream
      const decoder = new TextDecoder();
      let buffer = '';

      while (true) {
        const { done, value } = await reader.read();
        if (done) break;

        buffer += decoder.decode(value, { stream: true });

        // Process complete lines (SSE format)
        const lines = buffer.split('\n');
        buffer = lines.pop() || ''; // Keep incomplete line in buffer

        for (const line of lines) {
          if (line.startsWith('data: ')) {
            try {
              const data = JSON.parse(line.slice(6)); // Remove 'data: ' prefix

              if (data.type === 'text' && data.content) {
                onMessage(data.content);
              } else if (data.type === 'citations' && onCitations) {
                onCitations(data.content);
              } else if (data.type === 'error' && onError) {
                onError(data.message);
              }
            } catch (e) {
              // Ignore malformed JSON
            }
          }
        }
      }
    } catch (error) {
      if (onError) {
        onError(error instanceof Error ? error.message : 'Unknown error');
      } else {
        throw error;
      }
    }
  }
}

let chatService: ChatService;

export const getChatService = (config: Configuration): ChatService => {
  if (!chatService) {
    chatService = new ChatService(config);
  } else {
    chatService.updateConfig(config);
  }
  return chatService;
};

export default ChatService;